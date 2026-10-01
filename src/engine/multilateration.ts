import { Point2D, GeoStop, GeoTag, SignalMeasurement, TrilaterationResult, SimulationConfig, BuildingPolygon, LockType, BorderExitInfo, HistoricalFix } from '../types/simulation';
import { distance, signedCrossProduct, isDeepInsidePolygon } from './geometry';
import { CanineKalmanFilter } from './kalmanFilter';

const lastConfirmedFixMap = new Map<string, { pos: Point2D; timestamp: number; anchorCount: number; velocity: Point2D }>();
const kalmanFilterMap = new Map<string, CanineKalmanFilter>();
const borderExitStateMap = new Map<string, BorderExitInfo>();
const dogHistoricalFixMap = new Map<string, HistoricalFix[]>();
const MAX_HISTORY_SAMPLES = 15;

/**
 * Records a confirmed localization fix into the dog's sliding-window historical cache.
 */
export function recordHistoricalFix(tagId: string, fix: HistoricalFix): HistoricalFix[] {
  let history = dogHistoricalFixMap.get(tagId);
  if (!history) {
    history = [];
    dogHistoricalFixMap.set(tagId, history);
  }

  history.push(fix);
  if (history.length > MAX_HISTORY_SAMPLES) {
    history.shift();
  }

  return [...history];
}

/**
 * Retrieves the historical trajectory buffer for a dog.
 */
export function getHistoricalTrajectory(tagId: string): HistoricalFix[] {
  return [...(dogHistoricalFixMap.get(tagId) || [])];
}

/**
 * Computes exponentially weighted moving average velocity across historical trajectory.
 */
export function computeHistoricalVelocity(history: HistoricalFix[]): Point2D {
  if (history.length === 0) return { x: 0, y: 0 };
  if (history.length === 1) return { x: history[0].vx, y: history[0].vy };

  let totalVx = 0;
  let totalVy = 0;
  let totalWeight = 0;
  const beta = 0.8; // Exponential decay factor (more recent fixes have higher weight)

  for (let i = history.length - 1; i >= 0; i--) {
    const age = (history.length - 1) - i;
    const weight = Math.pow(beta, age);
    totalVx += history[i].vx * weight;
    totalVy += history[i].vy * weight;
    totalWeight += weight;
  }

  return {
    x: totalWeight > 0 ? totalVx / totalWeight : 0,
    y: totalWeight > 0 ? totalVy / totalWeight : 0
  };
}

interface TransitionState {
  prevAnchorCount: number;
  transitionStartMs: number;
  entryFix: Point2D;
}

const transitionStateMap = new Map<string, TransitionState>();

/**
 * Resets any tracking state for a dog.
 */
export function resetDogTrajectory(tagId: string, pos: Point2D): void {
  lastConfirmedFixMap.set(tagId, { pos: { x: pos.x, y: pos.y }, timestamp: Date.now(), anchorCount: 0, velocity: { x: 0, y: 0 } });
  transitionStateMap.delete(tagId);
  borderExitStateMap.delete(tagId);
  dogHistoricalFixMap.delete(tagId);
  const kf = kalmanFilterMap.get(tagId);
  if (kf) {
    kf.reset(pos);
  }
}

/**
 * Calculates Geometric Dilution of Precision (GDOP) for an anchor constellation.
 */
export function calculateGDOP(anchors: Point2D[], target: Point2D): number {
  if (anchors.length < 3) return 99.9;

  const H: number[][] = [];
  for (const a of anchors) {
    const d = Math.max(0.1, distance(target, a));
    H.push([(target.x - a.x) / d, (target.y - a.y) / d]);
  }

  let a11 = 0, a12 = 0, a21 = 0, a22 = 0;
  for (let i = 0; i < H.length; i++) {
    a11 += H[i][0] * H[i][0];
    a12 += H[i][0] * H[i][1];
    a21 += H[i][1] * H[i][0];
    a22 += H[i][1] * H[i][1];
  }

  const det = a11 * a22 - a12 * a21;
  if (Math.abs(det) < 1e-5) return 99.9;

  const invA11 = a22 / det;
  const invA22 = a11 / det;
  const trace = invA11 + invA22;
  return trace > 0 ? Math.min(99.9, Math.round(Math.sqrt(trace) * 100) / 100) : 99.9;
}

/**
 * Analytical intersection of two circles. Returns 2 intersection points.
 */
export function findTwoCircleIntersections(
  a1: Point2D,
  r1: number,
  a2: Point2D,
  r2: number
): [Point2D, Point2D] {
  const d = distance(a1, a2);
  if (d < 1e-4) {
    return [
      { x: a1.x + r1, y: a1.y },
      { x: a1.x - r1, y: a1.y }
    ];
  }

  let effectiveR1 = r1;
  let effectiveR2 = r2;
  if (d > r1 + r2) {
    const scale = (d / (r1 + r2)) * 1.002;
    effectiveR1 *= scale;
    effectiveR2 *= scale;
  } else if (d < Math.abs(r1 - r2)) {
    if (r1 > r2) effectiveR1 = d + r2 - 0.001;
    else effectiveR2 = d + r1 - 0.001;
  }

  const a = (effectiveR1 * effectiveR1 - effectiveR2 * effectiveR2 + d * d) / (2 * d);
  const h = Math.sqrt(Math.max(0, effectiveR1 * effectiveR1 - a * a));

  const p0x = a1.x + (a / d) * (a2.x - a1.x);
  const p0y = a1.y + (a / d) * (a2.y - a1.y);

  const rx = -(a2.y - a1.y) * (h / d);
  const ry = (a2.x - a1.x) * (h / d);

  return [
    { x: p0x + rx, y: p0y + ry },
    { x: p0x - rx, y: p0y - ry }
  ];
}

/**
 * Standard Nonlinear Least Squares (Gauss-Newton) Multilateration.
 * Solves: min sum_i ( ||P - A_i|| - r_i )^2
 */
function solveGaussNewton(
  anchors: Point2D[],
  radii: number[],
  initialGuess: Point2D,
  ppm: number = 2.8
): Point2D {
  let estX = initialGuess.x;
  let estY = initialGuess.y;

  const maxIter = 15;
  const damping = 0.8;
  const huberThreshold = 10.0 * ppm; // 10 meters outlier limit

  for (let iter = 0; iter < maxIter; iter++) {
    let j11 = 0, j12 = 0, j22 = 0;
    let r1 = 0, r2 = 0;

    for (let i = 0; i < anchors.length; i++) {
      const dx = estX - anchors[i].x;
      const dy = estY - anchors[i].y;
      const d = Math.max(0.1, Math.sqrt(dx * dx + dy * dy));
      const residual = d - radii[i];

      const jx = dx / d;
      const jy = dy / d;

      // Robust Huber M-estimator: downweights outliers with extreme residuals
      const absR = Math.abs(residual);
      const weight = absR <= huberThreshold ? 1.0 : huberThreshold / absR;
      const wR = residual * weight;

      j11 += jx * jx * weight;
      j12 += jx * jy * weight;
      j22 += jy * jy * weight;

      r1 += jx * wR;
      r2 += jy * wR;
    }

    const det = j11 * j22 - j12 * j12;
    if (Math.abs(det) < 1e-6) break;

    const deltaX = -(j22 * r1 - j12 * r2) / det;
    const deltaY = -(-j12 * r1 + j11 * r2) / det;

    estX += deltaX * damping;
    estY += deltaY * damping;

    if (deltaX * deltaX + deltaY * deltaY < 0.01) break;
  }

  return { x: estX, y: estY };
}

/**
 * Master Multilateration Solver with DVB-KF State Space & Present-Instant NLOS Zero-Weighting.
 */
export function solveTrilateration(
  tag: GeoTag,
  measurements: SignalMeasurement[],
  stopsMap: Map<string, GeoStop>,
  config: SimulationConfig,
  buildings: BuildingPolygon[] = []
): TrilaterationResult {
  const now = Date.now();

  // Dynamically compute maxSpeedMps scaled proportionally with simulation speed
  const simMaxSpeedMps = Math.max(45.0, (config.autoWalkSpeed ?? 1.5) * 32.0);

  // Retrieve or initialize the Dynamic Velocity-Bounded Kalman Filter for this canine
  let kf = kalmanFilterMap.get(tag.id);
  if (!kf) {
    kf = new CanineKalmanFilter({ x: tag.x, y: tag.y }, config.pixelsPerMeter, simMaxSpeedMps);
    kalmanFilterMap.set(tag.id, kf);
  } else {
    kf.setMaxSpeedMps(simMaxSpeedMps);
  }

  // Live actual dog speed in meters per second
  const actualSpeedMps = tag.speedMps !== undefined 
    ? tag.speedMps 
    : Math.round((Math.hypot(tag.vx, tag.vy) / 0.033 / config.pixelsPerMeter) * 10) / 10;

  const prior = lastConfirmedFixMap.get(tag.id);
  const headingDeg = tag.headingDeg ?? 90;
  const headingRad = (headingDeg * Math.PI) / 180;

  // 1. Predict Present-Instant Location using DVB-KF with Collar Heading Guidance
  const predPos = kf.predict(now, headingRad);

  // 2. GeoCentre NLOS Zero-Weighting:
  // Identify true 15" concrete wall NLOS anchors vs clean LOS anchors.
  // Crucial rule: If the physical RF measurement has m.obstructed === false (clear physical path),
  // it is genuine Line-of-Sight and must NEVER be falsely marked as NLOS.
  for (const m of measurements) {
    m.isOutlier = false;
    m.isNlosZeroWeighted = false;
    m.weight = 1.0;

    const stop = stopsMap.get(m.stopId);
    if (!stop) continue;

    // True physical wall penetration test:
    if (m.obstructed) {
      // Signal genuinely passed through 15" concrete wall (severe attenuation)
      m.isNlosZeroWeighted = true;
      m.weight = 0.0; // Strictly zero weight!
    } else {
      // Physical path is 100% clear LOS!
      m.isNlosZeroWeighted = false;
      m.weight = 1.0;
    }
  }

  // 3. Partition anchors into clean inliers vs zero-weighted NLOS
  const validAnchors: { id: string; point: Point2D; radius: number; measurement: SignalMeasurement }[] = [];
  const zeroWeightedAnchors: { id: string; point: Point2D; measurement: SignalMeasurement }[] = [];

  for (const m of measurements) {
    const stop = stopsMap.get(m.stopId);
    if (!stop) continue;

    if (m.isNlosZeroWeighted) {
      zeroWeightedAnchors.push({
        id: stop.id,
        point: { x: stop.x, y: stop.y },
        measurement: m
      });
    } else {
      validAnchors.push({
        id: stop.id,
        point: { x: stop.x, y: stop.y },
        radius: m.noisyDistanceMeters * config.pixelsPerMeter,
        measurement: m
      });
    }
  }

  // Case 0: 0 Clean Anchors in Range (Border Exit / Blind Zone)
  if (validAnchors.length === 0) {
    let borderExit = borderExitStateMap.get(tag.id);
    if (!borderExit && prior && prior.anchorCount > 0) {
      const kfState = kf.getState();
      borderExit = {
        lastKnownPos: { ...prior.pos },
        exitVelocity: { ...kfState.velocity },
        exitHeadingDeg: headingDeg,
        exitSpeedMps: kfState.speedMps,
        exitTimestamp: prior.timestamp,
        elapsedSeconds: Math.max(0.1, Math.round(((now - prior.timestamp) / 1000) * 10) / 10),
        previousAnchorCount: prior.anchorCount
      };
      borderExitStateMap.set(tag.id, borderExit);
    } else if (borderExit) {
      borderExit.elapsedSeconds = Math.max(0.1, Math.round(((now - borderExit.exitTimestamp) / 1000) * 10) / 10);
    }

    const fix: HistoricalFix = {
      x: borderExit ? borderExit.lastKnownPos.x : tag.x,
      y: borderExit ? borderExit.lastKnownPos.y : tag.y,
      timestamp: now,
      vx: borderExit ? borderExit.exitVelocity.x : 0,
      vy: borderExit ? borderExit.exitVelocity.y : 0,
      speedMps: borderExit ? borderExit.exitSpeedMps : 0,
      lockType: 'BLIND_ZONE',
      errorMeters: 0,
      headingDeg
    };
    const historyTrail = recordHistoricalFix(tag.id, fix);
    const historicalVelocity = computeHistoricalVelocity(historyTrail);

    return {
      tagId: tag.id,
      estimatedX: borderExit ? borderExit.lastKnownPos.x : tag.x,
      estimatedY: borderExit ? borderExit.lastKnownPos.y : tag.y,
      errorMeters: 0,
      lockType: 'BLIND_ZONE',
      contributingAnchorIds: [],
      outlierAnchorIds: zeroWeightedAnchors.map(a => a.id),
      borderExit,
      velocity: borderExit ? borderExit.exitVelocity : { x: 0, y: 0 },
      speedMps: borderExit ? borderExit.exitSpeedMps : 0,
      actualSpeedMps,
      historyTrail,
      historicalVelocity,
      covarianceTrace: kf.getCovarianceTrace(),
      headingDeg,
      gdop: 99.9,
      timestamp: now
    };
  }

  // Clear any border exit state once back in anchor coverage
  borderExitStateMap.delete(tag.id);

  // Case 1: Exactly 1 Clean Anchor in range (Radial Ring)
  if (validAnchors.length === 1) {
    const a = validAnchors[0];
    const radialError = Math.abs(a.measurement.noisyDistanceMeters - a.measurement.trueDistanceMeters);

    // Project predicted position onto the 1-anchor radial circle
    const dx = predPos.x - a.point.x;
    const dy = predPos.y - a.point.y;
    const d = Math.hypot(dx, dy) || 1;
    const ringPoint: Point2D = {
      x: a.point.x + (dx / d) * a.radius,
      y: a.point.y + (dy / d) * a.radius
    };

    const kfRes = kf.update(ringPoint, 2.5);

    lastConfirmedFixMap.set(tag.id, {
      pos: { x: kfRes.pos.x, y: kfRes.pos.y },
      timestamp: now,
      anchorCount: 1,
      velocity: kfRes.velocity
    });

    const fix: HistoricalFix = {
      x: kfRes.pos.x,
      y: kfRes.pos.y,
      timestamp: now,
      vx: kfRes.velocity.x,
      vy: kfRes.velocity.y,
      speedMps: kfRes.speedMps,
      lockType: 'RADIAL_RING',
      errorMeters: Math.round(radialError * 10) / 10,
      headingDeg
    };
    const historyTrail = recordHistoricalFix(tag.id, fix);
    const historicalVelocity = computeHistoricalVelocity(historyTrail);

    return {
      tagId: tag.id,
      estimatedX: kfRes.pos.x,
      estimatedY: kfRes.pos.y,
      errorMeters: Math.round(radialError * 10) / 10,
      lockType: 'RADIAL_RING',
      radialRing: {
        center: a.point,
        radiusPixels: a.radius,
        radiusMeters: Math.round(a.measurement.noisyDistanceMeters * 10) / 10,
        stopId: a.id
      },
      contributingAnchorIds: [a.id],
      outlierAnchorIds: zeroWeightedAnchors.map(a => a.id),
      velocity: kfRes.velocity,
      speedMps: kfRes.speedMps,
      actualSpeedMps,
      historyTrail,
      historicalVelocity,
      covarianceTrace: kf.getCovarianceTrace(),
      headingDeg,
      gdop: 99.9,
      timestamp: now
    };
  }

  // Case 2: Exactly 2 Clean Anchors in range (Orthogonal 3D Bayesian MAP Fusion)
  if (validAnchors.length === 2) {
    const a1 = validAnchors[0];
    const a2 = validAnchors[1];

    const [cand1, cand2] = findTwoCircleIntersections(a1.point, a1.radius, a2.point, a2.radius);

    const cand1InBuilding = buildings.some(b => b.closed && isDeepInsidePolygon(cand1, b.points, 12));
    const cand2InBuilding = buildings.some(b => b.closed && isDeepInsidePolygon(cand2, b.points, 12));

    // Dorsal Spine e-Compass Unit Heading Vector (strictly pointing along canine motion axis):
    // 0° = North (0, -1), 90° = East (1, 0), 180° = South (0, 1), 270° = West (-1, 0)
    const uCompass: Point2D = {
      x: Math.sin(headingRad),
      y: -Math.cos(headingRad)
    };

    // Prior confirmed position (or initial tag position if cold boot)
    const priorPt = prior?.pos ?? { x: tag.x, y: tag.y };
    const separationChordMeters = Math.round((distance(cand1, cand2) / config.pixelsPerMeter) * 10) / 10;

    // Baseline half-plane geometry: The directed line from anchor 1 to anchor 2
    // separates the two circle intersection candidates into opposite half-planes.
    const side1 = signedCrossProduct(a1.point, a2.point, cand1);
    const side2 = signedCrossProduct(a1.point, a2.point, cand2);
    const priorSide = signedCrossProduct(a1.point, a2.point, priorPt);

    const isSameSide1 = Math.sign(side1) === Math.sign(priorSide) || Math.abs(priorSide) < 1e-4;
    const isSameSide2 = Math.sign(side2) === Math.sign(priorSide) || Math.abs(priorSide) < 1e-4;

    // Evaluator balancing Vector Cosine Alignment and Prior Proximity:
    // 1. Vector Cosine Alignment: rewards candidate aligned with dorsal motion vector
    // 2. Kinematic Distance Penalty: penalizes leap distance from prior position
    // 3. Baseline Half-Plane Guard: blocks teleporting across wide chords (> 8m)
    // 4. Deep Building Occlusion: penalizes true interior ghosts, but ignores sidewalk jitter
    function evaluateCandidate(cand: Point2D, inBuilding: boolean, isSameSide: boolean) {
      const dx = cand.x - priorPt.x;
      const dy = cand.y - priorPt.y;
      const distPix = Math.hypot(dx, dy);
      const distM = distPix / config.pixelsPerMeter;

      // 1. Vector Cosine Alignment
      const align = distPix > 0.5 ? (dx * uCompass.x + dy * uCompass.y) / distPix : 1.0;
      const alignScore = -align * 50.0;

      // 2. Distance from Prior Fix (3.5 per meter)
      const distScore = distM * 3.5;

      // 3. Baseline Half-Plane Guard: only active when chord is wide (> 8m)
      const chordPenalty = (!isSameSide && separationChordMeters > 8.0) ? (separationChordMeters * 6.0) : 0.0;

      // 4. Deep Building Wall Penalty (only for points > 3.5m inside the building envelope)
      const wallScore = inBuilding ? 40.0 : 0.0;

      const score = alignScore + distScore + chordPenalty + wallScore;
      return { score, align, distM };
    }

    const res1 = evaluateCandidate(cand1, cand1InBuilding, isSameSide1);
    const res2 = evaluateCandidate(cand2, cand2InBuilding, isSameSide2);

    const validatedIdx: 0 | 1 = res1.score <= res2.score ? 0 : 1;
    const chosen = validatedIdx === 0 ? cand1 : cand2;
    const outcast = validatedIdx === 0 ? cand2 : cand1;
    const chosenEval = validatedIdx === 0 ? res1 : res2;
    const outcastEval = validatedIdx === 0 ? res2 : res1;

    // Update Kalman filter directly with validated candidate (zero lag!)
    const kfRes = kf.update(chosen, 1.8);
    // Blue crosshair error directly linked to validated candidate
    const blueCrosshairErrMeters = Math.round((distance({ x: tag.x, y: tag.y }, chosen) / config.pixelsPerMeter) * 10) / 10;

    // Save confirmed fix
    lastConfirmedFixMap.set(tag.id, {
      pos: { ...kfRes.pos },
      timestamp: now,
      anchorCount: 2,
      velocity: kfRes.velocity
    });

    transitionStateMap.set(tag.id, {
      prevAnchorCount: 2,
      transitionStartMs: now,
      entryFix: { ...kfRes.pos }
    });

    const lockType: LockType = 'DUAL_PINPOINT_LOCK';

    const reason = cand1InBuilding !== cand2InBuilding
      ? `Map Occlusion: Outcast ghost in building wall (Δ${separationChordMeters}m chord)`
      : `Fused Pinpoint: Alignment +${Math.round(chosenEval.align * 100) / 100} vs ${Math.round(outcastEval.align * 100) / 100} (Δ${Math.round(chosenEval.distM * 10) / 10}m vs Δ${Math.round(outcastEval.distM * 10) / 10}m)`;

    const fix: HistoricalFix = {
      x: kfRes.pos.x,
      y: kfRes.pos.y,
      timestamp: now,
      vx: kfRes.velocity.x,
      vy: kfRes.velocity.y,
      speedMps: kfRes.speedMps,
      lockType,
      errorMeters: blueCrosshairErrMeters,
      headingDeg
    };
    const historyTrail = recordHistoricalFix(tag.id, fix);
    const historicalVelocity = computeHistoricalVelocity(historyTrail);

    return {
      tagId: tag.id,
      estimatedX: kfRes.pos.x,
      estimatedY: kfRes.pos.y,
      errorMeters: blueCrosshairErrMeters,
      lockType,
      dualCandidates: [cand1, cand2],
      dualInfo: {
        candidate1: chosen,
        candidate2: outcast,
        validatedIndex: validatedIdx,
        outcastIndex: validatedIdx === 0 ? 1 : 0,
        outcastLeapMeters: separationChordMeters,
        separationChordMeters,
        reason,
        candidate1ErrorMeters: blueCrosshairErrMeters,
        candidate2ErrorMeters: Math.round((distance({ x: tag.x, y: tag.y }, outcast) / config.pixelsPerMeter) * 10) / 10,
        candidate1VectorAlignment: Math.round(chosenEval.align * 100) / 100,
        candidate2VectorAlignment: Math.round(outcastEval.align * 100) / 100,
        candidate1Score: Math.round(chosenEval.score * 100) / 100,
        candidate2Score: Math.round(outcastEval.score * 100) / 100,
        candidate1DistMeters: Math.round(chosenEval.distM * 10) / 10,
        candidate2DistMeters: Math.round(outcastEval.distM * 10) / 10,
        kinematicWeight: 0.70,
        compassWeight: 0.30,
        compassHeadingDeg: headingDeg,
        transitionDelayMs: 0,
        isTransitionBuffering: false
      },
      contributingAnchorIds: [a1.id, a2.id],
      outlierAnchorIds: zeroWeightedAnchors.map(a => a.id),
      velocity: kfRes.velocity,
      speedMps: kfRes.speedMps,
      actualSpeedMps,
      historyTrail,
      historicalVelocity,
      covarianceTrace: kf.getCovarianceTrace(),
      headingDeg,
      gdop: 3.2,
      timestamp: now
    };
  }

  // Case 3: 3 or more Clean Anchors (Gauss-Newton Multilateration)
  const centroidX = validAnchors.reduce((sum, a) => sum + a.point.x, 0) / validAnchors.length;
  const centroidY = validAnchors.reduce((sum, a) => sum + a.point.y, 0) / validAnchors.length;

  // Use present-instant predicted location as initial guess if available, otherwise anchor centroid
  const initialGuess = (predPos && !isNaN(predPos.x)) ? predPos : { x: centroidX, y: centroidY };

  const solved = solveGaussNewton(
    validAnchors.map(a => a.point),
    validAnchors.map(a => a.radius),
    initialGuess,
    config.pixelsPerMeter
  );

  const gdop = calculateGDOP(validAnchors.map(a => a.point), solved);

  // Dynamically scale measurement uncertainty with live RF channel noise & GDOP!
  // Prevents Kalman filter from overconfidently swallowing wild noise spikes
  const baseNoiseM = Math.max(1.5, config.noiseStdDevDb * 0.85);
  const effectiveMeasStdDevM = baseNoiseM * Math.max(1.0, Math.min(3.5, gdop * 0.5));

  // Update Kalman filter with the dynamically scaled Gauss-Newton solution
  const kfRes = kf.update(solved, effectiveMeasStdDevM);

  lastConfirmedFixMap.set(tag.id, {
    pos: { x: kfRes.pos.x, y: kfRes.pos.y },
    timestamp: now,
    anchorCount: validAnchors.length,
    velocity: kfRes.velocity
  });

  transitionStateMap.set(tag.id, {
    prevAnchorCount: validAnchors.length,
    transitionStartMs: now,
    entryFix: { x: kfRes.pos.x, y: kfRes.pos.y }
  });

  const errMeters = Math.round((distance({ x: tag.x, y: tag.y }, kfRes.pos) / config.pixelsPerMeter) * 10) / 10;

  const fix: HistoricalFix = {
    x: kfRes.pos.x,
    y: kfRes.pos.y,
    timestamp: now,
    vx: kfRes.velocity.x,
    vy: kfRes.velocity.y,
    speedMps: kfRes.speedMps,
    lockType: 'FULL_3D_LOCK',
    errorMeters: errMeters,
    headingDeg
  };
  const historyTrail = recordHistoricalFix(tag.id, fix);
  const historicalVelocity = computeHistoricalVelocity(historyTrail);

  return {
    tagId: tag.id,
    estimatedX: kfRes.pos.x,
    estimatedY: kfRes.pos.y,
    errorMeters: errMeters,
    lockType: 'FULL_3D_LOCK',
    contributingAnchorIds: validAnchors.map(a => a.id),
    outlierAnchorIds: zeroWeightedAnchors.map(a => a.id),
    velocity: kfRes.velocity,
    speedMps: kfRes.speedMps,
    actualSpeedMps,
    historyTrail,
    historicalVelocity,
    covarianceTrace: kf.getCovarianceTrace(),
    headingDeg,
    gdop,
    timestamp: now
  };
}
