import { Point2D, GeoStop, GeoTag, SignalMeasurement, TrilaterationResult, SimulationConfig, BuildingPolygon, LockType, BorderExitInfo, HistoricalFix } from '../types/simulation';
import { distance, isDeepInsidePolygon } from './geometry';
import { CanineKalmanFilter } from './kalmanFilter';

const lastConfirmedFixMap = new Map<string, { pos: Point2D; timestamp: number; anchorCount: number; velocity: Point2D }>();
const kalmanFilterMap = new Map<string, CanineKalmanFilter>();
const borderExitStateMap = new Map<string, BorderExitInfo>();
const dogHistoricalFixMap = new Map<string, HistoricalFix[]>();
const anchorRangeFilterMap = new Map<string, Map<string, number>>();
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

  const vx = totalWeight > 0 ? totalVx / totalWeight : 0;
  const vy = totalWeight > 0 ? totalVy / totalWeight : 0;
  const speed = Math.hypot(vx, vy);
  const maxSpeedPix = 16.0 * 2.8; // 16.0 m/s physical canine agile dynamic headroom
  if (speed > maxSpeedPix && speed > 1e-4) {
    const scale = maxSpeedPix / speed;
    return { x: vx * scale, y: vy * scale };
  }

  return { x: vx, y: vy };
}

interface TransitionState {
  prevAnchorCount: number;
  transitionStartMs: number;
  entryFix: Point2D;
}

interface DualBranchState {
  b1: Point2D;
  b2: Point2D;
  timestamp: number;
}

interface BaselineSideState {
  anchor1Id: string;
  anchor2Id: string;
  hDeadReckonM: number;
  lastTimestamp: number;
}

const transitionStateMap = new Map<string, TransitionState>();
const dualBranchMap = new Map<string, DualBranchState>();
const baselineSideMap = new Map<string, BaselineSideState>();

/**
 * Resets any tracking state for a dog.
 */
export function resetDogTrajectory(tagId: string, pos: Point2D): void {
  lastConfirmedFixMap.set(tagId, { pos: { x: pos.x, y: pos.y }, timestamp: Date.now(), anchorCount: 0, velocity: { x: 0, y: 0 } });
  transitionStateMap.delete(tagId);
  borderExitStateMap.delete(tagId);
  dogHistoricalFixMap.delete(tagId);
  anchorRangeFilterMap.delete(tagId);
  dualBranchMap.delete(tagId);
  baselineSideMap.delete(tagId);
  const kf = kalmanFilterMap.get(tagId);
  if (kf) {
    kf.reset(pos);
  }
}

/**
 * Completely clears all persistent tracking state across all dogs.
 */
export function resetAllTrackingState(): void {
  lastConfirmedFixMap.clear();
  kalmanFilterMap.clear();
  transitionStateMap.clear();
  borderExitStateMap.clear();
  dogHistoricalFixMap.clear();
  anchorRangeFilterMap.clear();
  dualBranchMap.clear();
  baselineSideMap.clear();
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

    // Damped Levenberg-Marquardt regularization:
    // Prevents det -> 0 when anchors are nearly collinear along boundaries.
    const lambda = 0.25;
    const a11 = j11 + lambda;
    const a22 = j22 + lambda;
    const det = a11 * a22 - j12 * j12;
    if (Math.abs(det) < 1e-6) break;

    const rX = r1 + lambda * (estX - initialGuess.x);
    const rY = r2 + lambda * (estY - initialGuess.y);
    const deltaX = -(a22 * rX - j12 * rY) / det;
    const deltaY = -(-j12 * rX + a11 * rY) / det;

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
  buildings: BuildingPolygon[] = [],
  currentTimeMs?: number
): TrilaterationResult {
  const now = currentTimeMs ?? Date.now();

  // Canine biological kinematic ceiling: provide full dynamic bandwidth (16.0 m/s ceiling)
  // so the filter never drags behind during agile canine maneuvers while preventing runaway spikes.
  const canineTargetSpeedMps = config.autoWalkSpeed ?? 2.5;
  const simMaxSpeedMps = Math.max(16.0, canineTargetSpeedMps * 2.5);

  // Retrieve or initialize the Dynamic Velocity-Bounded Kalman Filter for this canine
  let kf = kalmanFilterMap.get(tag.id);
  if (!kf) {
    kf = new CanineKalmanFilter({ x: tag.x, y: tag.y }, config.pixelsPerMeter, simMaxSpeedMps);
    kalmanFilterMap.set(tag.id, kf);
  } else {
    kf.setMaxSpeedMps(simMaxSpeedMps);
  }

  // Live actual dog speed in meters per second (dt = 50ms interval)
  const actualSpeedMps = tag.speedMps !== undefined
    ? tag.speedMps
    : Math.round((Math.hypot(tag.vx, tag.vy) / 0.050 / config.pixelsPerMeter) * 10) / 10;

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
      // Innovation Gating & Exponential Noise Spike Suppression:
      // 1. Cap to physical radio horizon for this anchor
      const maxHorizonMeters = config.anchorRadiusMeters ?? (stop.tier === 1 ? config.tier1RadiusMeters : config.tier2RadiusMeters) ?? 90;
      const rawRadiusPix = Math.min(m.noisyDistanceMeters, maxHorizonMeters) * config.pixelsPerMeter;

      // 2. Innovation gating & rate-limiting against canine kinematic velocity
      let anchorCache = anchorRangeFilterMap.get(tag.id);
      if (!anchorCache) {
        anchorCache = new Map();
        anchorRangeFilterMap.set(tag.id, anchorCache);
      }
      let prevR = anchorCache.get(stop.id);
      if (prevR === undefined) {
        prevR = rawRadiusPix;
      }
      const maxDeltaPix = 3.0 * config.pixelsPerMeter; // 3.0m per tick = 60 m/s equivalent dynamic agility headroom
      const clampedR = Math.max(prevR - maxDeltaPix, Math.min(prevR + maxDeltaPix, rawRadiusPix));
      const smoothedR = prevR * 0.40 + clampedR * 0.60;
      anchorCache.set(stop.id, smoothedR);

      validAnchors.push({
        id: stop.id,
        point: { x: stop.x, y: stop.y },
        radius: smoothedR,
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

  // Case 2: Exactly 2 Clean Anchors in range (Orthogonal 3D Bayesian MAP Fusion & Dual-Branch MHT)
  if (validAnchors.length === 2) {
    const a1 = validAnchors[0];
    const a2 = validAnchors[1];

    const [raw1, raw2] = findTwoCircleIntersections(a1.point, a1.radius, a2.point, a2.radius);

    const raw1InBuilding = buildings.some(b => b.closed && isDeepInsidePolygon(raw1, b.points, 12));
    const raw2InBuilding = buildings.some(b => b.closed && isDeepInsidePolygon(raw2, b.points, 12));

    // Dorsal Spine e-Compass Unit Heading Vector (strictly pointing along canine motion axis):
    // 0° = North (0, -1), 90° = East (1, 0), 180° = South (0, 1), 270° = West (-1, 0)
    const uCompass: Point2D = {
      x: Math.sin(headingRad),
      y: -Math.cos(headingRad)
    };

    const separationChordMeters = Math.round((distance(raw1, raw2) / config.pixelsPerMeter) * 10) / 10;

    const rawDt = (now - (prior?.timestamp ?? (now - 50))) / 1000.0;
    const dtSec = Math.max(0.02, Math.min(0.20, rawDt));

    // Forward Kinematic Momentum Projection:
    const stepDistM = canineTargetSpeedMps * dtSec;
    const canineSpeedPix = stepDistM * config.pixelsPerMeter;

    // Baseline vector between the 2 anchors:
    const bx = a2.point.x - a1.point.x;
    const by = a2.point.y - a1.point.y;
    const bLen = Math.hypot(bx, by) || 1;
    const getH = (p: Point2D) => (bx * (p.y - a1.point.y) - by * (p.x - a1.point.x)) / bLen / config.pixelsPerMeter;

    // Cross-heading kinematic displacement across baseline:
    const deltaH = (bx * (uCompass.y * stepDistM) - by * (uCompass.x * stepDistM)) / bLen;

    // Persistent signed baseline side dead-reckoning:
    let sideState = baselineSideMap.get(tag.id);
    if (!sideState || sideState.anchor1Id !== a1.id || sideState.anchor2Id !== a2.id || (now - sideState.lastTimestamp > 1000)) {
      const seedPos = prior?.pos ?? { x: tag.x, y: tag.y };
      sideState = {
        anchor1Id: a1.id,
        anchor2Id: a2.id,
        hDeadReckonM: getH(seedPos),
        lastTimestamp: now
      };
      baselineSideMap.set(tag.id, sideState);
    } else {
      sideState.hDeadReckonM += deltaH;
      sideState.lastTimestamp = now;
    }
    const expectedSign = sideState.hDeadReckonM >= 0 ? 1 : -1;

    // Dual-Branch Multi-Hypothesis Association:
    let branchState = dualBranchMap.get(tag.id);
    let cand1 = raw1;
    let cand2 = raw2;
    let cand1InBuilding = raw1InBuilding;
    let cand2InBuilding = raw2InBuilding;

    if (!branchState || (now - branchState.timestamp > 1000)) {
      const hRaw1 = getH(raw1);
      const sign1 = hRaw1 >= 0 ? 1 : -1;
      if (sign1 === expectedSign) {
        cand1 = raw1; cand2 = raw2;
        cand1InBuilding = raw1InBuilding; cand2InBuilding = raw2InBuilding;
      } else {
        cand1 = raw2; cand2 = raw1;
        cand1InBuilding = raw2InBuilding; cand2InBuilding = raw1InBuilding;
      }
      branchState = { b1: cand1, b2: cand2, timestamp: now };
      dualBranchMap.set(tag.id, branchState);
    } else {
      const costKeep = distance(raw1, branchState.b1) + distance(raw2, branchState.b2);
      const costSwap = distance(raw1, branchState.b2) + distance(raw2, branchState.b1);
      if (costSwap < costKeep) {
        cand1 = raw2; cand2 = raw1;
        cand1InBuilding = raw2InBuilding; cand2InBuilding = raw1InBuilding;
      }
    }

    // Branch 1 Kinematic & Angular Evaluation:
    const v1x = (cand1.x - branchState.b1.x) / config.pixelsPerMeter;
    const v1y = (cand1.y - branchState.b1.y) / config.pixelsPerMeter;
    const s1M = Math.hypot(v1x, v1y);
    const align1 = s1M > 0.005 ? Math.max(-1.0, Math.min(1.0, (v1x * uCompass.x + v1y * uCompass.y) / s1M)) : 1.0;
    const exp1: Point2D = {
      x: branchState.b1.x + uCompass.x * canineSpeedPix,
      y: branchState.b1.y + uCompass.y * canineSpeedPix
    };
    const distPred1M = distance(cand1, exp1) / config.pixelsPerMeter;
    const h1 = getH(cand1);
    const diffH1 = Math.abs(h1 - sideState.hDeadReckonM);

    // Branch 2 Kinematic & Angular Evaluation:
    const v2x = (cand2.x - branchState.b2.x) / config.pixelsPerMeter;
    const v2y = (cand2.y - branchState.b2.y) / config.pixelsPerMeter;
    const s2M = Math.hypot(v2x, v2y);
    const align2 = s2M > 0.005 ? Math.max(-1.0, Math.min(1.0, (v2x * uCompass.x + v2y * uCompass.y) / s2M)) : 1.0;
    const exp2: Point2D = {
      x: branchState.b2.x + uCompass.x * canineSpeedPix,
      y: branchState.b2.y + uCompass.y * canineSpeedPix
    };
    const distPred2M = distance(cand2, exp2) / config.pixelsPerMeter;
    const h2 = getH(cand2);
    const diffH2 = Math.abs(h2 - sideState.hDeadReckonM);

    // Configuration & Cost Function:
    const disambiguationMode = config.disambiguationMode ?? 'QUADRATIC_MAP';
    const W_DIST = disambiguationMode === 'QUADRATIC_MAP' ? 3.0 : 4.5;
    const W_ANGLE = 25.0;

    const score1 = (3.0 * distPred1M + 2.0 * distPred1M * distPred1M) +
                   W_ANGLE * (1.0 - align1) +
                   (4.0 * diffH1 + 2.0 * diffH1 * diffH1) +
                   (cand1InBuilding ? 60.0 : 0.0);

    const score2 = (3.0 * distPred2M + 2.0 * distPred2M * distPred2M) +
                   W_ANGLE * (1.0 - align2) +
                   (4.0 * diffH2 + 2.0 * diffH2 * diffH2) +
                   (cand2InBuilding ? 60.0 : 0.0);

    // Decision Logic Gate:
    let validatedIdx: 0 | 1;
    let reason: string;

    if (cand1InBuilding !== cand2InBuilding) {
      validatedIdx = cand1InBuilding ? 1 : 0;
      reason = `Map Occlusion: Outcast ghost in building wall (Δ${separationChordMeters}m chord)`;
    } else if (disambiguationMode === 'HIERARCHICAL_GATE') {
      const sign1 = h1 >= 0 ? 1 : -1;
      const sign2 = h2 >= 0 ? 1 : -1;
      if (align1 >= 0.80 && align2 < 0.65) {
        validatedIdx = 0;
        reason = `Compass Alignment Gate: Forward heading lock (+${align1.toFixed(2)} vs ${align2.toFixed(2)})`;
      } else if (align2 >= 0.80 && align1 < 0.65) {
        validatedIdx = 1;
        reason = `Compass Alignment Gate: Forward heading lock (+${align2.toFixed(2)} vs ${align1.toFixed(2)})`;
      } else if (separationChordMeters > 1.8 && sign1 !== sign2) {
        validatedIdx = sign1 === expectedSign ? 0 : 1;
        reason = `Spatial Gate: Baseline side match (${(validatedIdx === 0 ? sign1 : sign2) === 1 ? 'Pos' : 'Neg'} half-plane, Δ${separationChordMeters}m chord)`;
      } else if (distPred1M < distPred2M - 1.2) {
        validatedIdx = 0;
        reason = `Spatial Gate: Closer kinematic displacement (${distPred1M.toFixed(1)}m vs ${distPred2M.toFixed(1)}m)`;
      } else if (distPred2M < distPred1M - 1.2) {
        validatedIdx = 1;
        reason = `Spatial Gate: Closer kinematic displacement (${distPred2M.toFixed(1)}m vs ${distPred1M.toFixed(1)}m)`;
      } else {
        validatedIdx = score1 <= score2 ? 0 : 1;
        reason = `Collinear Tie-Breaker: MAP cost J=${Math.round((validatedIdx === 0 ? score1 : score2) * 10) / 10}`;
      }
    } else {
      // Option A: Continuous Quadratic MAP
      validatedIdx = score1 <= score2 ? 0 : 1;
      reason = `Quadratic MAP: J=${Math.round((validatedIdx === 0 ? score1 : score2) * 10) / 10} vs ${Math.round((validatedIdx === 0 ? score2 : score1) * 10) / 10} (d=${(validatedIdx === 0 ? distPred1M : distPred2M).toFixed(1)}m, Δh=${(validatedIdx === 0 ? diffH1 : diffH2).toFixed(1)}m)`;
    }

    const chosen = validatedIdx === 0 ? cand1 : cand2;
    const outcast = validatedIdx === 0 ? cand2 : cand1;
    const chosenEval = { score: validatedIdx === 0 ? score1 : score2, align: validatedIdx === 0 ? align1 : align2, distM: validatedIdx === 0 ? distPred1M : distPred2M };
    const outcastEval = { score: validatedIdx === 0 ? score2 : score1, align: validatedIdx === 0 ? align2 : align1, distM: validatedIdx === 0 ? distPred2M : distPred1M };

    branchState.b1 = cand1;
    branchState.b2 = cand2;
    branchState.timestamp = now;

    // Dynamic measurement uncertainty scaling based on separation chord (GDOP)
    const effectiveMeasStdDevM = Math.max(1.8, Math.min(5.0, separationChordMeters * 0.25));
    const kfRes = kf.update(chosen, effectiveMeasStdDevM);

    // True system tracking error to Kalman estimated position
    const systemErrMeters = Math.round((distance({ x: tag.x, y: tag.y }, kfRes.pos) / config.pixelsPerMeter) * 10) / 10;

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

    const candidate1ErrMeters = Math.round((distance({ x: tag.x, y: tag.y }, chosen) / config.pixelsPerMeter) * 10) / 10;
    const candidate2ErrMeters = Math.round((distance({ x: tag.x, y: tag.y }, outcast) / config.pixelsPerMeter) * 10) / 10;

    const fix: HistoricalFix = {
      x: kfRes.pos.x,
      y: kfRes.pos.y,
      timestamp: now,
      vx: kfRes.velocity.x,
      vy: kfRes.velocity.y,
      speedMps: kfRes.speedMps,
      lockType,
      errorMeters: systemErrMeters,
      headingDeg
    };
    const historyTrail = recordHistoricalFix(tag.id, fix);
    const historicalVelocity = computeHistoricalVelocity(historyTrail);

    return {
      tagId: tag.id,
      estimatedX: kfRes.pos.x,
      estimatedY: kfRes.pos.y,
      errorMeters: systemErrMeters,
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
        candidate1ErrorMeters: candidate1ErrMeters,
        candidate2ErrorMeters: candidate2ErrMeters,
        candidate1VectorAlignment: Math.round(chosenEval.align * 100) / 100,
        candidate2VectorAlignment: Math.round(outcastEval.align * 100) / 100,
        candidate1Score: Math.round(chosenEval.score * 100) / 100,
        candidate2Score: Math.round(outcastEval.score * 100) / 100,
        candidate1DistMeters: Math.round(chosenEval.distM * 10) / 10,
        candidate2DistMeters: Math.round(outcastEval.distM * 10) / 10,
        kinematicWeight: W_DIST,
        compassWeight: W_ANGLE,
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
