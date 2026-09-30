import { Point2D, GeoStop, GeoTag, SignalMeasurement, TrilaterationResult, SimulationConfig, BuildingPolygon, LockType } from '../types/simulation';
import { distance, isPointInPolygon } from './geometry';

const lastConfirmedFixMap = new Map<string, { pos: Point2D; timestamp: number; anchorCount: number }>();

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
  lastConfirmedFixMap.set(tagId, { pos: { x: pos.x, y: pos.y }, timestamp: Date.now(), anchorCount: 0 });
  transitionStateMap.delete(tagId);
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
  initialGuess: Point2D
): Point2D {
  let estX = initialGuess.x;
  let estY = initialGuess.y;

  const maxIter = 15;
  const damping = 0.8;

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

      j11 += jx * jx;
      j12 += jx * jy;
      j22 += jy * jy;

      r1 += jx * residual;
      r2 += jy * residual;
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
 * Master Multilateration Solver with 2-Anchor Pinpoint e-Compass Vector Matching.
 */
export function solveTrilateration(
  tag: GeoTag,
  measurements: SignalMeasurement[],
  stopsMap: Map<string, GeoStop>,
  config: SimulationConfig,
  buildings: BuildingPolygon[] = []
): TrilaterationResult {
  const now = Date.now();

  // Reset any outlier flag on measurements
  for (const m of measurements) {
    m.isOutlier = false;
  }

  // Collect valid anchors
  const validAnchors: { id: string; point: Point2D; radius: number; measurement: SignalMeasurement }[] = [];
  for (const m of measurements) {
    const stop = stopsMap.get(m.stopId);
    if (!stop) continue;
    validAnchors.push({
      id: stop.id,
      point: { x: stop.x, y: stop.y },
      radius: m.noisyDistanceMeters * config.pixelsPerMeter,
      measurement: m
    });
  }

  // Case 0: 0 Anchors in range (Blind Zone)
  if (validAnchors.length === 0) {
    return {
      tagId: tag.id,
      estimatedX: tag.x,
      estimatedY: tag.y,
      errorMeters: 0,
      lockType: 'BLIND_ZONE',
      contributingAnchorIds: [],
      gdop: 99.9,
      timestamp: now
    };
  }

  // Case 1: Exactly 1 Anchor in range (Radial Ring)
  if (validAnchors.length === 1) {
    const a = validAnchors[0];
    const radialError = Math.abs(a.measurement.noisyDistanceMeters - a.measurement.trueDistanceMeters);
    lastConfirmedFixMap.set(tag.id, {
      pos: { x: a.point.x, y: a.point.y },
      timestamp: now,
      anchorCount: 1
    });

    return {
      tagId: tag.id,
      estimatedX: a.point.x,
      estimatedY: a.point.y,
      errorMeters: Math.round(radialError * 10) / 10,
      lockType: 'RADIAL_RING',
      radialRing: {
        center: a.point,
        radiusPixels: a.radius,
        radiusMeters: Math.round(a.measurement.noisyDistanceMeters * 10) / 10,
        stopId: a.id
      },
      contributingAnchorIds: [a.id],
      gdop: 99.9,
      timestamp: now
    };
  }

  // Case 2: Exactly 2 Anchors in range (2-Anchor Pinpoint e-Compass Vector Matching)
  if (validAnchors.length === 2) {
    const a1 = validAnchors[0];
    const a2 = validAnchors[1];

    const [cand1, cand2] = findTwoCircleIntersections(a1.point, a1.radius, a2.point, a2.radius);

    const cand1InBuilding = buildings.some(b => b.closed && isPointInPolygon(cand1, b.points));
    const cand2InBuilding = buildings.some(b => b.closed && isPointInPolygon(cand2, b.points));

    // Retrieve prior confirmed fix (e.g. from 3-anchor exit or previous frame)
    const prior = lastConfirmedFixMap.get(tag.id);
    let trans = transitionStateMap.get(tag.id);

    // Detect transition into 2-anchor zone (e.g. from 3+ anchors or 1 anchor)
    if (!trans || trans.prevAnchorCount !== 2) {
      trans = {
        prevAnchorCount: 2,
        transitionStartMs: now,
        entryFix: prior?.pos ?? { x: tag.x, y: tag.y }
      };
      transitionStateMap.set(tag.id, trans);
    }

    const elapsedTransitionMs = now - trans.transitionStartMs;
    const isTransitionBuffering = elapsedTransitionMs < 140; // 120ms-160ms observational window

    // During the transition buffer, refPos stays anchored at entryFix so the dog's displacement
    // vector grows to ~25-40 cm, guaranteeing an unambiguous cosine similarity delta.
    // Once settled, refPos tracks the confirmed fix.
    const refPos = isTransitionBuffering ? trans.entryFix : (prior?.pos ?? trans.entryFix);

    // Dorsal e-Compass Unit Heading Vector:
    // 0° = North (0, -1), 90° = East (1, 0), 180° = South (0, 1), 270° = West (-1, 0)
    const headingDeg = tag.headingDeg ?? 90;
    const headingRad = (headingDeg * Math.PI) / 180;
    const uCompass: Point2D = {
      x: Math.sin(headingRad),
      y: -Math.cos(headingRad)
    };

    // Candidate displacement vectors w1, w2 from reference position:
    const w1 = { x: cand1.x - refPos.x, y: cand1.y - refPos.y };
    const w2 = { x: cand2.x - refPos.x, y: cand2.y - refPos.y };
    const len1 = Math.sqrt(w1.x * w1.x + w1.y * w1.y);
    const len2 = Math.sqrt(w2.x * w2.x + w2.y * w2.y);

    // Vector Cosine Similarity (Dot Product with e-Compass spine vector):
    const align1 = len1 > 0.5 ? (w1.x * uCompass.x + w1.y * uCompass.y) / len1 : 1.0;
    const align2 = len2 > 0.5 ? (w2.x * uCompass.x + w2.y * uCompass.y) / len2 : 1.0;

    // Chord leap distance in meters
    const leap1M = len1 / config.pixelsPerMeter;
    const leap2M = len2 / config.pixelsPerMeter;

    // Score candidates:
    // 1. Building occlusion is a hard physical impossibility
    // 2. High positive cosine alignment (+0.8 to +1.0) is a massive forward reward
    // 3. Negative alignment (reversing across baseline) is severely penalized
    // 4. Chord leap penalty: jumping > 4m across the chord when parallel to baseline is penalized
    function scoreCandidate(inBuilding: boolean, alignment: number, leapM: number): number {
      if (inBuilding) return 100000.0;
      const leapPenalty = Math.max(0, leapM - 4.0) * 18.0;
      const alignmentScore = -alignment * 60.0; // lower score is better
      return alignmentScore + leapPenalty;
    }

    const score1 = scoreCandidate(cand1InBuilding, align1, leap1M);
    const score2 = scoreCandidate(cand2InBuilding, align2, leap2M);

    const validatedIdx: 0 | 1 = score1 <= score2 ? 0 : 1;
    const chosen = validatedIdx === 0 ? cand1 : cand2;
    const outcast = validatedIdx === 0 ? cand2 : cand1;
    const chosenAlign = validatedIdx === 0 ? align1 : align2;
    const outcastAlign = validatedIdx === 0 ? align2 : align1;

    const separationChordMeters = Math.round((distance(cand1, cand2) / config.pixelsPerMeter) * 10) / 10;
    const errMeters = Math.round((distance({ x: tag.x, y: tag.y }, chosen) / config.pixelsPerMeter) * 10) / 10;

    // Save confirmed fix
    lastConfirmedFixMap.set(tag.id, {
      pos: { ...chosen },
      timestamp: now,
      anchorCount: 2
    });

    const isPinpoint = chosenAlign >= 0.4 && !buildings.some(b => b.closed && isPointInPolygon(chosen, b.points));
    const lockType: LockType = isPinpoint ? 'DUAL_PINPOINT_LOCK' : 'DUAL_KINEMATIC_LOCK';

    const reason = cand1InBuilding !== cand2InBuilding
      ? `Map Occlusion: Outcast ghost in building wall (Δ${separationChordMeters}m chord)`
      : isPinpoint
      ? `2-Anchor Pinpoint: e-Compass Vector Match (${Math.round(chosenAlign * 100)}% alignment vs ${Math.round(outcastAlign * 100)}% ghost)`
      : `Kinematic Proximity (Chord: ${separationChordMeters}m)`;

    return {
      tagId: tag.id,
      estimatedX: chosen.x,
      estimatedY: chosen.y,
      errorMeters: errMeters,
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
        candidate1ErrorMeters: errMeters,
        candidate2ErrorMeters: Math.round((distance({ x: tag.x, y: tag.y }, outcast) / config.pixelsPerMeter) * 10) / 10,
        candidate1VectorAlignment: Math.round(chosenAlign * 100) / 100,
        candidate2VectorAlignment: Math.round(outcastAlign * 100) / 100,
        compassHeadingDeg: headingDeg,
        transitionDelayMs: isTransitionBuffering ? Math.max(0, 140 - elapsedTransitionMs) : 0,
        isTransitionBuffering
      },
      contributingAnchorIds: [a1.id, a2.id],
      gdop: 4.8,
      timestamp: now
    };
  }

  // Case 3: 3 or more Anchors (Gauss-Newton Multilateration)
  const centroidX = validAnchors.reduce((sum, a) => sum + a.point.x, 0) / validAnchors.length;
  const centroidY = validAnchors.reduce((sum, a) => sum + a.point.y, 0) / validAnchors.length;

  const solved = solveGaussNewton(
    validAnchors.map(a => a.point),
    validAnchors.map(a => a.radius),
    { x: centroidX, y: centroidY }
  );

  lastConfirmedFixMap.set(tag.id, {
    pos: { x: solved.x, y: solved.y },
    timestamp: now,
    anchorCount: validAnchors.length
  });

  transitionStateMap.set(tag.id, {
    prevAnchorCount: validAnchors.length,
    transitionStartMs: now,
    entryFix: { x: solved.x, y: solved.y }
  });

  const errMeters = Math.round((distance({ x: tag.x, y: tag.y }, solved) / config.pixelsPerMeter) * 10) / 10;
  const gdop = calculateGDOP(validAnchors.map(a => a.point), solved);

  return {
    tagId: tag.id,
    estimatedX: solved.x,
    estimatedY: solved.y,
    errorMeters: errMeters,
    lockType: 'FULL_3D_LOCK',
    contributingAnchorIds: validAnchors.map(a => a.id),
    gdop,
    timestamp: now
  };
}
