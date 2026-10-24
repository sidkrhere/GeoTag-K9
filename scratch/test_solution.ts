import { Point2D, GeoStop, GeoTag, SignalMeasurement, SimulationConfig } from '../src/types/simulation';
import { CanineKalmanFilter } from '../src/engine/kalmanFilter';

function distance(p1: Point2D, p2: Point2D): number {
  return Math.hypot(p1.x - p2.x, p1.y - p2.y);
}

function findTwoCircleIntersections(p1: Point2D, r1: number, p2: Point2D, r2: number): [Point2D, Point2D] {
  const dx = p2.x - p1.x;
  const dy = p2.y - p1.y;
  const d = Math.hypot(dx, dy);

  if (d > r1 + r2) {
    const scale = r1 / (r1 + r2);
    const midX = p1.x + dx * scale;
    const midY = p1.y + dy * scale;
    return [{ x: midX, y: midY }, { x: midX, y: midY }];
  }

  if (d < Math.abs(r1 - r2)) {
    const rL = Math.max(r1, r2);
    const pL = r1 > r2 ? p1 : p2;
    const pS = r1 > r2 ? p2 : p1;
    const dirX = (pS.x - pL.x) / (d || 1);
    const dirY = (pS.y - pL.y) / (d || 1);
    const pt = { x: pL.x + dirX * rL, y: pL.y + dirY * rL };
    return [pt, pt];
  }

  const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
  const h2 = r1 * r1 - a * a;
  const h = Math.sqrt(Math.max(0, h2));

  const p2x = p1.x + (dx * a) / d;
  const p2y = p1.y + (dy * a) / d;

  return [
    { x: p2x + (h * dy) / d, y: p2y - (h * dx) / d },
    { x: p2x - (h * dy) / d, y: p2y + (h * dx) / d }
  ];
}

export function solve2AnchorTest(
  cand1: Point2D,
  cand2: Point2D,
  priorPt: Point2D,
  uCompass: Point2D,
  canineSpeedPix: number,
  pixelsPerMeter: number,
  mode: 'QUADRATIC_MAP' | 'HIERARCHICAL_GATE',
  cand1InBuilding: boolean = false,
  cand2InBuilding: boolean = false
): { chosen: Point2D; outcast: Point2D; reason: string } {
  const separationChordMeters = distance(cand1, cand2) / pixelsPerMeter;

  // Narrow-chord collinear midpoint: when chord <= 2.5m, range circles are near tangent.
  // Circle intersection suffers severe GDOP perpendicular to baseline. Midpoint P0 is rock solid.
  if (separationChordMeters <= 2.5) {
    const mid: Point2D = {
      x: (cand1.x + cand2.x) / 2,
      y: (cand1.y + cand2.y) / 2
    };
    return {
      chosen: mid,
      outcast: cand2,
      reason: `Collinear Midpoint: Coincident baseline transition (Δ${separationChordMeters.toFixed(1)}m chord)`
    };
  }

  const expectedForwardPos: Point2D = {
    x: priorPt.x + uCompass.x * canineSpeedPix,
    y: priorPt.y + uCompass.y * canineSpeedPix
  };

  const evaluate = (cand: Point2D, inBuilding: boolean) => {
    const dxPred = (cand.x - expectedForwardPos.x) / pixelsPerMeter;
    const dyPred = (cand.y - expectedForwardPos.y) / pixelsPerMeter;
    const distPredM = Math.hypot(dxPred, dyPred);

    let distScore: number;
    if (mode === 'QUADRATIC_MAP') {
      // Option A: Continuous Quadratic MAP
      distScore = 3.0 * distPredM + 2.0 * (distPredM * distPredM);
      if (distPredM > 2.5) {
        distScore += 8.0 * Math.pow(distPredM - 2.5, 1.5);
      }
    } else {
      // Option B: Base score for tie breaker
      distScore = 4.5 * distPredM;
    }

    const dxStep = (cand.x - priorPt.x) / pixelsPerMeter;
    const dyStep = (cand.y - priorPt.y) / pixelsPerMeter;
    const stepDistM = Math.hypot(dxStep, dyStep);
    let align = 1.0;
    let angleScore = 0.0;
    if (stepDistM > 0.15) {
      align = (dxStep * uCompass.x + dyStep * uCompass.y) / stepDistM;
      const boundedCos = Math.max(-1.0, Math.min(1.0, align));
      angleScore = 25.0 * (1.0 - boundedCos);
      if (stepDistM > 2.0 && boundedCos < -0.2) {
        angleScore += 60.0 * Math.abs(boundedCos);
      }
    }

    const wallScore = inBuilding ? 60.0 : 0.0;
    return { score: distScore + angleScore + wallScore, align, distM: distPredM };
  };

  const res1 = evaluate(cand1, cand1InBuilding);
  const res2 = evaluate(cand2, cand2InBuilding);

  let validatedIdx: 0 | 1;
  let reason: string;

  if (cand1InBuilding !== cand2InBuilding) {
    validatedIdx = cand1InBuilding ? 1 : 0;
    reason = `Map Occlusion: Outcast ghost in building wall (Δ${separationChordMeters.toFixed(1)}m chord)`;
  } else if (mode === 'HIERARCHICAL_GATE') {
    // Option B: Hierarchical Spatial Gate
    if (res1.distM < res2.distM - 1.5) {
      validatedIdx = 0;
      reason = `Spatial Gate: Closer candidate d=${res1.distM.toFixed(1)}m vs ${res2.distM.toFixed(1)}m (Δ${separationChordMeters.toFixed(1)}m chord)`;
    } else if (res2.distM < res1.distM - 1.5) {
      validatedIdx = 1;
      reason = `Spatial Gate: Closer candidate d=${res2.distM.toFixed(1)}m vs ${res1.distM.toFixed(1)}m (Δ${separationChordMeters.toFixed(1)}m chord)`;
    } else {
      validatedIdx = res1.score <= res2.score ? 0 : 1;
      reason = `Tie-Breaker: J=${Math.round((validatedIdx === 0 ? res1.score : res2.score) * 10) / 10} vs ${Math.round((validatedIdx === 0 ? res2.score : res1.score) * 10) / 10} (Align: +${(validatedIdx === 0 ? res1.align : res2.align).toFixed(2)})`;
    }
  } else {
    // Option A: Continuous Quadratic MAP
    validatedIdx = res1.score <= res2.score ? 0 : 1;
    reason = `Quadratic MAP: J=${Math.round((validatedIdx === 0 ? res1.score : res2.score) * 10) / 10} vs ${Math.round((validatedIdx === 0 ? res2.score : res1.score) * 10) / 10} (d=${(validatedIdx === 0 ? res1.distM : res2.distM).toFixed(1)}m vs ${(validatedIdx === 0 ? res2.distM : res1.distM).toFixed(1)}m)`;
  }

  let chosen = validatedIdx === 0 ? cand1 : cand2;
  let outcast = validatedIdx === 0 ? cand2 : cand1;
  const chosenEval = validatedIdx === 0 ? res1 : res2;
  const outcastEval = validatedIdx === 0 ? res2 : res1;

  // Latch-up Breaker:
  // If the filter latched onto the mirror side, the chosen candidate is moving backwards (align < 0.0)
  // while the real dog is moving forward (outcast align > 0.50).
  if (separationChordMeters >= 4.0 && chosenEval.align < 0.0 && outcastEval.align > 0.50) {
    chosen = outcast;
    outcast = validatedIdx === 0 ? cand1 : cand2;
    reason = `Latch-Up Breaker: Realigned to collar heading (align ${outcastEval.align.toFixed(2)} vs ${chosenEval.align.toFixed(2)})`;
  }

  return { chosen, outcast, reason };
}
