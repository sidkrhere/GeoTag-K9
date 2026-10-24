import { Point2D, GeoStop, GeoTag, SignalMeasurement, SimulationConfig } from '../src/types/simulation';
import { CanineKalmanFilter } from '../src/engine/kalmanFilter';

function distance(p1: Point2D, p2: Point2D): number {
  return Math.hypot(p1.x - p2.x, p1.y - p2.y);
}

function findTwoCircleIntersectionsClean(
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

  // Exact geometric tangency when circles are disjoint
  if (d >= r1 + r2) {
    const scale = r1 / (r1 + r2);
    const p0x = a1.x + scale * (a2.x - a1.x);
    const p0y = a1.y + scale * (a2.y - a1.y);
    return [{ x: p0x, y: p0y }, { x: p0x, y: p0y }];
  }

  // One circle inside another
  if (d <= Math.abs(r1 - r2)) {
    const scale = r1 > r2 ? (d + r2) / d : (r1 / d);
    const p0x = a1.x + scale * (a2.x - a1.x);
    const p0y = a1.y + scale * (a2.y - a1.y);
    return [{ x: p0x, y: p0y }, { x: p0x, y: p0y }];
  }

  const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
  const h2 = r1 * r1 - a * a;
  const h = Math.sqrt(Math.max(0, h2));

  const p0x = a1.x + (a / d) * (a2.x - a1.x);
  const p0y = a1.y + (a / d) * (a2.y - a1.y);

  const rx = -(a2.y - a1.y) * (h / d);
  const ry = (a2.x - a1.x) * (h / d);

  return [
    { x: p0x + rx, y: p0y + ry },
    { x: p0x - rx, y: p0y - ry }
  ];
}

function testDisambiguation(
  c1: Point2D,
  c2: Point2D,
  priorPt: Point2D,
  expectedForwardPos: Point2D,
  uCompass: Point2D,
  ppm: number,
  mode: 'QUADRATIC_MAP' | 'HIERARCHICAL_GATE',
  c1InBuilding: boolean = false,
  c2InBuilding: boolean = false
): { chosen: Point2D; outcast: Point2D; validatedIdx: 0 | 1; reason: string } {
  const chordM = distance(c1, c2) / ppm;

  // 1. Narrow Chord Collinear Midpoint (<= 1.5m)
  if (chordM <= 1.5) {
    const mid: Point2D = {
      x: (c1.x + c2.x) / 2,
      y: (c1.y + c2.y) / 2
    };
    return {
      chosen: mid,
      outcast: c2,
      validatedIdx: 0,
      reason: `Collinear Midpoint: Coincident baseline transition (Δ${chordM.toFixed(1)}m chord)`
    };
  }

  // Evaluate candidate function
  const evaluate = (cand: Point2D, inBuilding: boolean) => {
    const dxPred = (cand.x - expectedForwardPos.x) / ppm;
    const dyPred = (cand.y - expectedForwardPos.y) / ppm;
    const distPredM = Math.hypot(dxPred, dyPred);

    let distScore: number;
    if (mode === 'QUADRATIC_MAP') {
      // Option A: Continuous Quadratic MAP
      // Soft at close range (3d + 2d^2), steep (>400) for distant leaps
      distScore = 3.0 * distPredM + 2.0 * (distPredM * distPredM);
      if (distPredM > 2.5) {
        distScore += 8.0 * Math.pow(distPredM - 2.5, 1.5);
      }
    } else {
      // Option B: Hierarchical Gate base score for tie breaker
      distScore = 4.5 * distPredM;
    }

    const dxStep = (cand.x - priorPt.x) / ppm;
    const dyStep = (cand.y - priorPt.y) / ppm;
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

  const res1 = evaluate(c1, c1InBuilding);
  const res2 = evaluate(c2, c2InBuilding);

  let validatedIdx: 0 | 1;
  let reason: string;

  if (c1InBuilding !== c2InBuilding) {
    validatedIdx = c1InBuilding ? 1 : 0;
    reason = `Map Occlusion: Outcast ghost in building wall (Δ${chordM.toFixed(1)}m chord)`;
  } else if (mode === 'HIERARCHICAL_GATE') {
    // Option B: Hierarchical Spatial Gate
    // If distance difference > 1.5m, closer candidate wins unconditionally
    if (res1.distM < res2.distM - 1.5) {
      validatedIdx = 0;
      reason = `Spatial Gate: Closer candidate d=${res1.distM.toFixed(1)}m vs ${res2.distM.toFixed(1)}m (Δ${chordM.toFixed(1)}m chord)`;
    } else if (res2.distM < res1.distM - 1.5) {
      validatedIdx = 1;
      reason = `Spatial Gate: Closer candidate d=${res2.distM.toFixed(1)}m vs ${res1.distM.toFixed(1)}m (Δ${chordM.toFixed(1)}m chord)`;
    } else {
      validatedIdx = res1.score <= res2.score ? 0 : 1;
      reason = `Collinear Tie-Breaker: J=${Math.round((validatedIdx === 0 ? res1.score : res2.score) * 10) / 10} vs ${Math.round((validatedIdx === 0 ? res2.score : res1.score) * 10) / 10} (Align: +${(validatedIdx === 0 ? res1.align : res2.align).toFixed(2)})`;
    }
  } else {
    // Option A: Continuous Quadratic MAP
    validatedIdx = res1.score <= res2.score ? 0 : 1;
    reason = `Quadratic MAP: J=${Math.round((validatedIdx === 0 ? res1.score : res2.score) * 10) / 10} vs ${Math.round((validatedIdx === 0 ? res2.score : res1.score) * 10) / 10} (d=${(validatedIdx === 0 ? res1.distM : res2.distM).toFixed(1)}m vs ${(validatedIdx === 0 ? res2.distM : res1.distM).toFixed(1)}m)`;
  }

  let chosen = validatedIdx === 0 ? c1 : c2;
  let outcast = validatedIdx === 0 ? c2 : c1;
  const chosenEval = validatedIdx === 0 ? res1 : res2;
  const outcastEval = validatedIdx === 0 ? res2 : res1;

  // Heading Sanity & Latch-up Breaker:
  // If the chosen candidate is translating backwards against collar spine (align < -0.2)
  // while the outcast is moving forward (align > 0.4) and displacement is significant:
  if (chordM >= 3.5 && chosenEval.align < -0.2 && outcastEval.align > 0.4) {
    chosen = outcast;
    outcast = validatedIdx === 0 ? c1 : c2;
    validatedIdx = validatedIdx === 0 ? 1 : 0;
    reason = `Latch-Up Breaker: Realigned to collar heading (align ${outcastEval.align.toFixed(2)} vs ${chosenEval.align.toFixed(2)})`;
  }

  return { chosen, outcast, validatedIdx, reason };
}

// Run baseline crossing test suite
function runBaselineSuite(mode: 'QUADRATIC_MAP' | 'HIERARCHICAL_GATE') {
  console.log(`\n========================================================`);
  console.log(`  RUNNING BASELINE SUITE: MODE = ${mode}`);
  console.log(`========================================================`);

  const stop1: GeoStop = { id: 'stop-1', name: 'Stop 1', tier: 1, x: 200, y: 300, isOnline: true };
  const stop2: GeoStop = { id: 'stop-2', name: 'Stop 2', tier: 1, x: 600, y: 300, isOnline: true };
  const ppm = 2.8;

  function runSim(testName: string, startPos: Point2D, headingDeg: number, speedMps: number, totalSteps: number) {
    const headingRad = (headingDeg * Math.PI) / 180;
    const ux = Math.sin(headingRad);
    const uy = -Math.cos(headingRad);
    const stepPix = speedMps * ppm * 0.050;

    const tag: Point2D = { ...startPos };
    const kf = new CanineKalmanFilter(startPos, ppm, 16.0);
    let priorPt = { ...startPos };
    let priorTimestamp = 1700000000000;

    let maxErrorM = 0;
    let totalErrorM = 0;
    let falseFlips = 0;

    for (let step = 1; step <= totalSteps; step++) {
      tag.x += ux * stepPix;
      tag.y += uy * stepPix;
      const now = 1700000000000 + step * 50;

      kf.predict(now, headingRad);

      const d1Pix = Math.hypot(tag.x - stop1.x, tag.y - stop1.y);
      const d2Pix = Math.hypot(tag.x - stop2.x, tag.y - stop2.y);
      const noise1M = (Math.random() - 0.5) * 1.5;
      const noise2M = (Math.random() - 0.5) * 1.5;

      const r1 = Math.max(0.5, d1Pix + noise1M * ppm);
      const r2 = Math.max(0.5, d2Pix + noise2M * ppm);

      const [c1, c2] = findTwoCircleIntersectionsClean(stop1, r1, stop2, r2);

      const rawDt = (now - priorTimestamp) / 1000.0;
      const dtSec = Math.max(0.02, Math.min(0.20, rawDt));
      const canineSpeedPix = (speedMps * ppm) * dtSec;
      const uCompass = { x: ux, y: uy };
      const expectedForwardPos: Point2D = {
        x: priorPt.x + uCompass.x * canineSpeedPix,
        y: priorPt.y + uCompass.y * canineSpeedPix
      };

      const res = testDisambiguation(c1, c2, priorPt, expectedForwardPos, uCompass, ppm, mode);

      const chordM = distance(c1, c2) / ppm;
      const measStdDevM = Math.max(1.8, Math.min(5.0, chordM * 0.25));
      const kfRes = kf.update(res.chosen, measStdDevM);
      priorPt = { ...kfRes.pos };
      priorTimestamp = now;

      const errM = distance(kfRes.pos, tag) / ppm;
      totalErrorM += errM;
      if (errM > maxErrorM) maxErrorM = errM;
      if (errM > 10.0) falseFlips++;
    }

    const avgErrM = totalErrorM / totalSteps;
    console.log(`  ${testName}: Avg=${avgErrM.toFixed(2)}m, Max=${maxErrorM.toFixed(2)}m, Flips=${falseFlips} -> ${falseFlips === 0 ? 'PASS' : 'FAIL'}`);
    return falseFlips === 0;
  }

  const p1 = runSim('TEST 1: Perpendicular (90° South across Y=300)', { x: 400, y: 240 }, 180, 2.5, 300);
  const p2 = runSim('TEST 2: Shallow Diagonal (10° across baseline)', { x: 280, y: 290 }, 100, 2.5, 350);
  const p3 = runSim('TEST 3: 45° Diagonal (South-East)', { x: 300, y: 250 }, 135, 3.0, 250);
  const p4 = runSim('TEST 4: Pure Parallel (alongside baseline)', { x: 250, y: 286 }, 90, 2.5, 200);

  return p1 && p2 && p3 && p4;
}

const passA = runBaselineSuite('QUADRATIC_MAP');
const passB = runBaselineSuite('HIERARCHICAL_GATE');

console.log('\n========================================================');
console.log('QUADRATIC_MAP Baseline Suite PASS:', passA);
console.log('HIERARCHICAL_GATE Baseline Suite PASS:', passB);
console.log('========================================================');
