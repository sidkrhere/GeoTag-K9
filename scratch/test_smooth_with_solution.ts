import { Point2D, GeoStop, GeoTag, SignalMeasurement, SimulationConfig } from '../src/types/simulation';
import { CanineKalmanFilter } from '../src/engine/kalmanFilter';
import { solve2AnchorTest } from './test_solution';

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

const config: SimulationConfig = {
  tier1RadiusMeters: 120,
  tier2RadiusMeters: 60,
  pathLossExponent: 2.2,
  wallAttenuationDb: 18,
  noiseStdDevDb: 2.0,
  autoWalkSpeed: 2.5,
  pixelsPerMeter: 2.8,
  showTrueDoggos: true,
  showEstimatedLocations: true,
  showCoverageHalos: true,
  showBackhaulBeams: true,
  showRFRipples: true,
  showHeadingVisionCone: true,
  showHistoricalBreadcrumbs: true,
  imuSensorType: 'ECOMPASS_9DOF'
};

const stop1: GeoStop = { id: 'stop-1', name: 'Stop 1', tier: 1, x: 200, y: 300, isOnline: true };
const stop2: GeoStop = { id: 'stop-2', name: 'Stop 2', tier: 1, x: 600, y: 300, isOnline: true };

function runTest(mode: 'QUADRATIC_MAP' | 'HIERARCHICAL_GATE') {
  console.log(`\n=================================================================`);
  console.log(`  RUNNING BASELINE CROSSING TESTS: MODE = ${mode}`);
  console.log(`=================================================================`);

  function simulate(
    testName: string,
    startPos: Point2D,
    headingDeg: number,
    speedMps: number,
    totalSteps: number,
    dtSec: number = 0.050
  ) {
    const headingRad = (headingDeg * Math.PI) / 180;
    const ux = Math.sin(headingRad);
    const uy = -Math.cos(headingRad);
    const stepPix = speedMps * config.pixelsPerMeter * dtSec;

    const tag: Point2D = { x: startPos.x, y: startPos.y };
    const kf = new CanineKalmanFilter(startPos, config.pixelsPerMeter, 16.0);
    let priorPt = { ...startPos };
    let priorTimestamp = 1700000000000;

    let maxErrorM = 0;
    let totalErrorM = 0;
    let falseFlips = 0;

    for (let step = 1; step <= totalSteps; step++) {
      tag.x += ux * stepPix;
      tag.y += uy * stepPix;
      const now = 1700000000000 + step * 50;

      // Predict
      kf.predict(now, headingRad);

      const d1Pix = Math.hypot(tag.x - stop1.x, tag.y - stop1.y);
      const d2Pix = Math.hypot(tag.x - stop2.x, tag.y - stop2.y);
      const noise1M = (Math.random() - 0.5) * 1.5;
      const noise2M = (Math.random() - 0.5) * 1.5;

      const r1 = Math.max(0.5, d1Pix + noise1M * config.pixelsPerMeter);
      const r2 = Math.max(0.5, d2Pix + noise2M * config.pixelsPerMeter);

      const [c1, c2] = findTwoCircleIntersections(stop1, r1, stop2, r2);
      const dtSecVal = 0.050;
      const canineSpeedPix = (speedMps * config.pixelsPerMeter) * dtSecVal;
      const uCompass = { x: ux, y: uy };

      const { chosen } = solve2AnchorTest(
        c1,
        c2,
        priorPt,
        uCompass,
        canineSpeedPix,
        config.pixelsPerMeter,
        mode
      );

      const kfRes = kf.update(chosen, 2.0);
      priorPt = { ...kfRes.pos };

      const errM = distance(kfRes.pos, tag) / config.pixelsPerMeter;
      totalErrorM += errM;
      if (errM > maxErrorM) maxErrorM = errM;
      if (errM > 10.0) falseFlips++;
    }

    const avgErrM = totalErrorM / totalSteps;
    console.log(`  ${testName}: Avg=${avgErrM.toFixed(2)}m, Max=${maxErrorM.toFixed(2)}m, Flips=${falseFlips} -> ${falseFlips === 0 ? 'PASS' : 'FAIL'}`);
    return falseFlips === 0;
  }

  const p1 = simulate('TEST 1: Perpendicular (90° South across Y=300)', { x: 400, y: 240 }, 180, 2.5, 300);
  const p2 = simulate('TEST 2: Shallow Diagonal (10° across baseline)', { x: 280, y: 290 }, 100, 2.5, 350);
  const p3 = simulate('TEST 3: 45° Diagonal (South-East)', { x: 300, y: 250 }, 135, 3.0, 250);
  const p4 = simulate('TEST 4: Pure Parallel (alongside baseline)', { x: 250, y: 286 }, 90, 2.5, 200);

  return p1 && p2 && p3 && p4;
}

const allPassA = runTest('QUADRATIC_MAP');
const allPassB = runTest('HIERARCHICAL_GATE');

console.log('\n=== FINAL VERDICT ===');
console.log('QUADRATIC_MAP All Pass:', allPassA);
console.log('HIERARCHICAL_GATE All Pass:', allPassB);
