import { Point2D, GeoStop, GeoTag, SignalMeasurement, SimulationConfig } from '../src/types/simulation';
import { solveTrilateration, resetDogTrajectory, resetAllTrackingState } from '../src/engine/multilateration';

console.log('=================================================================');
console.log('  TEST: SMOOTH BASELINE CROSSINGS (PERPENDICULAR, SHALLOW, DIAGONAL)');
console.log('=================================================================\n');

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

// Baseline: Stop 1 at (200, 300), Stop 2 at (600, 300) -> Baseline along Y = 300
const stop1: GeoStop = { id: 'stop-1', name: 'Stop 1', tier: 1, x: 200, y: 300, isOnline: true };
const stop2: GeoStop = { id: 'stop-2', name: 'Stop 2', tier: 1, x: 600, y: 300, isOnline: true };
const stopsMap = new Map<string, GeoStop>([[stop1.id, stop1], [stop2.id, stop2]]);

function simulateRun(
  testName: string,
  startPos: Point2D,
  headingDeg: number,
  speedMps: number,
  totalSteps: number,
  dtSec: number = 0.050
) {
  console.log(`\n--- ${testName} ---`);
  resetAllTrackingState();
  const tagId = 'bruno-test';
  resetDogTrajectory(tagId, startPos);

  const headingRad = (headingDeg * Math.PI) / 180;
  const ux = Math.sin(headingRad);
  const uy = -Math.cos(headingRad);
  const stepPix = speedMps * config.pixelsPerMeter * dtSec;

  const tag: GeoTag = {
    id: tagId,
    name: 'Bruno',
    x: startPos.x,
    y: startPos.y,
    vx: ux * speedMps * config.pixelsPerMeter,
    vy: uy * speedMps * config.pixelsPerMeter,
    headingDeg,
    autoWalk: true,
    imuResting: false,
    speedMps,
    batteryPct: 95,
    color: '#00ff41'
  };

  let maxErrorM = 0;
  let totalErrorM = 0;
  let falseFlips = 0;
  let crossedBaseline = false;

  for (let step = 1; step <= totalSteps; step++) {
    const prevY = tag.y;
    tag.x += ux * stepPix;
    tag.y += uy * stepPix;

    if ((prevY < 300 && tag.y >= 300) || (prevY > 300 && tag.y <= 300)) {
      crossedBaseline = true;
      console.log(`  [BASELINE CROSSED at step ${step} | Tag pos: (${tag.x.toFixed(1)}, ${tag.y.toFixed(1)})]`);
    }

    // Generate RF measurements with realistic Gaussian noise
    const d1Pix = Math.hypot(tag.x - stop1.x, tag.y - stop1.y);
    const d2Pix = Math.hypot(tag.x - stop2.x, tag.y - stop2.y);
    const noise1M = (Math.random() - 0.5) * 1.5;
    const noise2M = (Math.random() - 0.5) * 1.5;

    const meas1: SignalMeasurement = {
      stopId: stop1.id,
      tagId: tag.id,
      trueDistanceMeters: d1Pix / config.pixelsPerMeter,
      noisyDistanceMeters: Math.max(0.5, d1Pix / config.pixelsPerMeter + noise1M),
      rssiDbm: -55,
      obstructed: false,
      numObstructions: 0
    };
    const meas2: SignalMeasurement = {
      stopId: stop2.id,
      tagId: tag.id,
      trueDistanceMeters: d2Pix / config.pixelsPerMeter,
      noisyDistanceMeters: Math.max(0.5, d2Pix / config.pixelsPerMeter + noise2M),
      rssiDbm: -55,
      obstructed: false,
      numObstructions: 0
    };

    const nowMs = 1700000000000 + step * 50;
    const res = solveTrilateration(tag, [meas1, meas2], stopsMap, config, [], nowMs);
    const errM = Math.hypot(res.estimatedX - tag.x, res.estimatedY - tag.y) / config.pixelsPerMeter;
    totalErrorM += errM;
    if (errM > maxErrorM) maxErrorM = errM;

    // A false flip happens when error exceeds 10m (stuck on opposite side of baseline)
    if (errM > 10.0) {
      falseFlips++;
    }

    if (step >= 170 && step <= 180) {
      console.log(`  Step ${step}: True=(${tag.x.toFixed(1)}, ${tag.y.toFixed(1)}) | Est=(${res.estimatedX.toFixed(1)}, ${res.estimatedY.toFixed(1)}) | Err=${errM.toFixed(2)}m`);
      if (res.dualInfo) {
        console.log(`    cand1=(${res.dualInfo.candidate1.x.toFixed(1)}, ${res.dualInfo.candidate1.y.toFixed(1)}) cand2=(${res.dualInfo.candidate2.x.toFixed(1)}, ${res.dualInfo.candidate2.y.toFixed(1)})`);
        console.log(`    score1=${res.dualInfo.candidate1Score} (align=${res.dualInfo.candidate1VectorAlignment}, distM=${res.dualInfo.candidate1DistMeters})`);
        console.log(`    score2=${res.dualInfo.candidate2Score} (align=${res.dualInfo.candidate2VectorAlignment}, distM=${res.dualInfo.candidate2DistMeters})`);
        console.log(`    valIdx=${res.dualInfo.validatedIndex}`);
      }
    }
    if (step % 25 === 0) {
      console.log(`  Step ${step}: True=(${tag.x.toFixed(1)}, ${tag.y.toFixed(1)}) | Est=(${res.estimatedX.toFixed(1)}, ${res.estimatedY.toFixed(1)}) | Err=${errM.toFixed(2)}m | Lock=${res.lockType}`);
    }
  }

  const avgErrorM = totalErrorM / totalSteps;
  console.log(`  Summary: Avg Err = ${avgErrorM.toFixed(2)}m | Max Err = ${maxErrorM.toFixed(2)}m | False Flips = ${falseFlips} | Crossed = ${crossedBaseline}`);
  if (falseFlips === 0) {
    console.log(`  >>> PASS: 100% CLEAN TRACKING (ZERO FALSE FLIPS) <<<`);
  } else {
    console.error(`  >>> FAIL: ${falseFlips} FALSE FLIPS DETECTED <<<`);
  }
  return falseFlips === 0;
}

// TEST 1: Perpendicular 90° Crossing (South: heading = 180°)
// Dog starts at (400, 240) and walks South past Y = 300 to (400, 360) (300 steps)
const pass1 = simulateRun('TEST 1: Perpendicular Crossing (90° South across Y=300)', { x: 400, y: 240 }, 180, 2.5, 300);

// TEST 2: Shallow Diagonal Crossing (10° across baseline: heading = 100°)
// Dog starts at (280, 290) and walks East-South-East (heading 100°) crossing Y = 300 gently
const pass2 = simulateRun('TEST 2: Shallow Diagonal Crossing (10° across baseline)', { x: 280, y: 290 }, 100, 2.5, 350);

// TEST 3: 45° Diagonal Crossing (South-East: heading = 135°)
const pass3 = simulateRun('TEST 3: 45° Diagonal Crossing (South-East across baseline)', { x: 300, y: 250 }, 135, 3.0, 250);

// TEST 4: Pure Parallel Motion alongside baseline (5m away: Y = 286, heading = 90° East)
const pass4 = simulateRun('TEST 4: Pure Parallel Motion alongside baseline (heading = 90° East)', { x: 250, y: 286 }, 90, 2.5, 200);

console.log('\n=================================================================');
if (pass1 && pass2 && pass3 && pass4) {
  console.log('  ALL BASELINE CROSSING TESTS PASSED WITH 100% SUCCESS!');
} else {
  console.log('  SOME TESTS FAILED!');
  process.exit(1);
}
console.log('=================================================================');
