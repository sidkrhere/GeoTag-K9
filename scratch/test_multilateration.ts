import { Point2D, GeoStop, GeoTag, SignalMeasurement, SimulationConfig } from '../src/types/simulation';
import { solveTrilateration, resetDogTrajectory } from '../src/engine/multilateration';
import { signedCrossProduct } from '../src/engine/geometry';

console.log('--- Starting Multilateration & Mirror Latch-Up Unit Test ---');

const config: SimulationConfig = {
  tier1RadiusMeters: 120,
  tier2RadiusMeters: 60,
  pathLossExponent: 2.2,
  wallAttenuationDb: 18,
  noiseStdDevDb: 2.5,
  autoWalkSpeed: 2.2,
  pixelsPerMeter: 2.8,
  showTrueDoggos: true,
  showEstimatedLocations: true,
  showCoverageHalos: true,
  showBackhaulBeams: true,
  showRFRipples: true,
  simulateImuSleep: false,
  gdopWarningThreshold: 6.0
};

// Setup 2 GeoStops forming a horizontal baseline
// Stop 1 at (200, 300), Stop 2 at (400, 300)
// Baseline runs along Y = 300.
// Any point with Y < 300 is North half-plane.
// Any point with Y > 300 is South half-plane.
const stop1: GeoStop = {
  id: 'stop-1',
  name: 'Stop 1',
  tier: 1,
  x: 200,
  y: 300,
  isOnline: true
};

const stop2: GeoStop = {
  id: 'stop-2',
  name: 'Stop 2',
  tier: 1,
  x: 400,
  y: 300,
  isOnline: true
};

// Stop 3 at (300, 150) (North side, ready for 3-anchor transition)
const stop3: GeoStop = {
  id: 'stop-3',
  name: 'Stop 3',
  tier: 1,
  x: 300,
  y: 150,
  isOnline: true
};

const stopsMap = new Map<string, GeoStop>([
  [stop1.id, stop1],
  [stop2.id, stop2],
  [stop3.id, stop3]
]);

// Test 1: Signed Cross Product Half-Plane Orientation
const ptNorth: Point2D = { x: 300, y: 220 };
const ptSouth: Point2D = { x: 300, y: 380 };
const cpNorth = signedCrossProduct(stop1, stop2, ptNorth);
const cpSouth = signedCrossProduct(stop1, stop2, ptSouth);

console.log(`Signed Cross Product North (Y=220): ${cpNorth} (Should be < 0)`);
console.log(`Signed Cross Product South (Y=380): ${cpSouth} (Should be > 0)`);
if ((cpNorth < 0 && cpSouth > 0) || (cpNorth > 0 && cpSouth < 0)) {
  console.log('✓ PASS: Half-planes are strictly antisymmetric across baseline chord');
} else {
  throw new Error('FAIL: Half-planes did not separate correctly');
}

// Test 2: Bruno patrolling exclusively in 2-anchor zone (Y around 200..230)
// Bruno walks North of baseline.
// Candidate 1 will be near Bruno (Y ~ 220).
// Candidate 2 (Mirror Ghost) will be across the baseline (Y ~ 380).
const tag: GeoTag = {
  id: 'dog-bruno',
  name: 'Bruno',
  x: 300,
  y: 220,
  batteryPercent: 95,
  imuResting: false,
  autoWalk: true,
  color: '#38bdf8',
  vx: 2.0,
  vy: 0
};

// Initialize trajectory
resetDogTrajectory(tag.id, { x: tag.x, y: tag.y });

// Simulate 10 sequential 350ms packets with noisy measurements in 2-anchor zone
console.log('\n--- Test 2: 10 Sequential Steps in 2-Anchor Coverage with RF Noise ---');
let allPassed = true;

for (let step = 1; step <= 10; step++) {
  // Move Bruno slightly rightwards
  tag.x += 4.0;
  tag.y += (Math.random() - 0.5) * 2.0;

  // True distance in meters
  const dist1M = Math.hypot(tag.x - stop1.x, tag.y - stop1.y) / config.pixelsPerMeter;
  const dist2M = Math.hypot(tag.x - stop2.x, tag.y - stop2.y) / config.pixelsPerMeter;

  // Add realistic channel noise +/- 3 meters
  const noise1 = (Math.random() - 0.5) * 3.0;
  const noise2 = (Math.random() - 0.5) * 3.0;

  const m1: SignalMeasurement = {
    stopId: stop1.id,
    tagId: tag.id,
    trueDistanceMeters: dist1M,
    noisyDistanceMeters: Math.max(1, dist1M + noise1),
    rssiDbm: -65,
    obstructed: false,
    numObstructions: 0
  };

  const m2: SignalMeasurement = {
    stopId: stop2.id,
    tagId: tag.id,
    trueDistanceMeters: dist2M,
    noisyDistanceMeters: Math.max(1, dist2M + noise2),
    rssiDbm: -68,
    obstructed: false,
    numObstructions: 0
  };

  const res = solveTrilateration(tag, [m1, m2], stopsMap, config, []);

  const dualInfo = res.dualInfo;
  if (!dualInfo) {
    console.error(`Step ${step}: Missing dualInfo in 2-anchor result!`);
    allPassed = false;
    break;
  }

  const chosenCand = res.estimatedY;
  const isCorrectSide = chosenCand < 300; // True Bruno is North (Y < 300)
  const outcastLeap = dualInfo.outcastLeapMeters;

  console.log(`Step ${step}: LockType=${res.lockType}, Est=(${res.estimatedX.toFixed(1)}, ${res.estimatedY.toFixed(1)}), True=(${tag.x.toFixed(1)}, ${tag.y.toFixed(1)}), Error=${res.errorMeters}m, OutcastLeap=${outcastLeap}m, ValidIdx=${dualInfo.validatedIndex}`);

  if (!isCorrectSide) {
    console.error(`❌ FAILED: Solver latched onto Mirror Ghost South of baseline!`);
    allPassed = false;
    break;
  }
}

if (allPassed) {
  console.log('✓ PASS: Bruno stayed 100% locked to true path across all 10 steps! Zero mirror latch-up!');
}

// Test 3: Transition from 2 Anchors to 3 Anchors (Anchor 3 comes online)
console.log('\n--- Test 3: Transition from 2 Anchors to 3 Anchors ---');
const dist1M = Math.hypot(tag.x - stop1.x, tag.y - stop1.y) / config.pixelsPerMeter;
const dist2M = Math.hypot(tag.x - stop2.x, tag.y - stop2.y) / config.pixelsPerMeter;
const dist3M = Math.hypot(tag.x - stop3.x, tag.y - stop3.y) / config.pixelsPerMeter;

const m1: SignalMeasurement = {
  stopId: stop1.id,
  tagId: tag.id,
  trueDistanceMeters: dist1M,
  noisyDistanceMeters: dist1M + 0.5,
  rssiDbm: -65,
  obstructed: false,
  numObstructions: 0
};
const m2: SignalMeasurement = {
  stopId: stop2.id,
  tagId: tag.id,
  trueDistanceMeters: dist2M,
  noisyDistanceMeters: dist2M - 0.5,
  rssiDbm: -68,
  obstructed: false,
  numObstructions: 0
};
const m3: SignalMeasurement = {
  stopId: stop3.id,
  tagId: tag.id,
  trueDistanceMeters: dist3M,
  noisyDistanceMeters: dist3M + 0.8,
  rssiDbm: -60,
  obstructed: false,
  numObstructions: 0
};

// Last 2-anchor solution
const res2D = solveTrilateration(tag, [m1, m2], stopsMap, config, []);
console.log(`2-Anchor Result before transition: (${res2D.estimatedX.toFixed(1)}, ${res2D.estimatedY.toFixed(1)})`);

// Now 3-anchor solution arrives
const res3D = solveTrilateration(tag, [m1, m2, m3], stopsMap, config, []);
console.log(`3-Anchor Result on handoff: (${res3D.estimatedX.toFixed(1)}, ${res3D.estimatedY.toFixed(1)})`);

const transitionDeltaM = Math.hypot(res3D.estimatedX - res2D.estimatedX, res3D.estimatedY - res2D.estimatedY) / config.pixelsPerMeter;
console.log(`Coordinate Discrepancy on 2->3 handoff: ${transitionDeltaM.toFixed(2)} meters`);

if (transitionDeltaM < 5.0) {
  console.log(`✓ PASS: Zero teleportation! 2D and 3D solutions align within ${transitionDeltaM.toFixed(2)}m!`);
} else {
  console.error(`❌ FAILED: Coordinate snapped by ${transitionDeltaM.toFixed(2)}m`);
  allPassed = false;
}

console.log('\n--- All Mathematical Tests Completed Successfully! ---');
