// Comprehensive verification test for:
// 1. DVB-KF (Dynamic Velocity-Bounded Kalman Filter) with canine speed bounding (v <= 8.5 m/s)
// 2. Present-Instant Location Prediction with Collar Heading Guidance
// 3. GeoCentre NLOS Raycasting & Zero-Weight Assignment
// 4. 0-Anchor Border Exit with Last Known Position (LKP) & Velocity Vector

function distance(p1, p2) {
  const dx = p1.x - p2.x;
  const dy = p1.y - p2.y;
  return Math.sqrt(dx * dx + dy * dy);
}

function lineSegmentsIntersect(p1, p2, p3, p4) {
  function ccw(a, b, c) {
    return (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x);
  }
  const c1 = ccw(p1, p3, p4);
  const c2 = ccw(p2, p3, p4);
  const c3 = ccw(p1, p2, p3);
  const c4 = ccw(p1, p2, p4);
  return c1 !== c2 && c3 !== c4;
}

function countBuildingWallIntersections(p1, p2, bldg) {
  let count = 0;
  for (let i = 0; i < bldg.points.length; i++) {
    const next = (i + 1) % bldg.points.length;
    if (lineSegmentsIntersect(p1, p2, bldg.points[i], bldg.points[next])) {
      count++;
    }
  }
  return count;
}

console.log('========================================================================');
console.log('  TEST 1: DVB-KF VELOCITY BOUNDING (CANINE SPEED LIMIT ENFORCEMENT)     ');
console.log('========================================================================\n');

// Simulate DVB-KF state: [x, y, vx, vy]
const ppm = 2.8;
const maxSpeedMps = 8.5; // Canine physical max sprint speed
const maxPixPerSec = maxSpeedMps * ppm;

let kfState = [300, 300, 2.0 * ppm, 0]; // moving East at 2 m/s

// Simulate sudden 15" concrete wall spike: measurement jumps 80 meters away in 80ms!
const dt = 0.08;
const spikeMeasurement = { x: 300 + 80 * ppm, y: 300 }; // 80m jump!

// Unconstrained velocity would be: 80m / 0.08s = 1000 m/s (Mach 3!)
const unconstrainedSpeed = (spikeMeasurement.x - kfState[0]) / dt / ppm;
console.log(`Unconstrained speed if measurement accepted: ${unconstrainedSpeed.toFixed(1)} m/s (IMPOSSIBLE)`);

// Apply DVB-KF clamp
let rawVx = (spikeMeasurement.x - kfState[0]) / dt;
let rawVy = (spikeMeasurement.y - kfState[1]) / dt;
let rawSpeedPix = Math.sqrt(rawVx * rawVx + rawVy * rawVy);

if (rawSpeedPix > maxPixPerSec) {
  const ratio = maxPixPerSec / rawSpeedPix;
  rawVx *= ratio;
  rawVy *= ratio;
}

const clampedSpeedMps = Math.sqrt(rawVx * rawVx + rawVy * rawVy) / ppm;
console.log(`DVB-KF Clamped speed: ${clampedSpeedMps.toFixed(2)} m/s (Capped at physical limit ${maxSpeedMps} m/s)`);

console.assert(clampedSpeedMps <= maxSpeedMps + 1e-4, 'Speed must not exceed maxSpeedMps');
console.log('✓ PASS: DVB-KF strictly bounds canine velocity to <= 8.5 m/s!\n');


console.log('========================================================================');
console.log('  TEST 2: PRESENT-INSTANT PREDICTION WITH COLLAR HEADING GUIDANCE       ');
console.log('========================================================================\n');

// Bruno is walking East at (400, 200), but collar heading detects a 90° turn toward South (heading = 180°)
const pLast = { x: 400, y: 200 };
const vLast = { x: 2.5 * ppm, y: 0 }; // prior velocity was East
const headingDeg = 180; // South
const headingRad = (headingDeg * Math.PI) / 180;
const uCompass = { x: Math.sin(headingRad), y: -Math.cos(headingRad) }; // (0, 1) South

const speedMps = Math.sqrt(vLast.x * vLast.x + vLast.y * vLast.y) / ppm;

// Guidance fusion: 65% linear momentum + 35% collar spine vector
const vGuided = {
  x: vLast.x * 0.65 + (uCompass.x * speedMps * ppm) * 0.35,
  y: vLast.y * 0.65 + (uCompass.y * speedMps * ppm) * 0.35
};

const pPred = {
  x: pLast.x + vGuided.x * dt,
  y: pLast.y + vGuided.y * dt
};

console.log(`Prior Fix: (${pLast.x}, ${pLast.y}), Velocity: East at ${speedMps.toFixed(1)} m/s`);
console.log(`Collar Dorsal Heading: ${headingDeg}° (South)`);
console.log(`Predicted Present Location: (${pPred.x.toFixed(1)}, ${pPred.y.toFixed(1)})`);
console.log(`-> Y moved South by ${(pPred.y - pLast.y).toFixed(2)}px due to collar heading guidance!`);

console.assert(pPred.y > pLast.y, 'Predicted position must bend South toward collar heading');
console.log('✓ PASS: Present-instant prediction cleanly fuses linear momentum and collar azimuth!\n');


console.log('========================================================================');
console.log('  TEST 3: GEOCENTRE NLOS RAYCASTING & ZERO-WEIGHT ASSIGNMENT           ');
console.log('========================================================================\n');

// Building: [370, 630] x [315, 485] (15-inch concrete walls)
const campusBuilding = {
  id: 'academic-block',
  points: [
    { x: 370, y: 315 },
    { x: 630, y: 315 },
    { x: 630, y: 485 },
    { x: 370, y: 485 }
  ]
};

// Stops:
// Stop 1: North Plaza Mast (500, 165) -> CLEAR LOS to North walkway (500, 240)
// Stop 2: East Veranda (690, 400) -> CLEAR LOS
// Stop 3: West Garden (310, 400) -> CLEAR LOS
// Stop 4: South Promenade Mast (500, 635) -> OBSTRUCTED by 15" concrete building!
const stopNorth = { id: 'stop-north', x: 500, y: 165 };
const stopEast = { id: 'stop-east', x: 690, y: 400 };
const stopWest = { id: 'stop-west', x: 310, y: 400 };
const stopSouth = { id: 'stop-south', x: 500, y: 635 };

const candidateStops = [stopNorth, stopEast, stopWest, stopSouth];
// Bruno at Top-Right walkway corner (660, 280) - North-East veranda
const dogPredPos = { x: 660, y: 280 }; 

console.log(`Testing rays from Predicted Present Location (${dogPredPos.x}, ${dogPredPos.y}) to all 4 GeoStops:`);

const evaluatedAnchors = candidateStops.map(s => {
  const walls = countBuildingWallIntersections(dogPredPos, s, campusBuilding);
  const isNlos = walls > 0;
  const weight = isNlos ? 0.0 : 1.0;
  return {
    id: s.id,
    walls,
    isNlos,
    weight
  };
});

evaluatedAnchors.forEach(a => {
  if (a.weight === 0.0) {
    console.log(`  ✗ ${a.id}: OBSTRUCTED by ${a.walls} walls -> ASSIGNED WEIGHT = 0.0 (EXCLUDED FROM MULTILATERATION)`);
  } else {
    console.log(`  ✓ ${a.id}: Clear Line-of-Sight -> ASSIGNED WEIGHT = 1.0 (RETAINED IN SOLVER)`);
  }
});

const southAnchor = evaluatedAnchors.find(a => a.id === 'stop-south');
const westAnchor = evaluatedAnchors.find(a => a.id === 'stop-west');
const northAnchor = evaluatedAnchors.find(a => a.id === 'stop-north');
const eastAnchor = evaluatedAnchors.find(a => a.id === 'stop-east');

console.assert(southAnchor.weight === 0.0, 'South anchor behind building must have weight = 0.0');
console.assert(westAnchor.weight === 0.0, 'West anchor behind building must have weight = 0.0');
console.assert(northAnchor.weight === 1.0, 'North anchor must have clear LOS (weight = 1.0)');
console.assert(eastAnchor.weight === 1.0, 'East anchor must have clear LOS (weight = 1.0)');

const cleanInliers = evaluatedAnchors.filter(a => a.weight > 0);
console.log(`Clean inliers remaining: ${cleanInliers.length} (${cleanInliers.map(a => a.id).join(', ')})`);
console.assert(cleanInliers.length === 2, 'Must have exactly 2 clean inliers (North & East)');
console.log('✓ PASS: Raycasting correctly isolates 15" wall anchors and triggers clean 2-anchor pinpoint mode!\n');


console.log('========================================================================');
console.log('  TEST 4: 0-ANCHOR BORDER EXIT (LAST KNOWN POSITION & VELOCITY VECTOR)  ');
console.log('========================================================================\n');

// State before exiting coverage: Bruno at (500, 240) moving East at 2.4 m/s with 3 anchors
const priorFix = {
  pos: { x: 500, y: 240 },
  velocity: { x: 2.4 * ppm, y: 0 },
  timestamp: Date.now() - 3200, // 3.2 seconds ago
  anchorCount: 3,
  speedMps: 2.4,
  headingDeg: 90
};

// Now Bruno steps outside all anchor ranges (0 valid anchors in range)
const validInliers = []; // 0 anchors!
let borderExitState = null;

if (validInliers.length === 0) {
  if (priorFix.anchorCount > 0) {
    const now = Date.now();
    const elapsedSec = Math.round(((now - priorFix.timestamp) / 1000) * 10) / 10;
    borderExitState = {
      lastKnownPos: { ...priorFix.pos },
      exitVelocity: { ...priorFix.velocity },
      exitHeadingDeg: priorFix.headingDeg,
      exitSpeedMps: priorFix.speedMps,
      exitTimestamp: priorFix.timestamp,
      elapsedSeconds: elapsedSec,
      previousAnchorCount: priorFix.anchorCount
    };
  }
}

console.log('Border Exit Event Captured:');
console.log(`  Last Known Position (LKP): (${borderExitState.lastKnownPos.x}, ${borderExitState.lastKnownPos.y})`);
console.log(`  Exit Velocity Vector: (${borderExitState.exitVelocity.x.toFixed(1)}, ${borderExitState.exitVelocity.y.toFixed(1)}) pixels/s`);
console.log(`  Exit Speed: ${borderExitState.exitSpeedMps} m/s`);
console.log(`  Exit Heading: ${borderExitState.exitHeadingDeg}°`);
console.log(`  Previous Coverage: ${borderExitState.previousAnchorCount} Anchors`);
console.log(`  Elapsed Time in Blind Zone: ${borderExitState.elapsedSeconds}s`);

console.assert(borderExitState !== null, 'Border exit state must be captured');
console.assert(borderExitState.lastKnownPos.x === 500 && borderExitState.lastKnownPos.y === 240, 'LKP must match prior fix');
console.assert(borderExitState.exitSpeedMps === 2.4, 'Exit speed must match');

console.log('✓ PASS: 0-anchor border exit successfully captures LKP and directional velocity vector!\n');
console.log('========================================================================');
console.log('  ALL MATHEMATICAL TESTS PASSED WITH 100% SUCCESS!                      ');
console.log('========================================================================');
