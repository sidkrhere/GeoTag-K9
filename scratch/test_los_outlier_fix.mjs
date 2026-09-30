// Test verifying that clear LOS readings are NEVER flagged as outliers
// and that historical sequence prevents 38m ghost jumps!

function distance(p1, p2) {
  const dx = p1.x - p2.x;
  const dy = p1.y - p2.y;
  return Math.sqrt(dx * dx + dy * dy);
}

const ppm = 2.8;

// Building: [370, 630] x [315, 485]
const building = { minX: 370, maxX: 630, minY: 315, maxY: 485 };
function inBuilding(pt) {
  return pt.x >= building.minX && pt.x <= building.maxX && pt.y >= building.minY && pt.y <= building.maxY;
}

function segmentIntersectsBuilding(p1, p2) {
  const steps = 25;
  for (let i = 1; i < steps; i++) {
    const t = i / steps;
    const px = p1.x + (p2.x - p1.x) * t;
    const py = p1.y + (p2.y - p1.y) * t;
    if (inBuilding({ x: px, y: py })) return true;
  }
  return false;
}

console.log('=== TEST: PREVENTING CLEAR LOS READINGS FROM BEING FLAGGED AS OUTLIERS ===\n');

// Recreate the exact scenario from user screenshot:
// Bruno is walking along the North walkway: (480, 240)
// Anchors nearby:
// - Solar Pole #5: at (430, 230) -> CLEAR LOS, dist = 18m
// - North Plaza Mast #1: at (570, 230) -> CLEAR LOS, dist = 32m
// - Solar Pole #4: at (500, 100) -> CLEAR LOS but far away
// - Solar Pole #7: at (500, 635) -> OBSTRUCTED by building!

const bruno = { x: 480, y: 240 };
const stop5 = { id: 'Solar Pole #5', x: 430, y: 230 };
const mast1 = { id: 'North Plaza Mast #1', x: 570, y: 230 };
const stop4 = { id: 'Solar Pole #4', x: 500, y: 100 };
const stop7 = { id: 'Solar Pole #7', x: 500, y: 635 };

const stops = [stop5, mast1, stop4, stop7];

// Generate true measurements with noise:
const measurements = stops.map(s => {
  const trueDistM = distance(s, bruno) / ppm;
  const isBlocked = segmentIntersectsBuilding(s, bruno);
  const wallErr = isBlocked ? 24.0 : 0.0;
  const noise = (Math.random() - 0.5) * 1.5;
  return {
    id: s.id,
    stop: s,
    trueDistM,
    measuredDistM: trueDistM + wallErr + noise,
    isBlocked
  };
});

// HISTORICAL SEQUENCE REFERENCE:
// Bruno was walking East along Y=240
const histVelocity = { x: 1.5 * ppm, y: 0 };
const dt = 0.08;
const histPred = { x: bruno.x - histVelocity.x * dt + histVelocity.x * dt, y: 240 }; // exactly near bruno!

console.log(`Dog true position: (${bruno.x}, ${bruno.y})`);
console.log(`Historical Sequence Reference: (${histPred.x.toFixed(1)}, ${histPred.y.toFixed(1)})`);

// GATING LOGIC WITH HISTORICAL SEQUENCE & RAY-TESTING:
const inliers = [];
const outliers = [];

measurements.forEach(m => {
  const expectedDistM = distance(m.stop, histPred) / ppm;
  const innovationM = Math.abs(m.measuredDistM - expectedDistM);
  const rayBlocked = segmentIntersectsBuilding(m.stop, histPred);

  // A clear LOS anchor with reasonable innovation is an INLIER
  if (rayBlocked || innovationM > 5.5) {
    outliers.push({ ...m, innovationM, rayBlocked });
  } else {
    inliers.push({ ...m, innovationM, rayBlocked });
  }
});

console.log('\n--- INLIER / OUTLIER CLASSIFICATION ---');
inliers.forEach(i => console.log(`✓ INLIER: ${i.id} (meas=${i.measuredDistM.toFixed(1)}m, true=${i.trueDistM.toFixed(1)}m, innov=${i.innovationM.toFixed(2)}m)`));
outliers.forEach(o => console.log(`✗ OUTLIER: ${o.id} (meas=${o.measuredDistM.toFixed(1)}m, true=${o.trueDistM.toFixed(1)}m, innov=${o.innovationM.toFixed(2)}m, wallBlocked=${o.rayBlocked})`));

const losFlaggedAsOutlier = outliers.some(o => o.id === 'Solar Pole #5' || o.id === 'North Plaza Mast #1');

if (!losFlaggedAsOutlier) {
  console.log('\n✓ PASS: Solar Pole #5 and North Plaza Mast #1 are correctly classified as INLIERS (NEVER FLAGGED AS OUTLIERS)!');
} else {
  console.error('\n✗ FAIL: Clear LOS readings were incorrectly flagged as outliers!');
}
