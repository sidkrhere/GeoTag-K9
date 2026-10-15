// Bulletproof Baseline Crossing with Corridor State Machine & Heading Vector Disambiguation
function distance(p1, p2) { return Math.hypot(p1.x - p2.x, p1.y - p2.y); }
function signedDistToLine(p, l1, l2) {
  // Directed perpendicular distance from point p to directed line l1 -> l2
  const dx = l2.x - l1.x;
  const dy = l2.y - l1.y;
  const len = Math.hypot(dx, dy) || 1;
  return ((l2.x - l1.x) * (p.y - l1.y) - (l2.y - l1.y) * (p.x - l1.x)) / len;
}

function findTwoCircleIntersections(a1, r1, a2, r2) {
  const d = distance(a1, a2);
  let effectiveR1 = r1, effectiveR2 = r2;
  if (d > r1 + r2) {
    const scale = (d / (r1 + r2)) * 1.002;
    effectiveR1 *= scale; effectiveR2 *= scale;
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
  return [{ x: p0x + rx, y: p0y + ry }, { x: p0x - rx, y: p0y - ry }];
}

console.log('=== TESTING BULLETPROOF BASELINE CROSSING STATE MACHINE ===\n');

const a1 = { id: 'a1', x: 200, y: 300 };
const a2 = { id: 'a2', x: 600, y: 300 };
const ppm = 2.8;

// Dog starts North at y = 260 and walks South across y = 300 to y = 340
let trueDog = { x: 400, y: 260 };
const speedMps = 2.0;
const dt = 0.050;
const stepPix = speedMps * ppm * dt;

let kfState = { x: 400, y: 260, vx: 0, vy: speedMps * ppm };
let lastEst = { x: 400, y: 260 };

// Active Baseline State: 'SIDE_A' (North, y < 300), 'CORRIDOR', 'SIDE_B' (South, y > 300)
let baselineState = 'SIDE_A';
let maxJump = 0;

for (let step = 0; step < 280; step++) {
  trueDog.y += stepPix;

  const r1 = distance(a1, trueDog) + (Math.random() - 0.5) * 1.5 * ppm;
  const r2 = distance(a2, trueDog) + (Math.random() - 0.5) * 1.5 * ppm;

  const [c1, c2] = findTwoCircleIntersections(a1, r1, a2, r2);

  // Perpendicular distance to baseline (meters)
  // Positive = North, Negative = South
  const distToLineM = signedDistToLine(kfState, a1, a2) / ppm;
  const uCompass = { x: 0, y: 1 }; // South

  // State Machine:
  // Corridor threshold = 4.0 meters
  if (Math.abs(distToLineM) < 4.0) {
    baselineState = 'CORRIDOR';
  } else if (distToLineM > 4.0) {
    baselineState = 'SIDE_A';
  } else {
    baselineState = 'SIDE_B';
  }

  // Predict position
  const pred = {
    x: kfState.x + kfState.vx * dt,
    y: kfState.y + kfState.vy * dt
  };

  let chosen;
  if (baselineState === 'CORRIDOR') {
    // In Crossing Corridor: Disambiguate using Heading Vector Projection!
    // The candidate that moves in the direction of the compass is the TRUE crossing!
    const v1 = { x: (c1.x - kfState.x) / ppm, y: (c1.y - kfState.y) / ppm };
    const v2 = { x: (c2.x - kfState.x) / ppm, y: (c2.y - kfState.y) / ppm };
    const dot1 = v1.x * uCompass.x + v1.y * uCompass.y;
    const dot2 = v2.x * uCompass.x + v2.y * uCompass.y;

    // Pick candidate moving in compass direction
    chosen = dot1 >= dot2 ? c1 : c2;
  } else if (baselineState === 'SIDE_A') {
    // Deep on Side A: Pick candidate on Side A (y < 300)
    const dLine1 = signedDistToLine(c1, a1, a2);
    const dLine2 = signedDistToLine(c2, a1, a2);
    chosen = dLine1 >= dLine2 ? c1 : c2;
  } else {
    // Deep on Side B: Pick candidate on Side B (y > 300)
    const dLine1 = signedDistToLine(c1, a1, a2);
    const dLine2 = signedDistToLine(c2, a1, a2);
    chosen = dLine1 <= dLine2 ? c1 : c2;
  }

  // Update filter
  const innovX = chosen.x - pred.x;
  const innovY = chosen.y - pred.y;

  kfState.x = pred.x + innovX * 0.40;
  kfState.y = pred.y + innovY * 0.40;
  kfState.vx = kfState.vx * 0.85 + (innovX / dt) * 0.15;
  kfState.vy = kfState.vy * 0.85 + (innovY / dt) * 0.15;

  const jump = distance(kfState, lastEst) / ppm;
  if (jump > maxJump) maxJump = jump;
  const error = distance(kfState, trueDog) / ppm;

  if (step % 35 === 0 || Math.abs(trueDog.y - 300) < 1.0) {
    console.log(`Step ${step.toString().padStart(3)}: trueY=${trueDog.y.toFixed(1)} | estY=${kfState.y.toFixed(1)} | state=${baselineState.padEnd(8)} | jump=${jump.toFixed(2)}m | err=${error.toFixed(2)}m`);
  }

  lastEst = { ...kfState };
}

console.log(`\nMax jump across entire crossing: ${maxJump.toFixed(2)} meters!`);
