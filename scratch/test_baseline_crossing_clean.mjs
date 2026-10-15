// Test of Smooth Baseline Crossing with Chord-Scaled Half-Plane Gating
function distance(p1, p2) {
  return Math.hypot(p1.x - p2.x, p1.y - p2.y);
}

function signedCrossProduct(a, b, p) {
  return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

function findTwoCircleIntersections(a1, r1, a2, r2) {
  const d = distance(a1, a2);
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

console.log('=== TESTING TRUE BASELINE CROSSING TRAJECTORY ===\n');

// Baseline: Anchor 1 at (200, 300), Anchor 2 at (600, 300) -> Horizontal line y = 300
const a1 = { id: 'a1', x: 200, y: 300 };
const a2 = { id: 'a2', x: 600, y: 300 };
const ppm = 2.8;

// Dog starts North at (400, 260) and walks straight South across the line to (400, 340)
// Crossing y = 300 happens at step 20!
let trueDog = { x: 400, y: 260 };
const headingDeg = 180; // Heading South (yaw = 180)
const headingRad = Math.PI; // (dx = 0, dy = +1)
const speedMps = 2.0;
const dt = 0.050; // 50ms
const stepPix = speedMps * ppm * dt; // ~0.28 px per step

const historyTrail = [];
for (let i = 0; i < 10; i++) {
  historyTrail.push({ x: 400, y: 260 - (10 - i) * stepPix });
}

let lastEst = { x: 400, y: 260 };
let maxJump = 0;
let crossingReported = false;

for (let step = 0; step < 250; step++) {
  // Move dog South
  trueDog.y += stepPix;

  if (trueDog.y >= 300 && !crossingReported) {
    crossingReported = true;
    console.log(`>>> DOG CROSSES BASELINE AT STEP ${step} (y = ${trueDog.y.toFixed(2)}) <<<`);
  }

  // Radii with realistic noise
  const r1 = distance(a1, trueDog) + (Math.random() - 0.5) * 1.5 * ppm;
  const r2 = distance(a2, trueDog) + (Math.random() - 0.5) * 1.5 * ppm;

  const [c1, c2] = findTwoCircleIntersections(a1, r1, a2, r2);
  const chordMeters = distance(c1, c2) / ppm;

  // 1. Compute Historical Consensus Position and Velocity
  let histCentroid = { x: 0, y: 0 };
  let weightSum = 0;
  for (let i = 0; i < historyTrail.length; i++) {
    // Exponential weighting giving recent fixes dominant power
    const w = Math.pow(1.3, i);
    histCentroid.x += historyTrail[i].x * w;
    histCentroid.y += historyTrail[i].y * w;
    weightSum += w;
  }
  histCentroid.x /= weightSum;
  histCentroid.y /= weightSum;

  // Recent velocity from last 3 fixes
  const lastFix = historyTrail[historyTrail.length - 1];
  const prevFix = historyTrail[historyTrail.length - 3] || historyTrail[0];
  const histVx = (lastFix.x - prevFix.x) / (2 * dt);
  const histVy = (lastFix.y - prevFix.y) / (2 * dt);

  // Extrapolate expected position
  const extrapolatedRef = {
    x: lastFix.x + histVx * dt,
    y: lastFix.y + histVy * dt
  };

  // 2. Chord-Scaled Half-Plane Gating
  // When chord is large (> 6m), dog is far from baseline -> enforce half-plane consistency
  // When chord is small (< 3m), dog is transit crossing -> penalty smoothly decays to ZERO!
  const chordGateFactor = Math.max(0, Math.min(1.0, (chordMeters - 2.5) / 4.0));

  // Count votes from weighted history
  const sign1 = Math.sign(signedCrossProduct(a1, a2, c1));
  let votes1 = 0, votes2 = 0;
  for (let i = 0; i < historyTrail.length; i++) {
    const w = Math.pow(1.2, i);
    const hSign = Math.sign(signedCrossProduct(a1, a2, historyTrail[i]));
    if (hSign === sign1) votes1 += w;
    else votes2 += w;
  }

  const halfPlanePenalty1 = (votes2 > votes1 ? 30.0 : 0.0) * chordGateFactor;
  const halfPlanePenalty2 = (votes1 > votes2 ? 30.0 : 0.0) * chordGateFactor;

  // Distance from extrapolated forward reference
  const dRef1 = distance(c1, extrapolatedRef) / ppm;
  const dRef2 = distance(c2, extrapolatedRef) / ppm;

  // Motion direction alignment with collar compass
  // Heading unit vector
  const uCompass = { x: Math.sin(headingRad), y: -Math.cos(headingRad) }; // South: (0, 1)
  const dir1 = { x: (c1.x - lastFix.x) / ppm, y: (c1.y - lastFix.y) / ppm };
  const dir2 = { x: (c2.x - lastFix.x) / ppm, y: (c2.y - lastFix.y) / ppm };
  const align1 = (dir1.x * uCompass.x + dir1.y * uCompass.y) / (Math.hypot(dir1.x, dir1.y) || 1);
  const align2 = (dir2.x * uCompass.x + dir2.y * uCompass.y) / (Math.hypot(dir2.x, dir2.y) || 1);

  // Score candidate:
  // Primary term: Euclidean distance from extrapolated trajectory
  // Secondary term: Directional alignment bonus (-2.0 max)
  // Gate term: Chord-scaled half-plane memory penalty
  const score1 = dRef1 * 4.0 - align1 * 2.5 + halfPlanePenalty1;
  const score2 = dRef2 * 4.0 - align2 * 2.5 + halfPlanePenalty2;

  if (step === 49 || step === 50) {
    console.log(`Step ${step} Debug:`);
    console.log(`  extrapolatedRef: (${extrapolatedRef.x.toFixed(1)}, ${extrapolatedRef.y.toFixed(1)})`);
    console.log(`  c1: (${c1.x.toFixed(1)}, ${c1.y.toFixed(1)}) | dRef1: ${dRef1.toFixed(2)}m | align1: ${align1.toFixed(2)} | penalty1: ${halfPlanePenalty1.toFixed(1)} | score1: ${score1.toFixed(2)}`);
    console.log(`  c2: (${c2.x.toFixed(1)}, ${c2.y.toFixed(1)}) | dRef2: ${dRef2.toFixed(2)}m | align2: ${align2.toFixed(2)} | penalty2: ${halfPlanePenalty2.toFixed(1)} | score2: ${score2.toFixed(2)}`);
  }
  const chosen = score1 <= score2 ? c1 : c2;
  const outcast = score1 <= score2 ? c2 : c1;

  // Track jump
  const jump = distance(chosen, lastEst) / ppm;
  if (jump > maxJump) maxJump = jump;
  const error = distance(chosen, trueDog) / ppm;

  if (step % 25 === 0 || Math.abs(trueDog.y - 300) < 1.0) {
    console.log(`Step ${step.toString().padStart(2)}: trueY=${trueDog.y.toFixed(1)} | estY=${chosen.y.toFixed(1)} | chord=${chordMeters.toFixed(1)}m | gate=${chordGateFactor.toFixed(2)} | jump=${jump.toFixed(2)}m | err=${error.toFixed(2)}m`);
  }

  historyTrail.push({ ...chosen });
  if (historyTrail.length > 12) historyTrail.shift();
  lastEst = { ...chosen };
}

console.log(`\nMax jump across entire crossing: ${maxJump.toFixed(2)} meters!`);
