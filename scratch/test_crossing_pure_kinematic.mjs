// Test of Kinematic Continuity Candidate Selection (Zero rigid half-plane lock)
function distance(p1, p2) { return Math.hypot(p1.x - p2.x, p1.y - p2.y); }
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

console.log('=== TESTING CONTINUOUS KINEMATIC CROSSING WITHOUT RIGID HALF-PLANE LOCK ===\n');

const a1 = { id: 'a1', x: 200, y: 300 };
const a2 = { id: 'a2', x: 600, y: 300 };
const ppm = 2.8;

// Dog starts North at y = 260 and walks South across y = 300 to y = 340
let trueDog = { x: 400, y: 260 };
const speedMps = 2.0;
const dt = 0.050;
const stepPix = speedMps * ppm * dt; // 0.28 px/step

// Kalman Filter state: [x, y, vx, vy]
let kfState = { x: 400, y: 260, vx: 0, vy: speedMps * ppm };
let lastEst = { x: 400, y: 260 };
let maxJump = 0;
let crossingReported = false;

for (let step = 0; step < 280; step++) {
  trueDog.y += stepPix;

  if (trueDog.y >= 300 && !crossingReported) {
    crossingReported = true;
    console.log(`>>> DOG CROSSES BASELINE AT STEP ${step} (y = ${trueDog.y.toFixed(2)}) <<<`);
  }

  // Radii with realistic RF noise (+/- 2 dB)
  const r1 = distance(a1, trueDog) + (Math.random() - 0.5) * 1.5 * ppm;
  const r2 = distance(a2, trueDog) + (Math.random() - 0.5) * 1.5 * ppm;

  const [c1, c2] = findTwoCircleIntersections(a1, r1, a2, r2);
  const chordMeters = distance(c1, c2) / ppm;

  // 1. Kinematic Prediction: forward extrapolation from current filtered state
  const predX = kfState.x + kfState.vx * dt;
  const predY = kfState.y + kfState.vy * dt;
  const pred = { x: predX, y: predY };

  // 2. Candidate Scoring using Kinematic Extrapolation:
  // How close is each candidate to the forward predicted position?
  const d1 = distance(c1, pred) / ppm;
  const d2 = distance(c2, pred) / ppm;

  // Heading alignment: collar compass points South: (0, 1)
  const uCompass = { x: 0, y: 1 };
  const stepV1 = { x: (c1.x - kfState.x) / ppm, y: (c1.y - kfState.y) / ppm };
  const stepV2 = { x: (c2.x - kfState.x) / ppm, y: (c2.y - kfState.y) / ppm };
  const align1 = (stepV1.x * uCompass.x + stepV1.y * uCompass.y) / (Math.hypot(stepV1.x, stepV1.y) || 1);
  const align2 = (stepV2.x * uCompass.x + stepV2.y * uCompass.y) / (Math.hypot(stepV2.x, stepV2.y) || 1);

  // Score = Distance to Prediction - Bounded Heading Alignment
  const score1 = d1 * 4.0 - align1 * 2.0;
  const score2 = d2 * 4.0 - align2 * 2.0;

  const chosen = score1 <= score2 ? c1 : c2;

  // 3. Smooth Kalman State Update
  // Innovation
  const innovX = chosen.x - pred.x;
  const innovY = chosen.y - pred.y;

  // Simple, stable gain (K_pos = 0.35, K_vel = 0.20)
  kfState.x = pred.x + innovX * 0.35;
  kfState.y = pred.y + innovY * 0.35;
  kfState.vx = kfState.vx * 0.90 + (innovX / dt) * 0.10;
  kfState.vy = kfState.vy * 0.90 + (innovY / dt) * 0.10;

  // Clamp velocity to canine physical limits (max 12 m/s)
  const vMag = Math.hypot(kfState.vx, kfState.vy);
  const maxV = 12.0 * ppm;
  if (vMag > maxV) {
    kfState.vx = (kfState.vx / vMag) * maxV;
    kfState.vy = (kfState.vy / vMag) * maxV;
  }

  const jump = distance(kfState, lastEst) / ppm;
  if (jump > maxJump) maxJump = jump;
  const error = distance(kfState, trueDog) / ppm;

  if (step % 30 === 0 || Math.abs(trueDog.y - 300) < 1.0) {
    console.log(`Step ${step.toString().padStart(3)}: trueY=${trueDog.y.toFixed(1)} | estY=${kfState.y.toFixed(1)} | chord=${chordMeters.toFixed(1)}m | jump=${jump.toFixed(2)}m | err=${error.toFixed(2)}m`);
  }

  lastEst = { ...kfState };
}

console.log(`\nMax jump across entire crossing: ${maxJump.toFixed(2)} meters!`);
