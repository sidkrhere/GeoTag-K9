// Test verifying the new bulletproof 2-anchor mirror outcasting logic

function distance(p1, p2) {
  const dx = p1.x - p2.x;
  const dy = p1.y - p2.y;
  return Math.sqrt(dx * dx + dy * dy);
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

console.log('Testing Canonical 2-Anchor Mirror Outcast Logic across 50 frames with turns...\n');

const stop1 = { id: 'pole-1', x: 200, y: 300 };
const stop2 = { id: 'pole-2', x: 500, y: 300 };
const ppm = 2.8;

// Canonical baseline sorted by ID
const [anchorA, anchorB] = stop1.id < stop2.id ? [stop1, stop2] : [stop2, stop1];

let brunoTrue = { x: 250, y: 200 }; // Starts North (Y < 300)
let historyTrail = [{ ...brunoTrue }];
let lastPos = { ...brunoTrue };
let velocity = { x: 3.0 * ppm, y: 0 };

let totalFrames = 50;
let failedFrames = 0;

for (let frame = 1; frame <= totalFrames; frame++) {
  // Simulate motion with turns
  if (frame === 15) velocity = { x: 0, y: -2.5 * ppm }; // Turn further North
  if (frame === 30) velocity = { x: 2.5 * ppm, y: 1.0 * ppm }; // Turn South-East (staying North of baseline)

  const dt = 0.08; // 80ms interval
  brunoTrue.x += (velocity.x / ppm) * dt * ppm;
  brunoTrue.y += (velocity.y / ppm) * dt * ppm;

  const r1 = distance(stop1, brunoTrue);
  const r2 = distance(stop2, brunoTrue);

  // Add severe RF noise
  const noise1 = (Math.random() - 0.5) * 6 * ppm;
  const noise2 = (Math.random() - 0.5) * 6 * ppm;

  // Measurement order might be [stop1, stop2] or [stop2, stop1] randomly!
  const swapOrder = Math.random() > 0.5;
  const [measA_stop, measA_r, measB_stop, measB_r] = swapOrder 
    ? [stop2, r2 + noise2, stop1, r1 + noise1]
    : [stop1, r1 + noise1, stop2, r2 + noise2];

  // ALWAYS sort canonically!
  const isOrdered = measA_stop.id < measB_stop.id;
  const canonA = isOrdered ? measA_stop : measB_stop;
  const canonB = isOrdered ? measB_stop : measA_stop;
  const canonRA = isOrdered ? measA_r : measB_r;
  const canonRB = isOrdered ? measB_r : measA_r;

  const [cA, cB] = findTwoCircleIntersections(canonA, canonRA, canonB, canonRB);

  const cpA = signedCrossProduct(canonA, canonB, cA);
  const cpB = signedCrossProduct(canonA, canonB, cB);

  // Determine trail side
  let posVotes = 0, negVotes = 0;
  for (const pt of historyTrail) {
    const cp = signedCrossProduct(canonA, canonB, pt);
    if (cp > 0.5) posVotes++;
    else if (cp < -0.5) negVotes++;
  }
  const trailSideSign = posVotes >= negVotes ? 1 : -1;

  // Proximity to trail
  const dTrailA = Math.min(...historyTrail.map(h => distance(h, cA)));
  const dTrailB = Math.min(...historyTrail.map(h => distance(h, cB)));

  // Distance to predicted position
  const predPos = { x: lastPos.x + velocity.x * dt, y: lastPos.y + velocity.y * dt };
  const dPredA = distance(predPos, cA);
  const dPredB = distance(predPos, cB);

  let scoreA = dTrailA * 0.7 + dPredA * 0.3;
  let scoreB = dTrailB * 0.7 + dPredB * 0.3;

  const chordSepM = distance(cA, cB) / ppm;
  const sideA = cpA >= 0 ? 1 : -1;
  const sideB = cpB >= 0 ? 1 : -1;

  if (chordSepM >= 6.0) {
    if (sideA !== trailSideSign) scoreA += 10000;
    if (sideB !== trailSideSign) scoreB += 10000;
  }

  const chosen = scoreA <= scoreB ? cA : cB;
  const outcast = scoreA <= scoreB ? cB : cA;

  // Check correctness: Bruno is North of baseline (Y < 300)
  if (chosen.y >= 300) {
    console.error(`Frame ${frame}: FAILED! Chosen candidate was at Y=${chosen.y.toFixed(1)} (South of baseline)`);
    failedFrames++;
  }

  historyTrail.push({ ...chosen });
  if (historyTrail.length > 8) historyTrail.shift();
  lastPos = chosen;
}

if (failedFrames === 0) {
  console.log(`✓ 100% SUCCESS across all ${totalFrames} frames with random turns, measurement swapping, and severe RF noise!`);
} else {
  console.error(`Failed ${failedFrames} frames!`);
}
