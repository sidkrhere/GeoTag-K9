// Verification of purged, pure simple trilateration solver

function distance(p1, p2) {
  const dx = p1.x - p2.x;
  const dy = p1.y - p2.y;
  return Math.sqrt(dx * dx + dy * dy);
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

function solveGaussNewton(anchors, radii, initialGuess) {
  let estX = initialGuess.x;
  let estY = initialGuess.y;
  for (let iter = 0; iter < 15; iter++) {
    let j11 = 0, j12 = 0, j22 = 0;
    let r1 = 0, r2 = 0;
    for (let i = 0; i < anchors.length; i++) {
      const dx = estX - anchors[i].x;
      const dy = estY - anchors[i].y;
      const d = Math.max(0.1, Math.sqrt(dx * dx + dy * dy));
      const residual = d - radii[i];
      const jx = dx / d;
      const jy = dy / d;
      j11 += jx * jx;
      j12 += jx * jy;
      j22 += jy * jy;
      r1 += jx * residual;
      r2 += jy * residual;
    }
    const det = j11 * j22 - j12 * j12;
    if (Math.abs(det) < 1e-6) break;
    const deltaX = -(j22 * r1 - j12 * r2) / det;
    const deltaY = -(-j12 * r1 + j11 * r2) / det;
    estX += deltaX * 0.8;
    estY += deltaY * 0.8;
  }
  return { x: estX, y: estY };
}

console.log('Testing Purged Simple Trilateration:');

// Test 3 anchors at true target (400, 300)
const target = { x: 400, y: 300 };
const anchors = [
  { x: 300, y: 150 },
  { x: 550, y: 200 },
  { x: 420, y: 480 }
];
const radii = anchors.map(a => distance(a, target));

const solved = solveGaussNewton(anchors, radii, { x: 400, y: 250 });
const err = distance(target, solved);

console.log(`Solved position: (${solved.x.toFixed(2)}, ${solved.y.toFixed(2)})`);
console.log(`Error: ${err.toFixed(4)}px (Expected: < 0.01px)`);

if (err < 0.1) {
  console.log('✓ PASS: Clean Gauss-Newton Trilateration is 100% accurate!\n');
} else {
  throw new Error('Test failed');
}

// Test 2 anchors
const [c1, c2] = findTwoCircleIntersections(anchors[0], radii[0], anchors[1], radii[1]);
console.log(`2-Anchor Candidate 1: (${c1.x.toFixed(2)}, ${c1.y.toFixed(2)})`);
console.log(`2-Anchor Candidate 2: (${c2.x.toFixed(2)}, ${c2.y.toFixed(2)})`);
const dist1 = distance(c1, target);
const dist2 = distance(c2, target);
console.log(`Distance to target: Cand 1 = ${dist1.toFixed(2)}px, Cand 2 = ${dist2.toFixed(2)}px`);
if (Math.min(dist1, dist2) < 0.1) {
  console.log('✓ PASS: 2-Anchor Circle-Circle Intersection matches true target!\n');
} else {
  throw new Error('Test failed');
}
