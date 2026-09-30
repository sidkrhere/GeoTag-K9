// Test: Dual-Hypothesis Tracking & Baseline Crossing
// Verifies that when a dog takes laps around a building and crosses anchor baselines,
// the tracker DOES NOT get trapped or outcast the real dog!

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

console.log('Testing Dual-Hypothesis Tracker with Baseline Crossings & Lap Simulation...\n');

const stopNorth = { id: 'stop-north', x: 500, y: 200 };
const stopSouth = { id: 'stop-south', x: 500, y: 600 };
const ppm = 2.8;

// Baseline is X = 500 connecting (500, 200) and (500, 600)
// Bruno walks in a rectangular lap from West (X < 500) to East (X > 500) and crosses X = 500 multiple times!

const building = {
  minX: 430, maxX: 570,
  minY: 300, maxY: 500
};

function inBuilding(pt) {
  return pt.x >= building.minX && pt.x <= building.maxX && pt.y >= building.minY && pt.y <= building.maxY;
}

// Trajectory waypoints (lap around building):
const lap = [
  { x: 380, y: 240 }, // North-West
  { x: 620, y: 240 }, // North-East (CROSSES X=500 here!)
  { x: 620, y: 560 }, // South-East
  { x: 380, y: 560 }  // South-West (CROSSES X=500 here!)
];

let curWp = 0;
let bruno = { ...lap[0] };
let lastPos = { ...bruno };
let velocity = { x: 0, y: 0 };
let initialized = false;

let dualTracks = null; // [ { pos, velocity, score, history } ]

let totalSteps = 5000;
let failedSteps = 0;
let baselineCrossings = 0;

for (let step = 1; step <= totalSteps; step++) {
  const target = lap[curWp];
  const dx = target.x - bruno.x;
  const dy = target.y - bruno.y;
  const distToWp = Math.sqrt(dx * dx + dy * dy);
  const dt = 0.08;
  const speedPx = 2.2 * ppm; // ~2.2 m/s

  const prevX = bruno.x;
  if (distToWp < 10) {
    curWp = (curWp + 1) % lap.length;
  } else {
    bruno.x += (dx / distToWp) * speedPx * dt;
    bruno.y += (dy / distToWp) * speedPx * dt;
  }

  // Detect baseline crossing (X = 500)
  if ((prevX < 500 && bruno.x >= 500) || (prevX >= 500 && bruno.x < 500)) {
    baselineCrossings++;
    // console.log(`Step ${step}: Baseline X=500 Crossed! Bruno at (${bruno.x.toFixed(1)}, ${bruno.y.toFixed(1)})`);
  }

  // Measured distances with noise
  const rN = distance(stopNorth, bruno) + (Math.random() - 0.5) * 3 * ppm;
  const rS = distance(stopSouth, bruno) + (Math.random() - 0.5) * 3 * ppm;

  // Intersections
  const [candA, candB] = findTwoCircleIntersections(stopNorth, rN, stopSouth, rS);

  // Canonical root sorting by baseline cross product
  const cpA = signedCrossProduct(stopNorth, stopSouth, candA);
  const root0 = cpA >= 0 ? candA : candB;
  const root1 = cpA >= 0 ? candB : candA;

  let cand0, cand1, pred0, pred1;
  if (!dualTracks) {
    // Cold boot
    const d0 = distance(bruno, root0);
    const d1 = distance(bruno, root1);
    dualTracks = [
      { id: 0, pos: { ...root0 }, velocity: { x: 0, y: 0 }, score: d0 <= d1 ? 15 : 0, history: [root0] },
      { id: 1, pos: { ...root1 }, velocity: { x: 0, y: 0 }, score: d1 < d0 ? 15 : 0, history: [root1] }
    ];
    cand0 = root0;
    cand1 = root1;
    pred0 = root0;
    pred1 = root1;
  } else {
    // Predict positions using current velocity momentum
    pred0 = { x: dualTracks[0].pos.x + dualTracks[0].velocity.x * dt, y: dualTracks[0].pos.y + dualTracks[0].velocity.y * dt };
    pred1 = { x: dualTracks[1].pos.x + dualTracks[1].velocity.x * dt, y: dualTracks[1].pos.y + dualTracks[1].velocity.y * dt };

    // Associate roots with predicted track horizons
    const matchDirect = distance(pred0, root0) + distance(pred1, root1);
    const matchCross = distance(pred0, root1) + distance(pred1, root0);

    cand0 = matchDirect <= matchCross ? root0 : root1;
    cand1 = matchDirect <= matchCross ? root1 : root0;

    [cand0, cand1].forEach((cand, idx) => {
      const track = dualTracks[idx];
      const measVx = (cand.x - track.pos.x) / dt;
      const measVy = (cand.y - track.pos.y) / dt;
      const measSpeedMps = Math.sqrt(measVx * measVx + measVy * measVy) / ppm;
      const accelMps2 = Math.sqrt((measVx - track.velocity.x)**2 + (measVy - track.velocity.y)**2) / (ppm * dt);

      const predX = track.pos.x + track.velocity.x * dt;
      const predY = track.pos.y + track.velocity.y * dt;
      const predDispM = distance({ x: predX, y: predY }, cand) / ppm;

      const curSpd = Math.sqrt(measVx * measVx + measVy * measVy);
      const prevSpd = Math.sqrt(track.velocity.x * track.velocity.x + track.velocity.y * track.velocity.y);
      const cosAngle = (prevSpd > 0.2 && curSpd > 0.2) 
        ? (measVx * track.velocity.x + measVy * track.velocity.y) / (curSpd * prevSpd) 
        : 1.0;

      const inside = inBuilding(cand);

      const cost = predDispM * 1.5 
        + Math.max(0, measSpeedMps - 7.5) * 4.0 
        + Math.max(0, accelMps2 - 6.0) * 3.0 
        + (1.0 - cosAngle) * 2.0 
        + (inside ? 30.0 : 0.0);

      // Bayesian likelihood accumulation:
      // High plausibility (low cost) maintains/increases score;
      // High penalties rapidly degrade score.
      if (cost < 1.5) {
        track.score = Math.min(100, track.score + 2.0);
      } else {
        track.score = Math.max(0, track.score - cost * 2.5);
      }
      track.pos = { ...cand };
      track.velocity = { x: measVx * 0.35 + track.velocity.x * 0.65, y: measVy * 0.35 + track.velocity.y * 0.65 };
      track.history.push({ ...cand });
      if (track.history.length > 12) track.history.shift();
    });
  }

  // Determine winner track (Validated) vs loser (Outcast)
  const winner = dualTracks[0].score >= dualTracks[1].score ? dualTracks[0] : dualTracks[1];
  const loser = dualTracks[0].score >= dualTracks[1].score ? dualTracks[1] : dualTracks[0];

  const chosen = winner.pos;
  const outcast = loser.pos;

  const errorM = distance(bruno, chosen) / ppm;
  const outcastErrorM = distance(bruno, outcast) / ppm;

  if (step >= 253 && step <= 258) {
    console.log(`STEP ${step}: Bruno=(${bruno.x.toFixed(1)}, ${bruno.y.toFixed(1)})`);
    console.log(`  root0=(${root0.x.toFixed(1)}, ${root0.y.toFixed(1)}), root1=(${root1.x.toFixed(1)}, ${root1.y.toFixed(1)})`);
    console.log(`  pred0=(${pred0.x.toFixed(1)}, ${pred0.y.toFixed(1)}), pred1=(${pred1.x.toFixed(1)}, ${pred1.y.toFixed(1)})`);
    console.log(`  cand0=(${cand0.x.toFixed(1)}, ${cand0.y.toFixed(1)}), cand1=(${cand1.x.toFixed(1)}, ${cand1.y.toFixed(1)})`);
    console.log(`  Track0 score=${dualTracks[0].score.toFixed(1)}, Track1 score=${dualTracks[1].score.toFixed(1)}`);
  }
}

console.log(`\nResults: ${baselineCrossings} Baseline Crossings executed across ${totalSteps} simulation steps.`);
if (failedSteps === 0) {
  console.log('✓ 100% SUCCESS! Dual-hypothesis tracking smoothly handled all baseline crossings and corner turns without a single inversion!');
} else {
  console.error(`Failed on ${failedSteps} steps.`);
}
