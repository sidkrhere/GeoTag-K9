// Test New Jump-Killer Logic on the Exact Preset Across 600 Steps
import fs from 'fs';

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

function countBuildingWallIntersections(p1, p2, b) {
  let intersections = 0;
  const pts = b.points;
  for (let i = 0; i < pts.length; i++) {
    const nextIdx = (i + 1) % pts.length;
    if (lineSegmentsIntersect(p1, p2, pts[i], pts[nextIdx])) {
      intersections++;
    }
  }
  return intersections;
}

function signedCrossProduct(a, b, p) {
  return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

function isPointInPolygon(point, polygon) {
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i].x, yi = polygon[i].y;
    const xj = polygon[j].x, yj = polygon[j].y;
    const intersect = ((yi > point.y) !== (yj > point.y)) &&
      (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}

function findTwoCircleIntersections(a1, r1, a2, r2) {
  const d = distance(a1, a2);
  if (d < 1e-4) {
    return [{ x: a1.x + r1, y: a1.y }, { x: a1.x - r1, y: a1.y }];
  }
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

function solveGaussNewton(anchors, radii, initialGuess, ppm = 2.8) {
  let estX = initialGuess.x;
  let estY = initialGuess.y;
  const maxIter = 15;
  const damping = 0.8;
  const huberThreshold = 10.0 * ppm;

  for (let iter = 0; iter < maxIter; iter++) {
    let j11 = 0, j12 = 0, j22 = 0;
    let r1 = 0, r2 = 0;

    for (let i = 0; i < anchors.length; i++) {
      const dx = estX - anchors[i].x;
      const dy = estY - anchors[i].y;
      const d = Math.max(0.1, Math.sqrt(dx * dx + dy * dy));
      const residual = d - radii[i];
      const jx = dx / d;
      const jy = dy / d;
      const absR = Math.abs(residual);
      const weight = absR <= huberThreshold ? 1.0 : huberThreshold / absR;
      const wR = residual * weight;

      j11 += jx * jx * weight;
      j12 += jx * jy * weight;
      j22 += jy * jy * weight;
      r1 += jx * wR;
      r2 += jy * wR;
    }

    const det = j11 * j22 - j12 * j12;
    if (Math.abs(det) < 1e-6) break;

    const deltaX = -(j22 * r1 - j12 * r2) / det;
    const deltaY = -(-j12 * r1 + j11 * r2) / det;

    estX += deltaX * damping;
    estY += deltaY * damping;

    if (deltaX * deltaX + deltaY * deltaY < 0.01) break;
  }
  return { x: estX, y: estY };
}

// Robust Kalman Filter with Biomechanical Velocity Bound & NO instant teleportation
class RobustCanineKalmanFilter {
  constructor(initialPos, pixelsPerMeter = 2.8, maxSpeedMps = 12.0) {
    this.pixelsPerMeter = pixelsPerMeter;
    this.maxSpeedMps = maxSpeedMps;
    this.state = [initialPos.x, initialPos.y, 0, 0];
    this.lastUpdateMs = Date.now();
    this.consecutiveRejections = 0;
    this.P = [
      [16.0, 0.0, 0.0, 0.0],
      [0.0, 16.0, 0.0, 0.0],
      [0.0, 0.0, 25.0, 0.0],
      [0.0, 0.0, 0.0, 25.0]
    ];
  }

  predict(nowMs, headingRad) {
    const rawDt = (nowMs - this.lastUpdateMs) / 1000.0;
    const dt = Math.max(0.01, Math.min(0.25, rawDt));

    if (headingRad !== undefined) {
      const uCompassX = Math.sin(headingRad);
      const uCompassY = -Math.cos(headingRad);
      const currentSpeed = Math.hypot(this.state[2], this.state[3]);
      if (currentSpeed > 0.5 * this.pixelsPerMeter) {
        this.state[2] = this.state[2] * 0.85 + (uCompassX * currentSpeed) * 0.15;
        this.state[3] = this.state[3] * 0.85 + (uCompassY * currentSpeed) * 0.15;
      }
    }

    this.state[0] += this.state[2] * dt;
    this.state[1] += this.state[3] * dt;
    this.clampVelocity();

    const q = 15.0 * this.pixelsPerMeter * this.pixelsPerMeter;
    const dt2 = dt * dt;
    const dt3 = dt2 * dt;
    this.P[0][0] += (q * dt3) / 3.0 + 2.0 * dt * this.P[0][2] + dt2 * this.P[2][2];
    this.P[1][1] += (q * dt3) / 3.0 + 2.0 * dt * this.P[1][3] + dt2 * this.P[3][3];
    this.P[0][2] += (q * dt2) / 2.0 + dt * this.P[2][2];
    this.P[2][0] = this.P[0][2];
    this.P[1][3] += (q * dt2) / 2.0 + dt * this.P[3][3];
    this.P[3][1] = this.P[1][3];
    this.P[2][2] += q * dt;
    this.P[3][3] += q * dt;

    this.lastUpdateMs = nowMs;
    return { x: this.state[0], y: this.state[1] };
  }

  update(meas, measStdDevMeters = 2.0) {
    const rPix = measStdDevMeters * this.pixelsPerMeter;
    const rVar = rPix * rPix;
    const yX = meas.x - this.state[0];
    const yY = meas.y - this.state[1];

    const s00 = this.P[0][0] + rVar;
    const s11 = this.P[1][1] + rVar;
    const detS = s00 * s11;

    const dM2 = (s11 * yX * yX + s00 * yY * yY) / detS;
    const now = Date.now();
    const dt = Math.max(0.01, Math.min(0.25, (now - this.lastUpdateMs) / 1000.0));

    // Hard Biomechanical Displacement Gating:
    // Max physically possible jump in dt is maxSpeed * dt + 3.0 meters slack
    const measJumpMeters = Math.hypot(yX, yY) / this.pixelsPerMeter;
    const maxPhysJumpMeters = (this.maxSpeedMps * dt) + 3.0;

    if (measJumpMeters > maxPhysJumpMeters || dM2 > 50.0) {
      this.consecutiveRejections++;
      if (this.consecutiveRejections >= 5) {
        // Smooth rate-limited convergence (maximum 2.5m per step towards measurement)
        const stepMeters = Math.min(2.5, measJumpMeters * 0.4);
        const stepPix = stepMeters * this.pixelsPerMeter;
        const norm = Math.hypot(yX, yY) || 1;
        this.state[0] += (yX / norm) * stepPix;
        this.state[1] += (yY / norm) * stepPix;
        this.consecutiveRejections = 0;
      }
      this.lastUpdateMs = now;
      return { pos: { x: this.state[0], y: this.state[1] }, wasRejected: true };
    }

    this.consecutiveRejections = 0;
    const k00 = this.P[0][0] / s00;
    const k11 = this.P[1][1] / s11;
    const k20 = this.P[2][0] / s00;
    const k31 = this.P[3][1] / s11;

    this.state[0] += k00 * yX;
    this.state[1] += k11 * yY;
    this.state[2] += k20 * yX;
    this.state[3] += k31 * yY;

    this.clampVelocity();
    this.P[0][0] = Math.max(0.1, (1 - k00) * this.P[0][0]);
    this.P[1][1] = Math.max(0.1, (1 - k11) * this.P[1][1]);
    this.P[2][2] = Math.max(0.5, this.P[2][2] - k20 * this.P[0][2]);
    this.P[3][3] = Math.max(0.5, this.P[3][3] - k31 * this.P[1][3]);

    this.lastUpdateMs = now;
    return { pos: { x: this.state[0], y: this.state[1] }, wasRejected: false };
  }

  clampVelocity() {
    const maxSpeedPixPerSec = this.maxSpeedMps * this.pixelsPerMeter;
    const speed = Math.hypot(this.state[2], this.state[3]);
    if (speed > maxSpeedPixPerSec) {
      const scale = maxSpeedPixPerSec / speed;
      this.state[2] *= scale;
      this.state[3] *= scale;
    }
  }
}

// SETUP EXACT PRESET ENVIRONMENT
const cx = 600, cy = 400;
const buildings = [
  {
    id: 'b-main-complex',
    closed: true,
    attenuationDb: 14,
    points: [
      { x: cx - 130, y: cy - 85 },
      { x: cx + 130, y: cy - 85 },
      { x: cx + 130, y: cy + 85 },
      { x: cx - 130, y: cy + 85 }
    ]
  }
];

const stops = [
  { id: 'stop-north', name: 'North Plaza Mast #1', tier: 1, x: cx, y: cy - 190 },
  { id: 'stop-solar-3', name: 'Solar Pole #3', tier: 1, x: cx - 170, y: cy - 110 },
  { id: 'stop-solar-4', name: 'Solar Pole #4', tier: 1, x: cx + 170, y: cy - 110 },
  { id: 'stop-east', name: 'East Veranda Anchor #2', tier: 2, x: cx + 210, y: cy + 15 },
  { id: 'stop-solar-5', name: 'Solar Pole #5', tier: 1, x: cx + 170, y: cy + 100 },
  { id: 'stop-south', name: 'South Promenade Mast #3', tier: 1, x: cx, y: cy + 190 },
  { id: 'stop-west', name: 'West Garden Anchor #4', tier: 2, x: cx - 210, y: cy + 90 }
];

const config = {
  tier1RadiusMeters: 90,
  tier2RadiusMeters: 45,
  noiseStdDevDb: 3.0,
  wallAttenuationDb: 14,
  pathLossExponent: 3.0,
  pixelsPerMeter: 2.8,
  autoWalkSpeed: 2.5
};

const lapPath = [
  { x: cx - 165, y: cy - 120 },
  { x: cx + 165, y: cy - 120 },
  { x: cx + 165, y: cy + 120 },
  { x: cx - 165, y: cy + 120 }
];

function calculateSignalMeasurement(tag, stop) {
  const pixelDist = distance(tag, stop);
  const trueDistanceMeters = pixelDist / config.pixelsPerMeter;
  const maxRadiusMeters = stop.tier === 1 ? config.tier1RadiusMeters : config.tier2RadiusMeters;
  if (trueDistanceMeters > maxRadiusMeters) return null;

  let totalWallIntersections = 0;
  for (const b of buildings) {
    if (b.closed && b.points.length >= 3) {
      totalWallIntersections += countBuildingWallIntersections(tag, stop, b);
    }
  }

  const A = -42;
  const n = config.pathLossExponent;
  const wallLossDb = totalWallIntersections * config.wallAttenuationDb;
  const u1 = Math.max(1e-10, Math.random());
  const u2 = Math.random();
  const z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
  const channelNoiseDb = z0 * config.noiseStdDevDb;

  const safeDist = Math.max(0.2, trueDistanceMeters);
  const rssiDbm = A - (10 * n * Math.log10(safeDist)) - wallLossDb + channelNoiseDb;
  const effectiveRssiNoWalls = rssiDbm + (wallLossDb * 0.5);
  const noisyExponent = (A - effectiveRssiNoWalls) / (10 * n);
  const noisyDistanceMeters = Math.max(0.1, Math.pow(10, noisyExponent));

  return {
    stopId: stop.id,
    trueDistanceMeters,
    noisyDistanceMeters,
    obstructed: totalWallIntersections > 0,
    isNlosZeroWeighted: totalWallIntersections > 0
  };
}

// RUN WITH TRUE HISTORY-CONSENSUS OUTCASTING
console.log('=== TESTING NEW JUMP-KILLER LOGIC ACROSS 600 STEPS ===');
const kf = new RobustCanineKalmanFilter(lapPath[0], config.pixelsPerMeter, 12.0);
let tag = { x: lapPath[0].x, y: lapPath[0].y, headingDeg: 90, currentWaypointIndex: 1 };
let lastEstimatedPos = { ...lapPath[0] };

// ACTIVE HISTORY BUFFER (Used for candidate validation & voting!)
const historyTrail = [{ ...lapPath[0] }];
const MAX_HISTORY = 12;

const dt = 0.050;
const stepPixels = config.autoWalkSpeed * config.pixelsPerMeter * dt;

let maxJumpDetected = 0;
let jumpCount = 0;
let totalError = 0;
let sampleCount = 0;

for (let step = 0; step < 600; step++) {
    // log after calculation
  const targetWp = lapPath[tag.currentWaypointIndex % lapPath.length];
  const dx = targetWp.x - tag.x;
  const dy = targetWp.y - tag.y;
  const distToWp = Math.hypot(dx, dy);

  let vx = 0, vy = 0;
  if (distToWp < 10) {
    tag.currentWaypointIndex = (tag.currentWaypointIndex + 1) % lapPath.length;
    const nextWp = lapPath[tag.currentWaypointIndex];
    const ndx = nextWp.x - tag.x;
    const ndy = nextWp.y - tag.y;
    const ndist = Math.hypot(ndx, ndy) || 1;
    vx = (ndx / ndist) * stepPixels;
    vy = (ndy / ndist) * stepPixels;
  } else {
    vx = (dx / distToWp) * stepPixels;
    vy = (dy / distToWp) * stepPixels;
  }
  tag.x += vx;
  tag.y += vy;
  if (Math.hypot(vx, vy) > 0.05) {
    tag.headingDeg = (Math.atan2(vx, -vy) * (180 / Math.PI) + 360) % 360;
  }

  const measurements = [];
  for (const s of stops) {
    const sig = calculateSignalMeasurement(tag, s);
    if (sig) measurements.push(sig);
  }

  const validAnchors = [];
  for (const m of measurements) {
    const s = stops.find(stop => stop.id === m.stopId);
    if (!m.obstructed) {
      validAnchors.push({ id: s.id, point: { x: s.x, y: s.y }, radius: m.noisyDistanceMeters * config.pixelsPerMeter });
    }
  }

  const nowMs = Date.now() + step * 50;
  const headingRad = (tag.headingDeg * Math.PI) / 180;
  const predPos = kf.predict(nowMs, headingRad);

  // 1. Compute Historical Trajectory Consensus Point
  // Extrapolate recent history using moving average
  let histCentroid = { x: 0, y: 0 };
  let histWeightTotal = 0;
  for (let i = 0; i < historyTrail.length; i++) {
    const w = i + 1; // More weight to recent points
    histCentroid.x += historyTrail[i].x * w;
    histCentroid.y += historyTrail[i].y * w;
    histWeightTotal += w;
  }
  histCentroid.x /= histWeightTotal;
  histCentroid.y /= histWeightTotal;

  // Expected reference position is blend of predPos and historical centroid
  const consensusRef = {
    x: predPos.x * 0.70 + histCentroid.x * 0.30,
    y: predPos.y * 0.70 + histCentroid.y * 0.30
  };

  let estPos = { ...predPos };

  if (validAnchors.length === 0) {
    estPos = { ...consensusRef };
  } else if (validAnchors.length === 1) {
    // 1 Clean Anchor: Radial projection with high uncertainty
    const a = validAnchors[0];
    const adx = consensusRef.x - a.point.x;
    const ady = consensusRef.y - a.point.y;
    const ad = Math.hypot(adx, ady) || 1;
    const ringPoint = {
      x: a.point.x + (adx / ad) * a.radius,
      y: a.point.y + (ady / ad) * a.radius
    };
    // Higher measurement noise for 1-anchor to prevent pulling off-track
    const kfRes = kf.update(ringPoint, 4.5);
    estPos = { ...kfRes.pos };
  } else if (validAnchors.length === 2) {
    // 2 Clean Anchors: TRUE HISTORY-CONSENSUS OUTCASTING!
    const a1 = validAnchors[0];
    const a2 = validAnchors[1];
    const [cand1, cand2] = findTwoCircleIntersections(a1.point, a1.radius, a2.point, a2.radius);

    // Vote on which candidate is closest to the historical consensus reference!
    const d1 = distance(cand1, consensusRef);
    const d2 = distance(cand2, consensusRef);

    // Baseline side voting from history
    let votes1 = 0, votes2 = 0;
    const sign1 = Math.sign(signedCrossProduct(a1.point, a2.point, cand1));
    for (const h of historyTrail) {
      const hSign = Math.sign(signedCrossProduct(a1.point, a2.point, h));
      if (hSign === sign1) votes1++;
      else votes2++;
    }

    // Candidate 1 score:
    // Distance from consensus + baseline history mismatch penalty + building penalty
    const inB1 = isPointInPolygon(cand1, buildings[0].points);
    const inB2 = isPointInPolygon(cand2, buildings[0].points);

    const score1 = d1 + (votes2 > votes1 ? 25.0 : 0) + (inB1 ? 50.0 : 0);
    const score2 = d2 + (votes1 > votes2 ? 25.0 : 0) + (inB2 ? 50.0 : 0);

    if (step === 40 || step === 50) {
    console.log(`Step ${step} Candidates:`);
    console.log(`  tag: (${tag.x.toFixed(1)}, ${tag.y.toFixed(1)})`);
    console.log(`  consensusRef: (${consensusRef.x.toFixed(1)}, ${consensusRef.y.toFixed(1)})`);
    console.log(`  cand1: (${cand1.x.toFixed(1)}, ${cand1.y.toFixed(1)}) | distToTag: ${(distance(cand1, tag)/2.8).toFixed(1)}m | dToConsensus: ${(d1/2.8).toFixed(1)}m | score: ${score1.toFixed(1)}`);
    console.log(`  cand2: (${cand2.x.toFixed(1)}, ${cand2.y.toFixed(1)}) | distToTag: ${(distance(cand2, tag)/2.8).toFixed(1)}m | dToConsensus: ${(d2/2.8).toFixed(1)}m | score: ${score2.toFixed(1)}`);
  }
  const chosen = score1 <= score2 ? cand1 : cand2;
    const kfRes = kf.update(chosen, 2.0);
    estPos = { ...kfRes.pos };
  } else {
    // 3 or more anchors: Gauss-Newton initialized at consensusRef
    const solved = solveGaussNewton(
      validAnchors.map(a => a.point),
      validAnchors.map(a => a.radius),
      consensusRef,
      config.pixelsPerMeter
    );

    // Sanity check: If Gauss-Newton produced a point > 15m from consensus, reject or clamp!
    const gnDist = distance(solved, consensusRef) / config.pixelsPerMeter;
    const validSolved = gnDist < 15.0 ? solved : consensusRef;

    const kfRes = kf.update(validSolved, 2.2);
    estPos = { ...kfRes.pos };
  }

  // Update history trail
  historyTrail.push({ ...estPos });
  if (historyTrail.length > MAX_HISTORY) historyTrail.shift();

  // Jump tracking
  const jumpMeters = Math.hypot(estPos.x - lastEstimatedPos.x, estPos.y - lastEstimatedPos.y) / config.pixelsPerMeter;
  const errorMeters = Math.hypot(estPos.x - tag.x, estPos.y - tag.y) / config.pixelsPerMeter;
  if (errorMeters > 5.0 && !global.firstLogged) {
    global.firstLogged = true;
    console.log(`FIRST HIGH ERROR AT STEP ${step}: tag=(${Math.round(tag.x)}, ${Math.round(tag.y)}) | est=(${Math.round(estPos.x)}, ${Math.round(estPos.y)}) | err=${errorMeters.toFixed(1)}m | anchors=${validAnchors.length} (${validAnchors.map(a => a.id).join(',')})`);
  }

  totalError += errorMeters;
  sampleCount++;

  if (jumpMeters > maxJumpDetected) maxJumpDetected = jumpMeters;
  if (jumpMeters > 5.0) jumpCount++;

  if (step >= 40 && step <= 70) {
    console.log(`Step ${step}: tag=(${Math.round(tag.x)}, ${Math.round(tag.y)}) | est=(${Math.round(estPos.x)}, ${Math.round(estPos.y)}) | err=${errorMeters.toFixed(1)}m | anchors=${validAnchors.length} (${validAnchors.map(a => a.id).join(',')})`);
  }
  lastEstimatedPos = { ...estPos };
}

console.log(`\n=== RESULTS WITH NEW LOGIC ===`);
console.log(`Max jump detected: ${maxJumpDetected.toFixed(2)} meters! (WAS 75.4 meters!)`);
console.log(`Total jumps > 5m: ${jumpCount} (WAS 507!)`);
console.log(`Average localization error: ${(totalError / sampleCount).toFixed(2)} meters!`);
