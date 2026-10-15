// Comprehensive Audit Script for GeoTag K9 Multilateration & Jumps
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

// Exactly the app's current CanineKalmanFilter from src/engine/kalmanFilter.ts
class AppCanineKalmanFilter {
  constructor(initialPos, pixelsPerMeter = 2.8, maxSpeedMps = 15.0) {
    this.pixelsPerMeter = pixelsPerMeter;
    this.maxSpeedMps = Math.max(2.0, maxSpeedMps);
    this.state = [initialPos.x, initialPos.y, 0, 0];
    this.lastUpdateMs = Date.now();
    this.consecutiveRejections = 0;
    this.smoothedSpeedMps = 0;
    this.P = [
      [16.0, 0.0, 0.0, 0.0],
      [0.0, 16.0, 0.0, 0.0],
      [0.0, 0.0, 25.0, 0.0],
      [0.0, 0.0, 0.0, 25.0]
    ];
    this.initialized = true;
  }

  predict(nowMs, headingRad) {
    if (!this.initialized) return { x: this.state[0], y: this.state[1] };
    const rawDt = (nowMs - this.lastUpdateMs) / 1000.0;
    const dt = Math.max(0.01, Math.min(0.25, rawDt));

    if (headingRad !== undefined) {
      const uCompassX = Math.sin(headingRad);
      const uCompassY = -Math.cos(headingRad);
      const currentSpeed = Math.sqrt(this.state[2] * this.state[2] + this.state[3] * this.state[3]);
      if (currentSpeed > 0.5 * this.pixelsPerMeter) {
        const guidedVx = this.state[2] * 0.80 + (uCompassX * currentSpeed) * 0.20;
        const guidedVy = this.state[3] * 0.80 + (uCompassY * currentSpeed) * 0.20;
        this.state[2] = guidedVx;
        this.state[3] = guidedVy;
      }
    }

    this.state[0] += this.state[2] * dt;
    this.state[1] += this.state[3] * dt;
    this.clampVelocity();

    const currentSpeedMps = Math.hypot(this.state[2], this.state[3]) / this.pixelsPerMeter;
    const dynamicMaxAccel = Math.max(25.0, Math.max(this.maxSpeedMps, currentSpeedMps) * 2.5);
    const q = dynamicMaxAccel * this.pixelsPerMeter * this.pixelsPerMeter;

    const dt2 = dt * dt;
    const dt3 = dt2 * dt;
    const q11 = (q * dt3) / 3.0;
    const q12 = (q * dt2) / 2.0;
    const q22 = q * dt;

    const p00 = this.P[0][0] + 2.0 * dt * this.P[0][2] + dt2 * this.P[2][2] + q11;
    const p11 = this.P[1][1] + 2.0 * dt * this.P[1][3] + dt2 * this.P[3][3] + q11;
    const p02 = this.P[0][2] + dt * this.P[2][2] + q12;
    const p13 = this.P[1][3] + dt * this.P[3][3] + q12;
    const p22 = this.P[2][2] + q22;
    const p33 = this.P[3][3] + q22;

    this.P[0][0] = p00;
    this.P[1][1] = p11;
    this.P[0][2] = p02;
    this.P[2][0] = p02;
    this.P[1][3] = p13;
    this.P[3][1] = p13;
    this.P[2][2] = p22;
    this.P[3][3] = p33;

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
    const s01 = this.P[0][1];
    const detS = s00 * s11 - s01 * s01;

    if (detS < 1e-6) return { pos: { x: this.state[0], y: this.state[1] }, wasRejected: false };

    const dM2 = (s11 * yX * yX - 2.0 * s01 * yX * yY + s00 * yY * yY) / detS;
    const now = Date.now();
    const dt = Math.max(0.01, Math.min(0.25, (now - this.lastUpdateMs) / 1000.0));

    if (dM2 > 60.0) {
      this.consecutiveRejections++;
      if (this.consecutiveRejections >= 2) {
        // FAST RECOVERY TELEPORT!
        this.state[0] = meas.x;
        this.state[1] = meas.y;
        this.state[2] = yX / dt;
        this.state[3] = yY / dt;
        this.clampVelocity();
        this.P[0][0] = Math.min(100.0, this.P[0][0] * 1.5);
        this.P[1][1] = Math.min(100.0, this.P[1][1] * 1.5);
        this.lastUpdateMs = now;
        return { pos: { x: this.state[0], y: this.state[1] }, wasRejected: false, isTeleport: true, dM2 };
      }
      return { pos: { x: this.state[0], y: this.state[1] }, wasRejected: true, dM2 };
    }

    this.consecutiveRejections = 0;
    const invS00 = s11 / detS;
    const invS11 = s00 / detS;
    const invS01 = -s01 / detS;

    const k00 = this.P[0][0] * invS00 + this.P[0][1] * invS01;
    const k01 = this.P[0][0] * invS01 + this.P[0][1] * invS11;
    const k10 = this.P[1][0] * invS00 + this.P[1][1] * invS01;
    const k11 = this.P[1][0] * invS01 + this.P[1][1] * invS11;
    const k20 = this.P[2][0] * invS00 + this.P[2][1] * invS01;
    const k21 = this.P[2][0] * invS01 + this.P[2][1] * invS11;
    const k30 = this.P[3][0] * invS00 + this.P[3][1] * invS01;
    const k31 = this.P[3][0] * invS01 + this.P[3][1] * invS11;

    this.state[0] += k00 * yX + k01 * yY;
    this.state[1] += k10 * yX + k11 * yY;
    this.state[2] += k20 * yX + k21 * yY;
    this.state[3] += k30 * yX + k31 * yY;

    this.clampVelocity();
    this.lastUpdateMs = now;
    return { pos: { x: this.state[0], y: this.state[1] }, wasRejected: false, dM2 };
  }

  clampVelocity() {
    const maxCatchUpMps = Math.max(25.0, this.maxSpeedMps * 2.0);
    const maxPixPerSec = maxCatchUpMps * this.pixelsPerMeter;
    const speed = Math.hypot(this.state[2], this.state[3]);
    if (speed > maxPixPerSec) {
      const scale = maxPixPerSec / speed;
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
  { x: cx - 165, y: cy - 120 }, // Turn 1: (435, 280)
  { x: cx + 165, y: cy - 120 }, // Turn 2: (765, 280)
  { x: cx + 165, y: cy + 120 }, // Turn 3: (765, 520)
  { x: cx - 165, y: cy + 120 }  // Turn 4: (435, 520)
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
  // Box-Muller Gaussian noise
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

// SIMULATE MULTILATERATION PIPELINE ACROSS 600 STEPS (3 FULL LAPS)
console.log('=== RUNNING MULTILATERATION AUDIT PASS ACROSS 600 STEPS ===');
const kf = new AppCanineKalmanFilter(lapPath[0], config.pixelsPerMeter, 15.0);
let tag = { x: lapPath[0].x, y: lapPath[0].y, headingDeg: 90, currentWaypointIndex: 1 };
let lastEstimatedPos = { ...lapPath[0] };
let priorFix = { ...lapPath[0] };

const dt = 0.050; // 50ms interval
const stepPixels = config.autoWalkSpeed * config.pixelsPerMeter * dt;

let maxJumpDetected = 0;
let jumpCount = 0;
const jumpsLog = [];

for (let step = 0; step < 600; step++) {
  // Move tag along waypoint path
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

  // Calculate RF signals
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

  // Now solve trilateration exactly like multilateration.ts:
  const nowMs = Date.now() + step * 50;
  const headingRad = (tag.headingDeg * Math.PI) / 180;
  const predPos = kf.predict(nowMs, headingRad);

  let estPos = { ...predPos };
  let lockType = 'NONE';
  let chosenRaw = null;

  if (validAnchors.length === 0) {
    lockType = 'BLIND_ZONE';
    estPos = { ...priorFix };
  } else if (validAnchors.length === 1) {
    lockType = 'RADIAL_RING';
    const a = validAnchors[0];
    const adx = predPos.x - a.point.x;
    const ady = predPos.y - a.point.y;
    const ad = Math.hypot(adx, ady) || 1;
    const ringPoint = {
      x: a.point.x + (adx / ad) * a.radius,
      y: a.point.y + (ady / ad) * a.radius
    };
    const kfRes = kf.update(ringPoint, 2.5);
    estPos = { ...kfRes.pos };
    chosenRaw = ringPoint;
  } else if (validAnchors.length === 2) {
    lockType = 'DUAL';
    const a1 = validAnchors[0];
    const a2 = validAnchors[1];
    const [cand1, cand2] = findTwoCircleIntersections(a1.point, a1.radius, a2.point, a2.radius);

    const uCompass = { x: Math.sin(headingRad), y: -Math.cos(headingRad) };
    const priorPt = priorFix;
    const separationChordMeters = Math.round((distance(cand1, cand2) / config.pixelsPerMeter) * 10) / 10;
    const side1 = signedCrossProduct(a1.point, a2.point, cand1);
    const side2 = signedCrossProduct(a1.point, a2.point, cand2);
    const priorSide = signedCrossProduct(a1.point, a2.point, priorPt);
    const isSameSide1 = Math.sign(side1) === Math.sign(priorSide) || Math.abs(priorSide) < 1e-4;
    const isSameSide2 = Math.sign(side2) === Math.sign(priorSide) || Math.abs(priorSide) < 1e-4;

    function evalCand(cand, isSameSide) {
      const cdx = cand.x - priorPt.x;
      const cdy = cand.y - priorPt.y;
      const distPix = Math.hypot(cdx, cdy);
      const distM = distPix / config.pixelsPerMeter;
      const align = distPix > 0.5 ? (cdx * uCompass.x + cdy * uCompass.y) / distPix : 1.0;
      const alignScore = -align * 50.0;
      const distScore = distM * 3.5;
      const chordPenalty = (!isSameSide && separationChordMeters > 8.0) ? (separationChordMeters * 6.0) : 0.0;
      const inB = isPointInPolygon(cand, buildings[0].points);
      const wallScore = inB ? 40.0 : 0.0;
      return { score: alignScore + distScore + chordPenalty + wallScore, align, distM };
    }

    const r1 = evalCand(cand1, isSameSide1);
    const r2 = evalCand(cand2, isSameSide2);
    const chosen = r1.score <= r2.score ? cand1 : cand2;
    chosenRaw = chosen;
    const kfRes = kf.update(chosen, 1.8);
    estPos = { ...kfRes.pos };
  } else {
    // 3 or more anchors
    lockType = '3D_LOCK';
    const solved = solveGaussNewton(
      validAnchors.map(a => a.point),
      validAnchors.map(a => a.radius),
      predPos,
      config.pixelsPerMeter
    );
    chosenRaw = solved;
    const kfRes = kf.update(solved, 2.5);
    estPos = { ...kfRes.pos };
  }

  priorFix = { ...estPos };

  // Calculate jump from previous frame's estimated position
  const jumpMeters = Math.hypot(estPos.x - lastEstimatedPos.x, estPos.y - lastEstimatedPos.y) / config.pixelsPerMeter;
  const errorMeters = Math.hypot(estPos.x - tag.x, estPos.y - tag.y) / config.pixelsPerMeter;

  if (jumpMeters > maxJumpDetected) maxJumpDetected = jumpMeters;

  if (jumpMeters > 20.0) {
    jumpCount++;
    jumpsLog.push({
      step,
      tag: { x: Math.round(tag.x), y: Math.round(tag.y) },
      jumpMeters: Math.round(jumpMeters * 10) / 10,
      errorMeters: Math.round(errorMeters * 10) / 10,
      lockType,
      validAnchorCount: validAnchors.length,
      validAnchorIds: validAnchors.map(a => a.id).join(','),
      prevEst: { x: Math.round(lastEstimatedPos.x), y: Math.round(lastEstimatedPos.y) },
      currEst: { x: Math.round(estPos.x), y: Math.round(estPos.y) }
    });
  }

  lastEstimatedPos = { ...estPos };
}

console.log(`\nMax jump detected: ${maxJumpDetected.toFixed(1)} meters!`);
console.log(`Total jumps > 20m: ${jumpCount}`);
console.log('Detailed big jumps (>20m):');
console.log(JSON.stringify(jumpsLog.slice(0, 20), null, 2));
