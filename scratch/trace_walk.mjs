// Full standalone trace of the app's exact logic
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

class CanineKalmanFilter {
  constructor(initialPos, pixelsPerMeter = 2.8, maxSpeedMps = 8.5) {
    this.pixelsPerMeter = pixelsPerMeter;
    this.maxSpeedMps = maxSpeedMps;
    this.state = [initialPos.x, initialPos.y, 0, 0];
    this.lastUpdateMs = Date.now();
    this.P = [
      [16.0, 0.0, 0.0, 0.0],
      [0.0, 16.0, 0.0, 0.0],
      [0.0, 0.0, 25.0, 0.0],
      [0.0, 0.0, 0.0, 25.0]
    ];
    this.initialized = true;
    this.consecutiveRejections = 0;
  }

  reset(pos) {
    this.state = [pos.x, pos.y, 0, 0];
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
    if (!this.initialized) return { x: this.state[0], y: this.state[1] };
    const rawDt = (nowMs - this.lastUpdateMs) / 1000.0;
    const dt = Math.max(0.01, Math.min(0.25, rawDt));

    if (headingRad !== undefined) {
      const uCompassX = Math.sin(headingRad);
      const uCompassY = -Math.cos(headingRad);
      const currentSpeed = Math.sqrt(this.state[2] * this.state[2] + this.state[3] * this.state[3]);
      if (currentSpeed > 0.5 * this.pixelsPerMeter) {
        const guidedVx = this.state[2] * 0.75 + (uCompassX * currentSpeed) * 0.25;
        const guidedVy = this.state[3] * 0.75 + (uCompassY * currentSpeed) * 0.25;
        this.state[2] = guidedVx;
        this.state[3] = guidedVy;
      }
    }

    this.state[0] += this.state[2] * dt;
    this.state[1] += this.state[3] * dt;
    this.clampVelocity();

    const maxAccelMps2 = 6.0;
    const q = maxAccelMps2 * this.pixelsPerMeter * this.pixelsPerMeter;
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
    
    // Gating check: if dM2 is high, check if consecutive rejections indicate filter divergence
    if (dM2 > 25.0) {
      this.consecutiveRejections++;
      if (this.consecutiveRejections >= 3) {
        // Recover filter: Re-anchor state to measurement
        this.state[0] = meas.x;
        this.state[1] = meas.y;
        this.state[2] = 0;
        this.state[3] = 0;
        this.consecutiveRejections = 0;
        this.P[0][0] = rVar;
        this.P[1][1] = rVar;
        return { pos: { x: this.state[0], y: this.state[1] }, wasRejected: false, recovered: true };
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
    return { pos: { x: this.state[0], y: this.state[1] }, wasRejected: false, dM2 };
  }


  clampVelocity() {
    const maxPixPerSec = this.maxSpeedMps * this.pixelsPerMeter;
    const currentSpeedPix = Math.sqrt(this.state[2] * this.state[2] + this.state[3] * this.state[3]);
    if (currentSpeedPix > maxPixPerSec && currentSpeedPix > 1e-4) {
      const ratio = maxPixPerSec / currentSpeedPix;
      this.state[2] *= ratio;
      this.state[3] *= ratio;
    }
  }
}

// Layout
const cx = 600, cy = 400;
const building = {
  closed: true,
  points: [
    { x: cx - 130, y: cy - 85 }, // 470, 315
    { x: cx + 130, y: cy - 85 }, // 730, 315
    { x: cx + 130, y: cy + 85 }, // 730, 485
    { x: cx - 130, y: cy + 85 }  // 470, 485
  ]
};

const stops = [
  { id: 'stop-north', name: 'North Plaza Mast #1', tier: 1, x: cx, y: cy - 135 }, // 600, 265
  { id: 'stop-east', name: 'East Veranda Anchor #2', tier: 2, x: cx + 190, y: cy }, // 790, 400
  { id: 'stop-south', name: 'South Promenade Mast #3', tier: 1, x: cx, y: cy + 135 }, // 600, 535
  { id: 'stop-west', name: 'West Garden Anchor #4', tier: 2, x: cx - 190, y: cy } // 410, 400
];

const config = {
  tier1RadiusMeters: 90, // 252px
  tier2RadiusMeters: 45, // 126px
  noiseStdDevDb: 3.0,
  wallAttenuationDb: 12,
  pathLossExponent: 3.0,
  pixelsPerMeter: 2.8,
  autoWalkSpeed: 1.2
};

function calculateSignalMeasurement(tag, stop) {
  const pixelDist = distance(tag, stop);
  const trueDistanceMeters = pixelDist / config.pixelsPerMeter;
  const maxRadiusMeters = stop.tier === 1 ? config.tier1RadiusMeters : config.tier2RadiusMeters;
  if (trueDistanceMeters > maxRadiusMeters) return null;

  const totalWalls = countBuildingWallIntersections(tag, stop, building);
  const A = -42;
  const n = config.pathLossExponent;
  const wallLossDb = totalWalls * config.wallAttenuationDb;
  const safeDist = Math.max(0.2, trueDistanceMeters);
  const rssiDbm = A - (10 * n * Math.log10(safeDist)) - wallLossDb;

  return {
    stopId: stop.id,
    trueDistanceMeters,
    noisyDistanceMeters: trueDistanceMeters, // ideal for testing
    rssiDbm: Math.round(rssiDbm * 10) / 10,
    obstructed: totalWalls > 0,
    numObstructions: totalWalls
  };
}

// Let's trace Bruno moving along the path:
// WP 0: (435, 280) -> WP 1: (765, 280) -> WP 2: (765, 520)
const kf = new CanineKalmanFilter({ x: 435, y: 280 }, 2.8, 8.5);
let tag = { x: 435, y: 280, headingDeg: 90 };
let now = Date.now();

function solveGaussNewton(anchors, radii, initialGuess) {
  let estX = initialGuess.x;
  let estY = initialGuess.y;
  for (let iter = 0; iter < 15; iter++) {
    let j11 = 0, j12 = 0, j22 = 0, r1 = 0, r2 = 0;
    for (let i = 0; i < anchors.length; i++) {
      const dx = estX - anchors[i].x;
      const dy = estY - anchors[i].y;
      const d = Math.max(0.1, Math.sqrt(dx * dx + dy * dy));
      const residual = d - radii[i];
      const jx = dx / d, jy = dy / d;
      j11 += jx * jx; j12 += jx * jy; j22 += jy * jy;
      r1 += jx * residual; r2 += jy * residual;
    }
    const det = j11 * j22 - j12 * j12;
    if (Math.abs(det) < 1e-6) break;
    const deltaX = -(j22 * r1 - j12 * r2) / det;
    const deltaY = -(-j12 * r1 + j11 * r2) / det;
    estX += deltaX * 0.8;
    estY += deltaY * 0.8;
    if (deltaX * deltaX + deltaY * deltaY < 0.01) break;
  }
  return { x: estX, y: estY };
}

console.log('--- STARTING STEP-BY-STEP TRACE WITH SOLVE ---');
for (let step = 0; step <= 20; step++) {
  tag.x = 435 + (765 - 435) * (step / 20);
  tag.y = 280;
  tag.headingDeg = 90;
  now += 100;

  const headingRad = (tag.headingDeg * Math.PI) / 180;
  const predPos = kf.predict(now, headingRad);

  const sigs = stops.map(s => calculateSignalMeasurement(tag, s)).filter(Boolean);

  // NLOS raycast check:
  // An anchor is zero-weighted NLOS if its physical path is obstructed by building walls
  // or if the server raycast from estimated position confirms 15" concrete wall penetration.
  // BUT if physical path is clear (m.obstructed === false), it is CLEAR LINE OF SIGHT!
  const validAnchors = [];
  const zeroWeightedAnchors = [];
  for (const m of sigs) {
    const stop = stops.find(s => s.id === m.stopId);
    let totalWalls = countBuildingWallIntersections(predPos, stop, building);
    // If the physical measurement is obstructed, OR (if predPos is reliable and walls > 0)
    // Note: if m.obstructed is false, physical radio wave did NOT pass through any walls!
    const isObstructed = m.obstructed || (totalWalls > 0 && distance(predPos, tag) < 50);
    if (isObstructed) {
      m.isNlosZeroWeighted = true;
      m.weight = 0.0;
      zeroWeightedAnchors.push(stop.id);
    } else {
      m.isNlosZeroWeighted = false;
      m.weight = 1.0;
      validAnchors.push({ id: stop.id, point: { x: stop.x, y: stop.y }, radius: m.noisyDistanceMeters * config.pixelsPerMeter });
    }
  }

  let lockType = 'NONE';
  let updatePos = null;
  if (validAnchors.length === 0) {
    lockType = 'BLIND_ZONE';
  } else if (validAnchors.length === 1) {
    lockType = 'RADIAL_RING';
    const a = validAnchors[0];
    const dx = predPos.x - a.point.x;
    const dy = predPos.y - a.point.y;
    const d = Math.hypot(dx, dy) || 1;
    const ringPoint = {
      x: a.point.x + (dx / d) * a.radius,
      y: a.point.y + (dy / d) * a.radius
    };
    const res = kf.update(ringPoint, 2.5);
    updatePos = res.pos;
  } else if (validAnchors.length === 2) {
    lockType = 'DUAL';
    const [cand1, cand2] = findTwoCircleIntersections(validAnchors[0].point, validAnchors[0].radius, validAnchors[1].point, validAnchors[1].radius);
    // choose candidate closest to predPos
    const d1 = distance(cand1, predPos);
    const d2 = distance(cand2, predPos);
    const chosen = d1 <= d2 ? cand1 : cand2;
    const res = kf.update(chosen, 2.0);
    updatePos = res.pos;
  } else if (validAnchors.length >= 3) {
    lockType = '3-ANCHOR';
    const solved = solveGaussNewton(validAnchors.map(a => a.point), validAnchors.map(a => a.radius), predPos);
    const res = kf.update(solved, 1.5);
    updatePos = res.pos;
  }

  console.log(`Step ${step}: tag=(${tag.x.toFixed(0)}, ${tag.y.toFixed(0)}) | predPos=(${predPos.x.toFixed(1)}, ${predPos.y.toFixed(1)}) | valid=[${validAnchors.map(a=>a.id).join(',')}] | zero=[${zeroWeightedAnchors.join(',')}] | lock=${lockType}`);
}

for (let step = 21; step <= 40; step++) {
  tag.x = 765;

  tag.y = 280 + (400 - 280) * ((step - 20) / 20);
  tag.headingDeg = 180;
  now += 100;

  const headingRad = (tag.headingDeg * Math.PI) / 180;
  const predPos = kf.predict(now, headingRad);

  const sigs = stops.map(s => calculateSignalMeasurement(tag, s)).filter(Boolean);

  const validAnchors = [];
  const zeroWeightedAnchors = [];
  for (const m of sigs) {
    const stop = stops.find(s => s.id === m.stopId);
    let totalWalls = countBuildingWallIntersections(predPos, stop, building);
    const isObstructed = m.obstructed || (totalWalls > 0 && distance(predPos, tag) < 50);
    if (isObstructed) {
      m.isNlosZeroWeighted = true;
      m.weight = 0.0;
      zeroWeightedAnchors.push(stop.id);
    } else {
      m.isNlosZeroWeighted = false;
      m.weight = 1.0;
      validAnchors.push({ id: stop.id, point: { x: stop.x, y: stop.y }, radius: m.noisyDistanceMeters * config.pixelsPerMeter });
    }
  }

  let lockType = 'NONE';
  let updatePos = null;
  if (validAnchors.length === 0) {
    lockType = 'BLIND_ZONE';
  } else if (validAnchors.length === 1) {
    lockType = 'RADIAL_RING';
    const a = validAnchors[0];
    const dx = predPos.x - a.point.x;
    const dy = predPos.y - a.point.y;
    const d = Math.hypot(dx, dy) || 1;
    const ringPoint = {
      x: a.point.x + (dx / d) * a.radius,
      y: a.point.y + (dy / d) * a.radius
    };
    const res = kf.update(ringPoint, 2.5);
    updatePos = res.pos;
  } else if (validAnchors.length === 2) {
    lockType = 'DUAL';
    const [cand1, cand2] = findTwoCircleIntersections(validAnchors[0].point, validAnchors[0].radius, validAnchors[1].point, validAnchors[1].radius);
    const d1 = distance(cand1, predPos);
    const d2 = distance(cand2, predPos);
    const chosen = d1 <= d2 ? cand1 : cand2;
    const res = kf.update(chosen, 2.0);
    updatePos = res.pos;
  } else if (validAnchors.length >= 3) {
    lockType = '3-ANCHOR';
    const solved = solveGaussNewton(validAnchors.map(a => a.point), validAnchors.map(a => a.radius), predPos);
    const res = kf.update(solved, 1.5);
    updatePos = res.pos;
  }

  console.log(`Step ${step}: tag=(${tag.x.toFixed(0)}, ${tag.y.toFixed(0)}) | predPos=(${predPos.x.toFixed(1)}, ${predPos.y.toFixed(1)}) | valid=[${validAnchors.map(a=>a.id).join(',')}] | zero=[${zeroWeightedAnchors.join(',')}] | lock=${lockType}`);
}



