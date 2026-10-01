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

function countBuildingWallIntersections(tag, stop, b) {
  let intersections = 0;
  const pts = b.points;
  for (let i = 0; i < pts.length; i++) {
    const nextIdx = (i + 1) % pts.length;
    if (lineSegmentsIntersect(tag, stop, pts[i], pts[nextIdx])) {
      intersections++;
    }
  }
  return intersections;
}

class CanineKalmanFilter {
  constructor(initialPos, pixelsPerMeter = 2.8, maxSpeedMps = 8.5) {
    this.pixelsPerMeter = pixelsPerMeter;
    this.maxSpeedMps = maxSpeedMps;
    this.state = [initialPos.x, initialPos.y, 0, 0];
    this.lastUpdateMs = Date.now();
    this.P = [
      [4.0, 0.0, 0.0, 0.0],
      [0.0, 4.0, 0.0, 0.0],
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
        const guidedVx = this.state[2] * 0.75 + (uCompassX * currentSpeed) * 0.25;
        const guidedVy = this.state[3] * 0.75 + (uCompassY * currentSpeed) * 0.25;
        this.state[2] = guidedVx;
        this.state[3] = guidedVy;
      }
    }

    this.state[0] += this.state[2] * dt;
    this.state[1] += this.state[3] * dt;
    this.clampVelocity();

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

    const dM2 = (s11 * yX * yX - 2.0 * s01 * yX * yY + s00 * yY * yY) / detS;

    if (dM2 > 14.0) {
      return { pos: { x: this.state[0], y: this.state[1] }, wasRejected: true };
    }

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
    return { pos: { x: this.state[0], y: this.state[1] }, wasRejected: false };
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

const cx = 600;
const cy = 400;

const building = {
  closed: true,
  points: [
    { x: cx - 130, y: cy - 85 },
    { x: cx + 130, y: cy - 85 },
    { x: cx + 130, y: cy + 85 },
    { x: cx - 130, y: cy + 85 }
  ]
};

const stops = [
  { id: 'stop-north', name: 'North Plaza Mast #1', x: cx, y: cy - 135 },
  { id: 'stop-east', name: 'East Veranda Anchor #2', x: cx + 190, y: cy },
  { id: 'stop-south', name: 'South Promenade Mast #3', x: cx, y: cy + 135 },
  { id: 'stop-west', name: 'West Garden Anchor #4', x: cx - 190, y: cy }
];

// Let's test the simulation
let now = Date.now();
const kf = new CanineKalmanFilter({ x: 435, y: 280 }, 2.8, 8.5);

let tagPos = { x: 435, y: 280 };

console.log('--- STARTING WALKING SIMULATION ---');
for (let frame = 1; frame <= 100; frame++) {
  now += 80;
  if (frame <= 50) {
    tagPos.x += (765 - 435) / 50;
  } else {
    tagPos.y += (400 - 280) / 50;
  }

  const headingRad = frame <= 50 ? Math.PI / 2 : Math.PI;
  const predPos = kf.predict(now, headingRad);

  const stopEast = stops[1];
  const wallsToEast = countBuildingWallIntersections(predPos, stopEast, building);
  const dPred = distance(predPos, tagPos);

  if (frame % 20 === 0 || wallsToEast > 0) {
    console.log(`Frame ${frame}: tagPos=(${tagPos.x.toFixed(1)}, ${tagPos.y.toFixed(1)}), predPos=(${predPos.x.toFixed(1)}, ${predPos.y.toFixed(1)}), dPred=${dPred.toFixed(1)}px, wallsToEast=${wallsToEast}`);
  }

  // Suppose multilateration solve produces a measurement:
  const meas = { x: tagPos.x + (Math.random() - 0.5) * 2, y: tagPos.y + (Math.random() - 0.5) * 2 };
  const res = kf.update(meas, 2.0);
  if (res.wasRejected) {
    console.log(`  --> Frame ${frame}: REJECTED! tagPos=(${tagPos.x.toFixed(1)}, ${tagPos.y.toFixed(1)}) vs kfState=(${kf.state[0].toFixed(1)}, ${kf.state[1].toFixed(1)})`);
  }
}
