import { Point2D } from '../types/simulation';

/**
 * 4-State Dynamic Velocity-Bounded Kalman Filter (DVB-KF) for Canine Tracking.
 * State vector: x = [px, py, vx, vy]^T
 * Enforces canine biomechanical velocity bounds (v <= 8.5 m/s) and rejects
 * unphysical wall-induced jumps via statistical Mahalanobis innovation gating.
 */
export class CanineKalmanFilter {
  // State: [px, py, vx, vy] (positions in pixels, velocities in pixels/sec)
  private state: [number, number, number, number];

  // 4x4 State Covariance Matrix P
  private P: number[][];

  private lastUpdateMs: number;
  private pixelsPerMeter: number;
  private maxSpeedMps: number;
  private smoothedSpeedMps: number = 0;
  private initialized: boolean = false;
  private consecutiveRejections: number = 0;

  constructor(initialPos: Point2D, pixelsPerMeter: number = 2.8, maxSpeedMps: number = 15.0) {
    this.pixelsPerMeter = pixelsPerMeter;
    // Generous canine kinematic ceiling (default 15.0 m/s = 54 km/h)
    this.maxSpeedMps = Math.max(2.0, maxSpeedMps);
    this.state = [initialPos.x, initialPos.y, 0, 0];
    this.lastUpdateMs = Date.now();
    this.consecutiveRejections = 0;
    this.smoothedSpeedMps = 0;

    // Initial covariance: Moderate certainty in position, moderate uncertainty in velocity
    this.P = [
      [16.0, 0.0, 0.0, 0.0],
      [0.0, 16.0, 0.0, 0.0],
      [0.0, 0.0, 25.0, 0.0],
      [0.0, 0.0, 0.0, 25.0]
    ];
    this.initialized = true;
  }

  /**
   * Sets dynamic velocity bounding limit in m/s scaled to canine gait.
   */
  public setMaxSpeedMps(speedMps: number): void {
    if (speedMps > 0) {
      this.maxSpeedMps = Math.max(2.0, speedMps);
    }
  }

  public getMaxSpeedMps(): number {
    return this.maxSpeedMps;
  }

  /**
   * Resets filter state (e.g. after manual drag or teleport).
   */
  public reset(pos: Point2D): void {
    this.state = [pos.x, pos.y, 0, 0];
    this.lastUpdateMs = Date.now();
    this.consecutiveRejections = 0;
    this.smoothedSpeedMps = 0;
    this.P = [
      [16.0, 0.0, 0.0, 0.0],
      [0.0, 16.0, 0.0, 0.0],
      [0.0, 0.0, 25.0, 0.0],
      [0.0, 0.0, 0.0, 25.0]
    ];
  }

  /**
   * Kinematic state & covariance prediction step: x_{k|k-1} = F * x_{k-1|k-1}
   * Predicts future position and incorporates dynamic continuous acceleration noise.
   */
  public predict(nowMs: number, headingRad?: number): Point2D {
    if (!this.initialized) return { x: this.state[0], y: this.state[1] };

    const rawDt = (nowMs - this.lastUpdateMs) / 1000.0;
    // Bound dt between 10ms and 250ms to prevent numerical instability during frame drops
    const dt = Math.max(0.01, Math.min(0.25, rawDt));

    // Optional heading guidance: If heading is provided and moving, bias velocity toward collar spine
    if (headingRad !== undefined) {
      const uCompassX = Math.sin(headingRad);
      const uCompassY = -Math.cos(headingRad);
      const currentSpeed = Math.sqrt(this.state[2] * this.state[2] + this.state[3] * this.state[3]);

      if (currentSpeed > 0.5 * this.pixelsPerMeter) {
        // Blend 20% of heading vector into velocity orientation
        const guidedVx = this.state[2] * 0.80 + (uCompassX * currentSpeed) * 0.20;
        const guidedVy = this.state[3] * 0.80 + (uCompassY * currentSpeed) * 0.20;
        this.state[2] = guidedVx;
        this.state[3] = guidedVy;
      }
    }

    // 1. Propagate State: x = x + vx * dt, y = y + vy * dt
    this.state[0] += this.state[2] * dt;
    this.state[1] += this.state[3] * dt;

    // 2. Soft velocity headroom bound (prevents runaway numerical divergence while allowing agile catchup)
    this.clampVelocity();

    // 3. Propagate Covariance: P = F * P * F^T + Q
    // Continuous adaptive acceleration model: dynamically scales Q with speed and acceleration demand
    const currentSpeedMps = Math.hypot(this.state[2], this.state[3]) / this.pixelsPerMeter;
    const dynamicMaxAccel = Math.max(25.0, Math.max(this.maxSpeedMps, currentSpeedMps) * 2.5);
    const q = dynamicMaxAccel * this.pixelsPerMeter * this.pixelsPerMeter;

    const dt2 = dt * dt;
    const dt3 = dt2 * dt;
    const q11 = (q * dt3) / 3.0;
    const q12 = (q * dt2) / 2.0;
    const q22 = q * dt;

    // P_new = F * P * F^T + Q
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

  /**
   * Bayesian measurement update step: x_{k|k} = x_{k|k-1} + K * (z - H * x_{k|k-1})
   * Eliminates artificial position displacement clamping so the filter locks onto
   * true position with zero spatial lag during acceleration.
   */
  public update(
    meas: Point2D,
    measStdDevMeters: number = 2.0
  ): { pos: Point2D; velocity: Point2D; speedMps: number; wasRejected: boolean } {
    const rPix = measStdDevMeters * this.pixelsPerMeter;
    const rVar = rPix * rPix;

    // Innovation residual: y = z - H * x
    const yX = meas.x - this.state[0];
    const yY = meas.y - this.state[1];

    // Innovation covariance: S = H * P * H^T + R
    const s00 = this.P[0][0] + rVar;
    const s11 = this.P[1][1] + rVar;
    const s01 = this.P[0][1];
    const detS = s00 * s11 - s01 * s01;

    if (detS < 1e-6) {
      return this.getResult(false);
    }

    // Mahalanobis distance squared: d_M^2 = y^T * S^-1 * y
    const dM2 = (s11 * yX * yX - 2.0 * s01 * yX * yY + s00 * yY * yY) / detS;

    const now = Date.now();

    // Chi-Square gating: threshold 60.0 allows swift agility during canine sprints while rejecting wall spikes
    if (dM2 > 60.0) {
      this.consecutiveRejections++;
      if (this.consecutiveRejections >= 8) {
        // Prolonged dropout recovery (>400ms): gently realign position, but NEVER inject unphysical velocity spikes
        this.state[0] = meas.x;
        this.state[1] = meas.y;
        this.state[2] = 0;
        this.state[3] = 0;

        // Inflate covariance to reflect transition uncertainty
        this.P[0][0] = Math.min(100.0, this.P[0][0] * 1.5);
        this.P[1][1] = Math.min(100.0, this.P[1][1] * 1.5);

        this.lastUpdateMs = now;
        this.consecutiveRejections = 0;
        return this.getResult(false);
      }
      // Smoothly coast on kinematic prediction for outlier noise spikes
      return this.getResult(true);
    }

    this.consecutiveRejections = 0;

    // Kalman Gain: K = P * H^T * S^-1 (4x2 matrix)
    const invS00 = s11 / detS;
    const invS11 = s00 / detS;
    const invS01 = -s01 / detS;

    // K rows:
    const k00 = this.P[0][0] * invS00 + this.P[0][1] * invS01;
    const k01 = this.P[0][0] * invS01 + this.P[0][1] * invS11;

    const k10 = this.P[1][0] * invS00 + this.P[1][1] * invS01;
    const k11 = this.P[1][0] * invS01 + this.P[1][1] * invS11;

    const k20 = this.P[2][0] * invS00 + this.P[2][1] * invS01;
    const k21 = this.P[2][0] * invS01 + this.P[2][1] * invS11;

    const k30 = this.P[3][0] * invS00 + this.P[3][1] * invS01;
    const k31 = this.P[3][0] * invS01 + this.P[3][1] * invS11;

    // Direct, optimal Bayesian state update with ZERO artificial position clamping:
    this.state[0] += k00 * yX + k01 * yY;
    this.state[1] += k10 * yX + k11 * yY;
    this.state[2] += k20 * yX + k21 * yY;
    this.state[3] += k30 * yX + k31 * yY;

    // Apply soft velocity headroom bound
    this.clampVelocity();
    this.lastUpdateMs = now;

    // Joseph-form or standard covariance update: P = (I - K * H) * P
    const p00Old = this.P[0][0];
    const p11Old = this.P[1][1];
    const p22Old = this.P[2][2];
    const p33Old = this.P[3][3];

    this.P[0][0] = Math.max(0.1, p00Old - (k00 * p00Old + k01 * this.P[1][0]));
    this.P[1][1] = Math.max(0.1, p11Old - (k10 * this.P[0][1] + k11 * p11Old));
    this.P[2][2] = Math.max(0.5, p22Old - (k20 * this.P[0][2] + k21 * this.P[1][2]));
    this.P[3][3] = Math.max(0.5, p33Old - (k30 * this.P[0][3] + k31 * this.P[1][3]));

    return this.getResult(false);
  }

  /**
   * Soft-bounds velocity magnitude with generous catch-up headroom (up to 18.0 m/s),
   * ensuring the filter never starves position convergence or drags behind during agile canine sprints.
   */
  private clampVelocity(): void {
    const maxCatchUpMps = Math.max(18.0, this.maxSpeedMps * 2.0);
    const maxPixPerSec = maxCatchUpMps * this.pixelsPerMeter;
    const currentSpeedPix = Math.sqrt(this.state[2] * this.state[2] + this.state[3] * this.state[3]);

    if (currentSpeedPix > maxPixPerSec && currentSpeedPix > 1e-4) {
      const ratio = maxPixPerSec / currentSpeedPix;
      this.state[2] *= ratio;
      this.state[3] *= ratio;
    }
  }

  private getResult(wasRejected: boolean) {
    const rawSpeedPix = Math.hypot(this.state[2], this.state[3]);
    const rawSpeedMps = rawSpeedPix / this.pixelsPerMeter;

    // Smooth reported telemetry speed with EMA to prevent digit flicker:
    this.smoothedSpeedMps = this.smoothedSpeedMps === 0
      ? rawSpeedMps
      : this.smoothedSpeedMps * 0.70 + rawSpeedMps * 0.30;

    const speedMps = Math.round(this.smoothedSpeedMps * 10) / 10;

    return {
      pos: { x: this.state[0], y: this.state[1] },
      velocity: { x: this.state[2], y: this.state[3] },
      speedMps,
      wasRejected
    };
  }

  public getState() {
    return this.getResult(false);
  }

  public getCovarianceTrace(): number {
    return Math.sqrt(this.P[0][0] + this.P[1][1]) / this.pixelsPerMeter;
  }
}
