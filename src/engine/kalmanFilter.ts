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
  private initialized: boolean = false;
  private consecutiveRejections: number = 0;

  constructor(initialPos: Point2D, pixelsPerMeter: number = 2.8, maxSpeedMps: number = 45.0) {
    this.pixelsPerMeter = pixelsPerMeter;
    this.maxSpeedMps = maxSpeedMps;
    this.state = [initialPos.x, initialPos.y, 0, 0];
    this.lastUpdateMs = Date.now();
    this.consecutiveRejections = 0;

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
   * Sets dynamic velocity bounding limit in m/s scaled to simulation speed.
   */
  public setMaxSpeedMps(speedMps: number): void {
    if (speedMps > 0) {
      this.maxSpeedMps = speedMps;
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
    this.P = [
      [16.0, 0.0, 0.0, 0.0],
      [0.0, 16.0, 0.0, 0.0],
      [0.0, 0.0, 25.0, 0.0],
      [0.0, 0.0, 0.0, 25.0]
    ];
  }

  /**
   * Kinematic state & covariance prediction step: x_{k|k-1} = F * x_{k-1|k-1}
   * Enforces the canine dynamic velocity bound: ||v|| <= maxSpeedMps
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
        // Blend 25% of heading vector into velocity orientation
        const guidedVx = this.state[2] * 0.75 + (uCompassX * currentSpeed) * 0.25;
        const guidedVy = this.state[3] * 0.75 + (uCompassY * currentSpeed) * 0.25;
        this.state[2] = guidedVx;
        this.state[3] = guidedVy;
      }
    }

    // 1. Propagate State: x = x + vx * dt, y = y + vy * dt
    this.state[0] += this.state[2] * dt;
    this.state[1] += this.state[3] * dt;

    // 2. Dynamic Velocity Clamping (Canine physiological bound)
    this.clampVelocity();

    // 3. Propagate Covariance: P = F * P * F^T + Q
    // Continuous white-noise acceleration model scaled with dynamic speed cap
    const maxAccelMps2 = Math.max(12.0, this.maxSpeedMps * 0.6);
    const q = maxAccelMps2 * this.pixelsPerMeter * this.pixelsPerMeter;

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
   * Uses Mahalanobis distance gating to reject wall-occluded phantom range jumps,
   * while adaptively recovering if consecutive rejections indicate filter divergence.
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
    const dt = Math.max(0.01, Math.min(0.25, (now - this.lastUpdateMs) / 1000.0));
    const maxDisplacementPix = this.maxSpeedMps * this.pixelsPerMeter * dt;

    // Chi-Square gating: threshold 50.0 gives robust margin against high simulation speeds & sudden turns
    if (dM2 > 50.0) {
      this.consecutiveRejections++;
      if (this.consecutiveRejections >= 2) {
        // Fast recovery: catch up swiftly towards the measurement to close spatial lag
        const dist = Math.hypot(yX, yY) || 1;
        const catchupStep = Math.min(dist, Math.max(maxDisplacementPix * 3.0, dist * 0.65));
        
        this.state[0] += (yX / dist) * catchupStep;
        this.state[1] += (yY / dist) * catchupStep;
        this.state[2] = (yX / dist) * Math.min(this.maxSpeedMps * this.pixelsPerMeter, (catchupStep / dt));
        this.state[3] = (yY / dist) * Math.min(this.maxSpeedMps * this.pixelsPerMeter, (catchupStep / dt));
        this.clampVelocity();

        // Inflate covariance to reflect transition uncertainty
        this.P[0][0] = Math.min(100.0, this.P[0][0] * 1.2);
        this.P[1][1] = Math.min(100.0, this.P[1][1] * 1.2);

        this.lastUpdateMs = now;
        return this.getResult(false);
      }
      // Coast on prediction for single isolated outlier spikes
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

    // State update with Dynamic Velocity Position Clamping:
    const deltaX = k00 * yX + k01 * yY;
    const deltaY = k10 * yX + k11 * yY;
    const deltaDist = Math.hypot(deltaX, deltaY);
    const allowedDisplacement = maxDisplacementPix * 1.6;

    if (deltaDist > allowedDisplacement && deltaDist > 1e-4) {
      const scale = allowedDisplacement / deltaDist;
      this.state[0] += deltaX * scale;
      this.state[1] += deltaY * scale;
    } else {
      this.state[0] += deltaX;
      this.state[1] += deltaY;
    }

    this.state[2] += k20 * yX + k21 * yY;
    this.state[3] += k30 * yX + k31 * yY;

    // Enforce dynamic canine velocity clamping on posterior state
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
   * Strictly clamps velocity magnitude to the canine maximum physical limit.
   */
  private clampVelocity(): void {
    const maxPixPerSec = this.maxSpeedMps * this.pixelsPerMeter;
    const currentSpeedPix = Math.sqrt(this.state[2] * this.state[2] + this.state[3] * this.state[3]);

    if (currentSpeedPix > maxPixPerSec && currentSpeedPix > 1e-4) {
      const ratio = maxPixPerSec / currentSpeedPix;
      this.state[2] *= ratio;
      this.state[3] *= ratio;
    }
  }

  private getResult(wasRejected: boolean) {
    const speedPix = Math.sqrt(this.state[2] * this.state[2] + this.state[3] * this.state[3]);
    const speedMps = Math.round((speedPix / this.pixelsPerMeter) * 10) / 10;

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
