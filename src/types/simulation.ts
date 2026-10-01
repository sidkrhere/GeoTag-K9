export interface Point2D {
  x: number;
  y: number;
}

export type GeoStopTier = 1 | 2; // Tier 1: Solar Poles, Tier 2: Eateries & Hotspots

export interface GeoStop {
  id: string;
  name: string;
  tier: GeoStopTier;
  x: number;
  y: number;
  isOnline: boolean;
  batteryMv?: number; // For Tier 1 Solar poles
}

export interface GeoCentre {
  id: string;
  name: string;
  x: number;
  y: number;
  isOnline: boolean;
}

export type ImuSensorType = 'ACCEL_ONLY' | 'IMU_6DOF' | 'ECOMPASS_9DOF' | 'GPS_RTK_BENCHMARK';

export interface CompassHeadingData {
  headingDeg: number;         // 0° - 359° absolute azimuth (0° = N, 90° = E, 180° = S, 270° = W)
  cardinal: 'N' | 'NE' | 'E' | 'SE' | 'S' | 'SW' | 'W' | 'NW';
  pitchDeg: number;           // -25° (sniffing) to +15° (alert)
  rollDeg: number;            // -10° to +10°
  yawRateDegPerSec: number;   // Angular turn rate from gyro
  sensorCurrentUa?: number;   // 54 µA active / 3.5 µA sleeping
  projectedBatteryDays?: number; // ~60 days on 600 mAh LiPo
  dorsalVector?: Point2D;
  sensorHealth?: 'CALIBRATED' | 'UNCALIBRATED' | 'INTERFERENCE';
}

export interface GeoTag {
  id: string;
  name: string;
  x: number;
  y: number;
  targetX?: number;
  targetY?: number;
  vx: number;
  vy: number;
  headingDeg?: number;        // Current pointing azimuth [0°, 360°)
  compassData?: CompassHeadingData;
  autoWalk: boolean;
  imuResting: boolean;        // IMU resting state
  speedMps?: number;          // Live physical velocity in m/s
  batteryPct: number;
  color: string;
  pathWaypoints?: Point2D[];
  currentWaypointIndex?: number;
}

export interface BuildingPolygon {
  id: string;
  name: string;
  points: Point2D[];
  closed: boolean;
  attenuationDb: number;
  color?: string;
}

export interface SignalMeasurement {
  stopId: string;
  tagId: string;
  trueDistanceMeters: number;
  noisyDistanceMeters: number;
  rssiDbm: number;
  obstructed: boolean;
  numObstructions: number;
  isOutlier?: boolean;
  weight?: number;
  isNlosZeroWeighted?: boolean;
}

export type LockType =
  | 'FULL_3D_LOCK'
  | 'DUAL_PINPOINT_LOCK'       // Exactly 2 GeoStops + e-Compass vector match!
  | 'DUAL_KINEMATIC_LOCK'
  | 'DUAL_AMBIGUOUS_CHORD'
  | 'DUAL_COLD_BOOT'
  | 'DUAL_AMBIGUITY'
  | 'RADIAL_RING'
  | 'BLIND_ZONE';

export interface BorderExitInfo {
  lastKnownPos: Point2D;
  exitVelocity: Point2D;
  exitHeadingDeg: number;
  exitSpeedMps: number;
  exitTimestamp: number;
  elapsedSeconds: number;
  previousAnchorCount: number;
}

export interface DualCandidateInfo {
  candidate1: Point2D;
  candidate2: Point2D;
  validatedIndex: 0 | 1 | null;
  outcastIndex: 0 | 1 | null;
  outcastLeapMeters: number;
  separationChordMeters: number;
  reason: string;
  candidate1ErrorMeters?: number;
  candidate2ErrorMeters?: number;
  candidate1PredErrorMeters?: number;
  candidate2PredErrorMeters?: number;
  candidate1VectorAlignment?: number; // cos(α) similarity in [-1, +1]
  candidate2VectorAlignment?: number; // cos(α) similarity in [-1, +1]
  compassHeadingDeg?: number;
  transitionDelayMs?: number;
  isTransitionBuffering?: boolean;
  candidate1Score?: number;
  candidate2Score?: number;
  candidate1DistMeters?: number;
  candidate2DistMeters?: number;
  kinematicWeight?: number;
  compassWeight?: number;
}

export interface HistoricalFix {
  x: number;
  y: number;
  timestamp: number;
  vx: number;
  vy: number;
  speedMps: number;
  lockType: LockType;
  errorMeters: number;
  headingDeg?: number;
}

export interface TrilaterationResult {
  tagId: string;
  estimatedX: number;
  estimatedY: number;
  errorMeters: number;
  lockType: LockType;
  contributingAnchorIds: string[];
  outlierAnchorIds?: string[];
  dualCandidates?: [Point2D, Point2D];
  dualInfo?: DualCandidateInfo;
  borderExit?: BorderExitInfo;
  hypothesisFlipDetected?: boolean;
  velocity?: Point2D;
  speedMps?: number;
  actualSpeedMps?: number;
  historyTrail?: HistoricalFix[];
  historicalVelocity?: Point2D;
  covarianceTrace?: number;
  headingDeg?: number;
  timestamp?: number;
  radialRing?: {
    center: Point2D;
    radiusPixels: number;
    radiusMeters: number;
    stopId: string;
  };
  gdop: number;
}

export interface SimulationConfig {
  tier1RadiusMeters: number; // Linked to top-right slider
  tier2RadiusMeters: number; // Linked to top-right slider
  noiseStdDevDb: number;     // Channel noise (dB)
  wallAttenuationDb: number; // Attenuation per building wall
  pathLossExponent: number;  // Log-distance path loss exponent n (usually 2.5 - 3.5 outdoors)
  pixelsPerMeter: number;    // Scale: e.g. 3 pixels = 1 meter
  autoWalkSpeed: number;     // Speed for autonomous wandering dogs
  showRFRipples: boolean;
  showBackhaulBeams: boolean;
  showCoverageHalos: boolean;
  showTrueDoggos: boolean;
  showEstimatedLocations: boolean;
  showHeadingVisionCone: boolean; // Canine forward radar / vision arc
  showHistoricalBreadcrumbs: boolean; // Historical trajectory tail
  imuSensorType: ImuSensorType;   // 'ECOMPASS_9DOF' or 'ACCEL_ONLY'
}

export type ActiveTool =
  | 'SELECT_MOVE'
  | 'DRAW_BUILDING'
  | 'TRACE_PATH'
  | 'ADD_TIER1_STOP'
  | 'ADD_TIER2_STOP'
  | 'ADD_GEOCENTRE';

export interface LapMetrics {
  currentLapNumber: number;
  currentLapAvgError: number;
  lastCompletedLapAvgError: number | null;
  lifetimeAvgError: number;
  currentLapSamples: number;
  totalSamples: number;
}

