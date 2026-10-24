import { getIITRoparPreset } from '../src/engine/presetLayouts';
import { solveTrilateration, resetDogTrajectory, resetAllTrackingState } from '../src/engine/multilateration';
import { calculateSignalMeasurement } from '../src/engine/rfPropagation';
import { SignalMeasurement, SimulationConfig } from '../src/types/simulation';

console.log('=== RUNNING REAL PRESET TEST WITH NEW DISAMBIGUATION LOGIC (600 STEPS) ===');

const width = 1200;
const height = 800;
const preset = getIITRoparPreset(width, height);
resetAllTrackingState();

const config: SimulationConfig = {
  tier1RadiusMeters: 90,
  tier2RadiusMeters: 45,
  noiseStdDevDb: 3.0,
  wallAttenuationDb: 12,
  pathLossExponent: 3.0,
  pixelsPerMeter: 2.8,
  autoWalkSpeed: 2.5,
  showRFRipples: true,
  showBackhaulBeams: true,
  showCoverageHalos: true,
  showTrueDoggos: true,
  showEstimatedLocations: true,
  showHeadingVisionCone: true,
  showHistoricalBreadcrumbs: true,
  imuSensorType: 'ECOMPASS_9DOF'
};

const tag = { ...preset.tags[0] };
resetDogTrajectory(tag.id, { x: tag.x, y: tag.y });
const waypoints = tag.pathWaypoints || [];
let wpIdx = tag.currentWaypointIndex || 0;

const stopsMap = new Map(preset.stops.map(s => [s.id, s]));

let totalErrorMeters = 0;
let maxJumpMeters = 0;
let jumpsOver10m = 0;
let prevEst = { x: tag.x, y: tag.y };
let totalSteps = 600;

for (let step = 0; step < totalSteps; step++) {
  // Move dog along waypoint path
  const targetWp = waypoints[wpIdx];
  if (targetWp) {
    const dx = targetWp.x - tag.x;
    const dy = targetWp.y - tag.y;
    const distToWp = Math.hypot(dx, dy);
    const stepDist = (config.autoWalkSpeed * config.pixelsPerMeter) * 0.050; // 50ms step

    if (distToWp < stepDist + 2) {
      wpIdx = (wpIdx + 1) % waypoints.length;
    } else {
      const vx = (dx / distToWp) * stepDist;
      const vy = (dy / distToWp) * stepDist;
      tag.x += vx;
      tag.y += vy;
      tag.vx = vx;
      tag.vy = vy;
      tag.headingDeg = (Math.atan2(vx, -vy) * 180 / Math.PI + 360) % 360;
    }
  }

  // RF measurements
  const measurements: SignalMeasurement[] = [];
  for (const stop of preset.stops) {
    const sig = calculateSignalMeasurement(tag, stop, preset.buildings, config);
    if (sig) measurements.push(sig);
  }

  const nowMs = 1700000000000 + step * 50;
  const res = solveTrilateration(tag, measurements, stopsMap, config, preset.buildings, nowMs);

  const jumpM = Math.hypot(res.estimatedX - prevEst.x, res.estimatedY - prevEst.y) / config.pixelsPerMeter;
  if (step > 0 && jumpM > maxJumpMeters) {
    maxJumpMeters = jumpM;
  }
  if (step > 0 && jumpM > 10.0) {
    jumpsOver10m++;
    console.log(`Big jump at step ${step}: ${jumpM.toFixed(1)}m | Lock=${res.lockType} | Err=${res.errorMeters}m`);
    if (res.dualInfo) {
      console.log(`  Reason: ${res.dualInfo.reason}`);
    }
  }

  totalErrorMeters += res.errorMeters;
  prevEst = { x: res.estimatedX, y: res.estimatedY };
}

const avgErr = totalErrorMeters / totalSteps;
console.log('\n=== REAL PRESET TEST RESULTS ===');
console.log(`Total Steps: ${totalSteps}`);
console.log(`Average Error: ${avgErr.toFixed(2)} meters`);
console.log(`Max Step Jump: ${maxJumpMeters.toFixed(2)} meters`);
console.log(`Jumps > 10m: ${jumpsOver10m}`);
if (jumpsOver10m === 0) {
  console.log('>>> SUCCESS: 0 JUMPS OVER 10M ACROSS ALL 600 STEPS! <<<');
} else {
  console.log(`>>> FAILED: ${jumpsOver10m} big jumps <<<`);
}
