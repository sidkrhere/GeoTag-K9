import { getIITRoparPreset } from '../src/engine/presetLayouts';
import { solveTrilateration, resetDogTrajectory, resetAllTrackingState } from '../src/engine/multilateration';
import { calculateSignalMeasurement } from '../src/engine/rfPropagation';
import { SignalMeasurement, SimulationConfig } from '../src/types/simulation';

console.log('=== DIAGNOSING: WHEN IS THE MORE DISTANT CANDIDATE CHOSEN? ===\n');

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

let distantChoices = 0;
let total2AnchorSteps = 0;

for (let step = 0; step < 1200; step++) {
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

  if (res.lockType === 'DUAL_PINPOINT_LOCK' && res.dualInfo) {
    total2AnchorSteps++;
    const chosenDist = res.dualInfo.candidate1DistMeters ?? 0; // chosen distance to expectedForwardPos
    const outcastDist = res.dualInfo.candidate2DistMeters ?? 0; // outcast distance to expectedForwardPos

    const chosenErrToTrue = res.dualInfo.candidate1ErrorMeters ?? 0;
    const outcastErrToTrue = res.dualInfo.candidate2ErrorMeters ?? 0;

    // Check if chosen candidate is MORE DISTANT than the outcast candidate!
    // Case A: Chosen candidate is further from the dog than the outcast
    if (chosenErrToTrue > outcastErrToTrue + 0.5) {
      distantChoices++;
      console.log(`[Step ${step}] MORE DISTANT CANDIDATE CHOSEN RELATIVE TO TRUE DOG:`);
      console.log(`  Chosen Err to Dog: ${chosenErrToTrue}m (Score: ${res.dualInfo.candidate1Score}, Align: ${res.dualInfo.candidate1VectorAlignment}, dPred: ${chosenDist}m)`);
      console.log(`  Outcast Err to Dog: ${outcastErrToTrue}m (Score: ${res.dualInfo.candidate2Score}, Align: ${res.dualInfo.candidate2VectorAlignment}, dPred: ${outcastDist}m)`);
      console.log(`  Reason: ${res.dualInfo.reason}`);
      console.log(`  Tag pos: (${tag.x.toFixed(1)}, ${tag.y.toFixed(1)}), Heading: ${Math.round(tag.headingDeg)} deg`);
      console.log('---');
    }
  }
}

console.log(`\n=== SUMMARY ===`);
console.log(`Total 2-Anchor Steps: ${total2AnchorSteps}`);
console.log(`Instances where more distant candidate was chosen: ${distantChoices}`);
