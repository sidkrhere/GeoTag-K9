import { getIITRoparPreset } from '../src/engine/presetLayouts';
import { solveTrilateration, resetDogTrajectory, resetAllTrackingState } from '../src/engine/multilateration';
import { calculateSignalMeasurement } from '../src/engine/rfPropagation';
import { SignalMeasurement, SimulationConfig, DisambiguationMode } from '../src/types/simulation';

console.log('========================================================================');
console.log('       GEO-TAG K9: COMPREHENSIVE DISAMBIGUATION ENGINE AUDIT            ');
console.log('========================================================================\n');

function auditMode(mode: DisambiguationMode, totalSteps: number = 600) {
  console.log(`>>> AUDITING MODE: [${mode}] (${totalSteps} steps) <<<`);

  const width = 1200;
  const height = 800;
  const preset = getIITRoparPreset(width, height);
  resetAllTrackingState();

  const config: SimulationConfig = {
    anchorRadiusMeters: 90,
    tier1RadiusMeters: 90,
    tier2RadiusMeters: 90,
    noiseStdDevDb: 0.5,
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
    imuSensorType: 'ECOMPASS_9DOF',
    disambiguationMode: mode
  };

  const tag = { ...preset.tags[0] };
  resetDogTrajectory(tag.id, { x: tag.x, y: tag.y });
  const waypoints = tag.pathWaypoints || [];
  let wpIdx = tag.currentWaypointIndex || 0;
  const stopsMap = new Map(preset.stops.map(s => [s.id, s]));

  let total2AnchorSteps = 0;
  let distantChoices = 0;
  let maxJumpMeters = 0;
  let jumpsOver5m = 0;
  let totalErrorMeters = 0;
  let prevEst = { x: tag.x, y: tag.y };

  for (let step = 0; step < totalSteps; step++) {
    // Canine path waypoint progression
    const targetWp = waypoints[wpIdx];
    if (targetWp) {
      const dx = targetWp.x - tag.x;
      const dy = targetWp.y - tag.y;
      const distToWp = Math.hypot(dx, dy);
      const stepDist = (config.autoWalkSpeed * config.pixelsPerMeter) * 0.050; // 50ms interval

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

    // Measure RF signals
    const measurements: SignalMeasurement[] = [];
    for (const stop of preset.stops) {
      const sig = calculateSignalMeasurement(tag, stop, preset.buildings, config);
      if (sig) measurements.push(sig);
    }

    const nowMs = Date.now() + step * 50;
    const res = solveTrilateration(tag, measurements, stopsMap, config, preset.buildings, nowMs);

    const jumpM = Math.hypot(res.estimatedX - prevEst.x, res.estimatedY - prevEst.y) / config.pixelsPerMeter;
    if (step > 0 && jumpM > maxJumpMeters) maxJumpMeters = jumpM;
    if (step > 0 && jumpM > 5.0) {
      jumpsOver5m++;
      console.log(`    [JUMP > 5m] Step ${step}: jump=${jumpM.toFixed(2)}m, lock=${res.lockType}, anchors=${res.contributingAnchorIds.length}`);
    }

    totalErrorMeters += res.errorMeters;
    prevEst = { x: res.estimatedX, y: res.estimatedY };

    if (res.lockType === 'DUAL_PINPOINT_LOCK' && res.dualInfo) {
      total2AnchorSteps++;
      const chosenErr = res.dualInfo.candidate1ErrorMeters ?? 0;
      const outcastErr = res.dualInfo.candidate2ErrorMeters ?? 0;

      // Flag if outcast was closer by > 1.5m
      if (chosenErr > outcastErr + 1.5) {
        distantChoices++;
        if (distantChoices <= 5) {
          console.log(`    [ANOMALY] Step ${step}: chosenErr=${chosenErr}m, outcastErr=${outcastErr}m, reason=${res.dualInfo?.reason}`);
        }
      }
    }
  }

  const avgErr = totalErrorMeters / totalSteps;
  console.log(`  Total Steps: ${totalSteps}`);
  console.log(`  Total 2-Anchor Steps: ${total2AnchorSteps}`);
  console.log(`  Distant Candidate Anomalies (>1.5m closer outcast): ${distantChoices}`);
  console.log(`  Average Tracking Error: ${avgErr.toFixed(2)} m`);
  console.log(`  Max Step Jump: ${maxJumpMeters.toFixed(2)} m`);
  console.log(`  Jumps > 5.0m: ${jumpsOver5m}`);
  console.log(`  Audit Result: ${distantChoices === 0 ? 'PASSED' : 'FAILED'}\n`);

  return {
    mode: `${mode} (${totalSteps} steps)`,
    totalSteps,
    total2AnchorSteps,
    distantChoices,
    avgError: avgErr.toFixed(2),
    maxJump: maxJumpMeters.toFixed(2),
    jumpsOver5m,
    result: distantChoices === 0 ? 'PASSED' : 'FAILED'
  };
}

const auditA600 = auditMode('QUADRATIC_MAP', 600);
const auditB600 = auditMode('HIERARCHICAL_GATE', 600);
const auditA1200 = auditMode('QUADRATIC_MAP', 1200);
const auditB1200 = auditMode('HIERARCHICAL_GATE', 1200);

console.log('========================================================================');
console.log('SUMMARY COMPARISON TABLE:');
console.table([auditA600, auditB600, auditA1200, auditB1200]);
console.log('========================================================================');
