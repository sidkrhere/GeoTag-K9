import { getIITRoparPreset } from '../src/engine/presetLayouts';
import { calculateSignalMeasurement } from '../src/engine/rfPropagation';
import { CanineKalmanFilter } from '../src/engine/kalmanFilter';
import { Point2D, SignalMeasurement, SimulationConfig, BuildingPolygon, GeoTag, GeoStop } from '../src/types/simulation';

function distance(p1: Point2D, p2: Point2D): number {
  return Math.hypot(p1.x - p2.x, p1.y - p2.y);
}

function findTwoCircleIntersections(p1: Point2D, r1: number, p2: Point2D, r2: number): [Point2D, Point2D] {
  const dx = p2.x - p1.x;
  const dy = p2.y - p1.y;
  const d = Math.hypot(dx, dy);

  if (d > r1 + r2) {
    const scale = r1 / (r1 + r2);
    const midX = p1.x + dx * scale;
    const midY = p1.y + dy * scale;
    return [{ x: midX, y: midY }, { x: midX, y: midY }];
  }

  if (d < Math.abs(r1 - r2)) {
    const rL = Math.max(r1, r2);
    const pL = r1 > r2 ? p1 : p2;
    const pS = r1 > r2 ? p2 : p1;
    const dirX = (pS.x - pL.x) / (d || 1);
    const dirY = (pS.y - pL.y) / (d || 1);
    const pt = { x: pL.x + dirX * rL, y: pL.y + dirY * rL };
    return [pt, pt];
  }

  const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
  const h2 = r1 * r1 - a * a;
  const h = Math.sqrt(Math.max(0, h2));

  const p2x = p1.x + (dx * a) / d;
  const p2y = p1.y + (dy * a) / d;

  return [
    { x: p2x + (h * dy) / d, y: p2y - (h * dx) / d },
    { x: p2x - (h * dy) / d, y: p2y + (h * dx) / d }
  ];
}

function runSimulation(mode: 'CURRENT' | 'QUADRATIC_MAP' | 'HIERARCHICAL_GATE') {
  const width = 1200;
  const height = 800;
  const preset = getIITRoparPreset(width, height);
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
  const waypoints = tag.pathWaypoints || [];
  let wpIdx = tag.currentWaypointIndex || 0;
  const stopsMap = new Map(preset.stops.map(s => [s.id, s]));

  const kf = new CanineKalmanFilter({ x: tag.x, y: tag.y }, config.pixelsPerMeter, 16.0);
  let priorPt: Point2D = { x: tag.x, y: tag.y };
  let priorTimestamp = 1700000000000;

  let total2AnchorSteps = 0;
  let distantChoices = 0;
  let maxErrorMeters = 0;
  let totalErrorMeters = 0;

  for (let step = 0; step < 1200; step++) {
    const targetWp = waypoints[wpIdx];
    if (targetWp) {
      const dx = targetWp.x - tag.x;
      const dy = targetWp.y - tag.y;
      const distToWp = Math.hypot(dx, dy);
      const stepDist = (config.autoWalkSpeed * config.pixelsPerMeter) * 0.050;
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

    const headingDeg = tag.headingDeg ?? 90;
    const headingRad = (headingDeg * Math.PI) / 180;
    const now = 1700000000000 + step * 50;

    const uCompass: Point2D = {
      x: Math.sin(headingRad),
      y: -Math.cos(headingRad)
    };

    const measurements: SignalMeasurement[] = [];
    for (const stop of preset.stops) {
      const sig = calculateSignalMeasurement(tag, stop, preset.buildings, config);
      if (sig) measurements.push(sig);
    }

    const validAnchors = measurements
      .filter(m => !m.isNlosZeroWeighted)
      .map(m => {
        const s = stopsMap.get(m.stopId)!;
        return { id: s.id, point: { x: s.x, y: s.y }, radius: m.noisyDistanceMeters * config.pixelsPerMeter, measurement: m };
      });

    kf.predict(now, headingRad);

    if (validAnchors.length === 2) {
      total2AnchorSteps++;
      const [a1, a2] = validAnchors;
      const [cand1, cand2] = findTwoCircleIntersections(a1.point, a1.radius, a2.point, a2.radius);
      const separationChordMeters = Math.round((distance(cand1, cand2) / config.pixelsPerMeter) * 10) / 10;

      const rawDt = (now - priorTimestamp) / 1000.0;
      const dtSec = Math.max(0.02, Math.min(0.20, rawDt));
      const canineSpeedPix = (config.autoWalkSpeed * config.pixelsPerMeter) * dtSec;
      const expectedForwardPos: Point2D = {
        x: priorPt.x + uCompass.x * canineSpeedPix,
        y: priorPt.y + uCompass.y * canineSpeedPix
      };

      const evalCandidate = (cand: Point2D) => {
        const dxPred = (cand.x - expectedForwardPos.x) / config.pixelsPerMeter;
        const dyPred = (cand.y - expectedForwardPos.y) / config.pixelsPerMeter;
        const distPredM = Math.hypot(dxPred, dyPred);

        let distScore: number;
        if (mode === 'CURRENT') {
          distScore = 4.5 * distPredM;
          if (distPredM > 2.5) distScore += 8.0 * Math.pow(distPredM - 2.5, 1.5);
        } else if (mode === 'QUADRATIC_MAP') {
          // Quadratic MAP: soft at small noise, astronomical for multi-meter leaps
          distScore = 3.0 * distPredM + 2.0 * (distPredM * distPredM);
          if (distPredM > 2.5) distScore += 8.0 * Math.pow(distPredM - 2.5, 1.5);
        } else {
          // In hierarchical gate, base score used for tie-breaker
          distScore = 4.5 * distPredM;
        }

        const dxStep = (cand.x - priorPt.x) / config.pixelsPerMeter;
        const dyStep = (cand.y - priorPt.y) / config.pixelsPerMeter;
        const stepDistM = Math.hypot(dxStep, dyStep);
        let align = 1.0;
        let angleScore = 0.0;
        if (stepDistM > 0.15) {
          align = (dxStep * uCompass.x + dyStep * uCompass.y) / stepDistM;
          const boundedCos = Math.max(-1.0, Math.min(1.0, align));
          angleScore = 25.0 * (1.0 - boundedCos);
          if (stepDistM > 2.0 && boundedCos < -0.2) {
            angleScore += 60.0 * Math.abs(boundedCos);
          }
        }

        return { score: distScore + angleScore, align, distM: distPredM };
      };

      const res1 = evalCandidate(cand1);
      const res2 = evalCandidate(cand2);

      let validatedIdx: 0 | 1;
      if (mode === 'HIERARCHICAL_GATE') {
        if (res1.distM < res2.distM - 1.5) {
          validatedIdx = 0;
        } else if (res2.distM < res1.distM - 1.5) {
          validatedIdx = 1;
        } else {
          validatedIdx = res1.score <= res2.score ? 0 : 1;
        }
      } else {
        validatedIdx = res1.score <= res2.score ? 0 : 1;
      }

      const chosen = validatedIdx === 0 ? cand1 : cand2;
      const outcast = validatedIdx === 0 ? cand2 : cand1;

      const chosenErr = distance(chosen, { x: tag.x, y: tag.y }) / config.pixelsPerMeter;
      const outcastErr = distance(outcast, { x: tag.x, y: tag.y }) / config.pixelsPerMeter;

      if (chosenErr > outcastErr + 0.5) {
        distantChoices++;
      }

      const kfRes = kf.update(chosen, 2.0);
      priorPt = { ...kfRes.pos };
      priorTimestamp = now;

      const err = distance(kfRes.pos, { x: tag.x, y: tag.y }) / config.pixelsPerMeter;
      totalErrorMeters += err;
      if (err > maxErrorMeters) maxErrorMeters = err;
    }
  }

  return {
    mode,
    total2AnchorSteps,
    distantChoices,
    avgErrorMeters: (totalErrorMeters / total2AnchorSteps).toFixed(2),
    maxErrorMeters: maxErrorMeters.toFixed(2)
  };
}

console.log('Result Current:', runSimulation('CURRENT'));
console.log('Result Quadratic MAP:', runSimulation('QUADRATIC_MAP'));
console.log('Result Hierarchical Gate:', runSimulation('HIERARCHICAL_GATE'));
