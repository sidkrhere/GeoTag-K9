import { getIITRoparPreset } from '../src/engine/presetLayouts';
import { calculateSignalMeasurement } from '../src/engine/rfPropagation';
import { findTwoCircleIntersections } from '../src/engine/multilateration';
import { CanineKalmanFilter } from '../src/engine/kalmanFilter';
import { Point2D } from '../src/types/simulation';

function distance(p1: Point2D, p2: Point2D): number {
  return Math.hypot(p1.x - p2.x, p1.y - p2.y);
}

const preset = getIITRoparPreset(1200, 800);
const config = {
  tier1RadiusMeters: 90,
  tier2RadiusMeters: 45,
  noiseStdDevDb: 3.0,
  wallAttenuationDb: 12,
  pathLossExponent: 3.0,
  pixelsPerMeter: 2.8,
  autoWalkSpeed: 2.5
};

const tag = { ...preset.tags[0] };
const waypoints = tag.pathWaypoints || [];
let wpIdx = 0;
const stopsMap = new Map(preset.stops.map(s => [s.id, s]));

let b1: Point2D | null = null;
let b2: Point2D | null = null;

let total2ASteps = 0;
let fartherCandidateChosenCount = 0;
let maxErrMeters = 0;
let totalErrMeters = 0;

for (let step = 0; step < 1200; step++) {
  const targetWp = waypoints[wpIdx];
  if (targetWp) {
    const dx = targetWp.x - tag.x;
    const dy = targetWp.y - tag.y;
    const distToWp = Math.hypot(dx, dy);
    const stepDist = (config.autoWalkSpeed * config.pixelsPerMeter) * 0.050;
    if (distToWp < stepDist + 2) wpIdx = (wpIdx + 1) % waypoints.length;
    else {
      const vx = (dx / distToWp) * stepDist;
      const vy = (dy / distToWp) * stepDist;
      tag.x += vx; tag.y += vy; tag.vx = vx; tag.vy = vy;
      tag.headingDeg = (Math.atan2(vx, -vy) * 180 / Math.PI + 360) % 360;
    }
  }

  const measurements = [];
  for (const stop of preset.stops) {
    const sig = calculateSignalMeasurement(tag, stop, preset.buildings, config);
    if (sig) measurements.push(sig);
  }

  const valid = measurements.filter(m => !m.isNlosZeroWeighted);
  if (valid.length === 2) {
    total2ASteps++;
    const a1 = stopsMap.get(valid[0].stopId)!;
    const a2 = stopsMap.get(valid[1].stopId)!;
    const [raw1, raw2] = findTwoCircleIntersections(a1, valid[0].noisyDistanceMeters * 2.8, a2, valid[1].noisyDistanceMeters * 2.8);

    const uX = Math.sin(tag.headingDeg * Math.PI / 180);
    const uY = -Math.cos(tag.headingDeg * Math.PI / 180);

    // Track two continuous branches
    let cand1: Point2D, cand2: Point2D;
    if (!b1 || !b2) {
      cand1 = raw1;
      cand2 = raw2;
    } else {
      // Proximity assignment to previous branch positions
      const d11 = distance(raw1, b1) + distance(raw2, b2);
      const d12 = distance(raw1, b2) + distance(raw2, b1);
      if (d11 <= d12) {
        cand1 = raw1; cand2 = raw2;
      } else {
        cand1 = raw2; cand2 = raw1;
      }
    }

    let chosen: Point2D, outcast: Point2D;

    if (b1 && b2) {
      const v1x = cand1.x - b1.x;
      const v1y = cand1.y - b1.y;
      const v2x = cand2.x - b2.x;
      const v2y = cand2.y - b2.y;

      const d1 = Math.hypot(v1x, v1y) || 1;
      const d2 = Math.hypot(v2x, v2y) || 1;

      const align1 = (v1x * uX + v1y * uY) / d1;
      const align2 = (v2x * uX + v2y * uY) / d2;

      // Higher alignment along collar azimuth wins
      if (align1 >= align2) {
        chosen = cand1;
        outcast = cand2;
      } else {
        chosen = cand2;
        outcast = cand1;
      }
    } else {
      // Cold boot: pick closer to tag
      chosen = cand1;
      outcast = cand2;
    }

    b1 = cand1;
    b2 = cand2;

    const dChosenReal = distance(chosen, tag) / config.pixelsPerMeter;
    const dOutcastReal = distance(outcast, tag) / config.pixelsPerMeter;

    totalErrMeters += dChosenReal;
    if (dChosenReal > maxErrMeters) maxErrMeters = dChosenReal;

    if (dChosenReal > dOutcastReal + 1.0) {
      fartherCandidateChosenCount++;
      if (fartherCandidateChosenCount <= 5) {
        console.log(`[Step ${step}] Farther chosen: chosen=${dChosenReal.toFixed(1)}m, outcast=${dOutcastReal.toFixed(1)}m`);
      }
    }
  } else {
    b1 = null;
    b2 = null;
  }
}

console.log('========================================================');
console.log(`Total 2-Anchor Steps: ${total2ASteps}`);
console.log(`Farther Candidate Chosen Count (>1.0m difference): ${fartherCandidateChosenCount}`);
console.log(`Average Error: ${(totalErrMeters / total2ASteps).toFixed(2)} m`);
console.log(`Max Error: ${maxErrMeters.toFixed(2)} m`);
console.log('========================================================');
