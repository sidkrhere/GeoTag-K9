import { Point2D, GeoStop, GeoTag, BuildingPolygon, SimulationConfig, SignalMeasurement } from '../types/simulation';
import { distance, countBuildingWallIntersections } from './geometry';

/**
 * Standard Box-Muller transform to generate normally distributed random numbers (Gaussian noise).
 */
function randomGaussian(mean: number = 0, stdDev: number = 1): number {
  const u1 = Math.max(1e-10, Math.random());
  const u2 = Math.random();
  const z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
  return z0 * stdDev + mean;
}

interface NoiseCacheEntry {
  noiseDb: number;
  lastUpdateTime: number;
}

const packetNoiseCache = new Map<string, NoiseCacheEntry>();

/**
 * Computes RF signal propagation, wall attenuation, and noisy RSSI from GeoTag to GeoStop.
 */
export function calculateSignalMeasurement(
  tag: GeoTag,
  stop: GeoStop,
  buildings: BuildingPolygon[],
  config: SimulationConfig
): SignalMeasurement | null {
  const tagPos: Point2D = { x: tag.x, y: tag.y };
  const stopPos: Point2D = { x: stop.x, y: stop.y };

  const pixelDist = distance(tagPos, stopPos);
  const trueDistanceMeters = pixelDist / config.pixelsPerMeter;

  // Get active radius for this GeoStop tier
  const maxRadiusMeters = stop.tier === 1 ? config.tier1RadiusMeters : config.tier2RadiusMeters;

  // If outside physical radio horizon
  if (trueDistanceMeters > maxRadiusMeters) {
    return null;
  }

  // Count wall obstructions through building outlines
  let totalWallIntersections = 0;
  for (const b of buildings) {
    if (b.closed && b.points.length >= 3) {
      totalWallIntersections += countBuildingWallIntersections(tagPos, stopPos, b);
    }
  }

  // Log-Distance Path Loss model
  // RSSI(d) = A - 10 * n * log10(d) - WallLoss + Noise
  const A = -42;
  const n = config.pathLossExponent;
  const wallLossDb = totalWallIntersections * config.wallAttenuationDb;

  // Discretize noise sampling: Real RF collar chirps at ~10 Hz (every 100ms)
  const cacheKey = `${tag.id}-${stop.id}`;
  const now = Date.now();
  let cached = packetNoiseCache.get(cacheKey);

  if (!cached || now - cached.lastUpdateTime > 100) {
    const freshNoise = randomGaussian(0, config.noiseStdDevDb);
    const smoothed = cached ? (cached.noiseDb * 0.35 + freshNoise * 0.65) : freshNoise;
    cached = { noiseDb: smoothed, lastUpdateTime: now };
    packetNoiseCache.set(cacheKey, cached);
  }

  const channelNoiseDb = cached.noiseDb;

  // Avoid log10(0)
  const safeDist = Math.max(0.2, trueDistanceMeters);
  const rssiDbm = A - (10 * n * Math.log10(safeDist)) - wallLossDb + channelNoiseDb;

  // Invert RSSI back to estimated distance
  // noisyDistance = 10^((A - RSSI - WallLoss) / (10 * n))
  // Notice that in real scenarios the server doesn't always know exact wall positions without raycasting,
  // so wall loss directly skews the apparent distance larger!
  const effectiveRssiNoWalls = rssiDbm + (wallLossDb * 0.5); // Server partially compensates if map is known
  const noisyExponent = (A - effectiveRssiNoWalls) / (10 * n);
  const noisyDistanceMeters = Math.max(0.1, Math.pow(10, noisyExponent));

  return {
    stopId: stop.id,
    tagId: tag.id,
    trueDistanceMeters,
    noisyDistanceMeters,
    rssiDbm: Math.round(rssiDbm * 10) / 10,
    obstructed: totalWallIntersections > 0,
    numObstructions: totalWallIntersections
  };
}
