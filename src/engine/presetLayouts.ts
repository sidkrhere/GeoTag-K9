import { BuildingPolygon, GeoStop, GeoCentre, GeoTag, Point2D } from '../types/simulation';

export function getIITRoparPreset(width: number, height: number): {
  buildings: BuildingPolygon[];
  stops: GeoStop[];
  centre: GeoCentre;
  tags: GeoTag[];
} {
  // Center coordinates on canvas
  const cx = width / 2;
  const cy = height / 2;

  // Single clean, majestic campus building: K9 Academic & Research Hub
  const buildings: BuildingPolygon[] = [
    {
      id: 'b-main-complex',
      name: 'K9 Central Innovation Complex',
      closed: true,
      attenuationDb: 14,
      color: 'rgba(59, 130, 246, 0.16)',
      points: [
        { x: cx - 130, y: cy - 85 },
        { x: cx + 130, y: cy - 85 },
        { x: cx + 130, y: cy + 85 },
        { x: cx - 130, y: cy + 85 }
      ]
    }
  ];

  // Exactly 1 GeoCentre placed atop the central building roof
  const centre: GeoCentre = {
    id: 'geocentre-core',
    name: 'GeoCentre (Rooftop Master Hub)',
    x: cx,
    y: cy,
    isOnline: true
  };

  // Exactly 4 GeoStops arranged across the single building (North, East, South, West)
  // Forms orthogonal baselines that showcase 2-anchor outcasting and 3-anchor corner locks
  const stops: GeoStop[] = [
    // North High-Mast Solar Pole (Tier 1)
    {
      id: 'stop-north',
      name: 'North Plaza Mast #1',
      tier: 1,
      x: cx,
      y: cy - 135,
      isOnline: true,
      batteryMv: 4180
    },
    // East Promenade Veranda Anchor (Tier 2)
    {
      id: 'stop-east',
      name: 'East Veranda Anchor #2',
      tier: 2,
      x: cx + 190,
      y: cy,
      isOnline: true
    },
    // South High-Mast Solar Pole (Tier 1)
    {
      id: 'stop-south',
      name: 'South Promenade Mast #3',
      tier: 1,
      x: cx,
      y: cy + 135,
      isOnline: true,
      batteryMv: 4120
    },
    // West Garden Courtyard Anchor (Tier 2)
    {
      id: 'stop-west',
      name: 'West Garden Anchor #4',
      tier: 2,
      x: cx - 190,
      y: cy,
      isOnline: true
    }
  ];

  // Clean designated rectangular perimeter lap circuit around the single building
  // Positioned along the exterior walkway with 35px clearance from walls
  const lapPath: Point2D[] = [
    { x: cx - 165, y: cy - 120 }, // WP 0: Top-Left (North-West)
    { x: cx + 165, y: cy - 120 }, // WP 1: Top-Right (North-East)
    { x: cx + 165, y: cy + 120 }, // WP 2: Bottom-Right (South-East)
    { x: cx - 165, y: cy + 120 }  // WP 3: Bottom-Left (South-West)
  ];

  // Exactly 1 GeoTag K9 Dog (Bruno) taking smooth laps along the designated perimeter path
  const tags: GeoTag[] = [
    {
      id: 'k9-bruno',
      name: 'Bruno (Campus Retriever)',
      x: cx - 165,
      y: cy - 120,
      vx: 0,
      vy: 0,
      headingDeg: 90,
      compassData: {
        headingDeg: 90,
        cardinal: 'E',
        pitchDeg: 0,
        rollDeg: 0,
        yawRateDegPerSec: 0,
        sensorCurrentUa: 54,
        projectedBatteryDays: 60
      },
      autoWalk: true,
      imuResting: false,
      batteryPct: 98,
      color: '#00f2fe',
      pathWaypoints: lapPath,
      currentWaypointIndex: 1
    }
  ];

  return { buildings, stops, centre, tags };
}
