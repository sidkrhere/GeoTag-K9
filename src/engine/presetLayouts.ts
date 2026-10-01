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

  // Single clean campus complex: K9 Academic & Research Hub
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

  // 7 GeoStops: 5 Tier 1 Solar Poles + 2 Tier 2 Anchors
  // Arranged across the campus complex to showcase 3D locks, 2-anchor outcasting, and 15" wall NLOS zero-weighting
  const stops: GeoStop[] = [
    // 1. North High-Mast Solar Pole (Tier 1)
    {
      id: 'stop-north',
      name: 'North Plaza Mast #1',
      tier: 1,
      x: cx,
      y: cy - 190,
      isOnline: true,
      batteryMv: 4180
    },
    // 2. Solar Pole #3 near Turn #1 (Tier 1)
    {
      id: 'stop-solar-3',
      name: 'Solar Pole #3',
      tier: 1,
      x: cx - 170,
      y: cy - 110,
      isOnline: true,
      batteryMv: 4150
    },
    // 3. Solar Pole #4 near Turn #2 (Tier 1)
    {
      id: 'stop-solar-4',
      name: 'Solar Pole #4',
      tier: 1,
      x: cx + 170,
      y: cy - 110,
      isOnline: true,
      batteryMv: 4160
    },
    // 4. East Veranda Anchor #2 (Tier 2 Eatery / Ground-Level)
    {
      id: 'stop-east',
      name: 'East Veranda Anchor #2',
      tier: 2,
      x: cx + 210,
      y: cy + 15,
      isOnline: true
    },
    // 5. Solar Pole #5 near Turn #3 (Tier 1)
    {
      id: 'stop-solar-5',
      name: 'Solar Pole #5',
      tier: 1,
      x: cx + 170,
      y: cy + 100,
      isOnline: true,
      batteryMv: 4140
    },
    // 6. South Promenade Mast #3 (Tier 1)
    {
      id: 'stop-south',
      name: 'South Promenade Mast #3',
      tier: 1,
      x: cx,
      y: cy + 190,
      isOnline: true,
      batteryMv: 4120
    },
    // 7. West Garden Courtyard Anchor #4 (Tier 2 Eatery / Ground-Level)
    {
      id: 'stop-west',
      name: 'West Garden Anchor #4',
      tier: 2,
      x: cx - 210,
      y: cy + 90,
      isOnline: true
    }
  ];

  // Designated rectangular perimeter lap circuit around the single building
  // Positioned along the exterior walkway with 35px clearance from walls
  const lapPath: Point2D[] = [
    { x: cx - 165, y: cy - 120 }, // Turn #1 (North-West)
    { x: cx + 165, y: cy - 120 }, // Turn #2 (North-East)
    { x: cx + 165, y: cy + 120 }, // Turn #3 (South-East)
    { x: cx - 165, y: cy + 120 }  // Turn #4 / Next Turn (South-West)
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
