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

  // Single clean campus complex: Admin Block
  const buildings: BuildingPolygon[] = [
    {
      id: 'b-main-complex',
      name: 'Admin Block',
      closed: true,
      attenuationDb: 14,
      color: 'rgba(40, 10, 8, 0.45)',
      points: [
        { x: cx - 130, y: cy - 85 },
        { x: cx + 130, y: cy - 85 },
        { x: cx + 130, y: cy + 85 },
        { x: cx - 130, y: cy + 85 }
      ]
    }
  ];

  // Exactly 1 Central Gateway placed atop the Admin Block roof (positioned below the Admin Block title)
  const centre: GeoCentre = {
    id: 'geocentre-core',
    name: 'Central Gateway (Admin Block)',
    x: cx,
    y: cy + 26,
    isOnline: true
  };

  // 7 Unified UWB Anchors arranged across the campus complex
  const stops: GeoStop[] = [
    // 1. Anchor #1 (North Plaza)
    {
      id: 'stop-north',
      name: 'Anchor #1 (North Plaza)',
      tier: 1,
      x: cx,
      y: cy - 190,
      isOnline: true,
      batteryMv: 4180
    },
    // 2. Anchor #2 (Northwest Plaza)
    {
      id: 'stop-solar-3',
      name: 'Anchor #2 (Northwest Plaza)',
      tier: 1,
      x: cx - 170,
      y: cy - 110,
      isOnline: true,
      batteryMv: 4150
    },
    // 3. Anchor #3 (Northeast Plaza)
    {
      id: 'stop-solar-4',
      name: 'Anchor #3 (Northeast Plaza)',
      tier: 1,
      x: cx + 170,
      y: cy - 110,
      isOnline: true,
      batteryMv: 4160
    },
    // 4. Anchor #4 (East Veranda)
    {
      id: 'stop-east',
      name: 'Anchor #4 (East Veranda)',
      tier: 1,
      x: cx + 210,
      y: cy + 15,
      isOnline: true
    },
    // 5. Anchor #5 (Southeast Plaza)
    {
      id: 'stop-solar-5',
      name: 'Anchor #5 (Southeast Plaza)',
      tier: 1,
      x: cx + 170,
      y: cy + 100,
      isOnline: true,
      batteryMv: 4140
    },
    // 6. Anchor #6 (South Promenade)
    {
      id: 'stop-south',
      name: 'Anchor #6 (South Promenade)',
      tier: 1,
      x: cx,
      y: cy + 190,
      isOnline: true,
      batteryMv: 4120
    },
    // 7. Anchor #7 (West Garden)
    {
      id: 'stop-west',
      name: 'Anchor #7 (West Garden)',
      tier: 1,
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
      name: 'Bruno',
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
