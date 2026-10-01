import { Point2D, BuildingPolygon } from '../types/simulation';

export function distance(p1: Point2D, p2: Point2D): number {
  const dx = p1.x - p2.x;
  const dy = p1.y - p2.y;
  return Math.sqrt(dx * dx + dy * dy);
}

/**
 * Checks if two line segments (p1-p2) and (p3-p4) intersect.
 */
export function lineSegmentsIntersect(
  p1: Point2D,
  p2: Point2D,
  p3: Point2D,
  p4: Point2D
): boolean {
  function ccw(a: Point2D, b: Point2D, c: Point2D): boolean {
    return (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x);
  }

  const c1 = ccw(p1, p3, p4);
  const c2 = ccw(p2, p3, p4);
  const c3 = ccw(p1, p2, p3);
  const c4 = ccw(p1, p2, p4);

  return c1 !== c2 && c3 !== c4;
}

/**
 * Counts how many wall segments of a building polygon intersect the line segment between tag and stop.
 */
export function countBuildingWallIntersections(
  tag: Point2D,
  stop: Point2D,
  building: BuildingPolygon
): number {
  if (!building.closed || building.points.length < 3) return 0;
  
  let intersections = 0;
  const pts = building.points;
  for (let i = 0; i < pts.length; i++) {
    const nextIdx = (i + 1) % pts.length;
    if (lineSegmentsIntersect(tag, stop, pts[i], pts[nextIdx])) {
      intersections++;
    }
  }
  return intersections;
}

/**
 * Checks if a point is inside a closed polygon (Ray casting algorithm).
 */
export function isPointInPolygon(point: Point2D, polygon: Point2D[]): boolean {
  if (polygon.length < 3) return false;
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i].x, yi = polygon[i].y;
    const xj = polygon[j].x, yj = polygon[j].y;
    const intersect = ((yi > point.y) !== (yj > point.y))
        && (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}

/**
 * Finds the nearest point or vertex to a given coordinate within a tolerance radius.
 */
export function findNearestPoint(
  target: Point2D,
  points: { point: Point2D; id: string; meta?: any }[],
  maxDistance: number = 20
): { id: string; point: Point2D; meta?: any; dist: number } | null {
  let nearest: { id: string; point: Point2D; meta?: any; dist: number } | null = null;
  let minDist = maxDistance;

  for (const item of points) {
    const d = distance(target, item.point);
    if (d < minDist) {
      minDist = d;
      nearest = { ...item, dist: d };
    }
  }

  return nearest;
}

/**
 * Computes the 2D signed cross product (determinant) of vector (b - a) and (p - a).
 * Positive if p is to the left of directed baseline a -> b,
 * Negative if p is to the right,
 * Zero if collinear.
 */
export function signedCrossProduct(a: Point2D, b: Point2D, p: Point2D): number {
  return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

/**
 * Checks if a point is deeply inside a closed polygon (at least minDepthPixels away from all boundary edges).
 * Points within minDepthPixels of an exterior wall are treated as perimeter sidewalk jitter.
 */
export function isDeepInsidePolygon(point: Point2D, polygon: Point2D[], minDepthPixels: number = 10): boolean {
  if (!isPointInPolygon(point, polygon)) return false;
  if (minDepthPixels <= 0) return true;

  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const a = polygon[j];
    const b = polygon[i];
    const l2 = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
    let distToSeg = 0;
    if (l2 === 0) {
      distToSeg = distance(point, a);
    } else {
      let t = ((point.x - a.x) * (b.x - a.x) + (point.y - a.y) * (b.y - a.y)) / l2;
      t = Math.max(0, Math.min(1, t));
      const projX = a.x + t * (b.x - a.x);
      const projY = a.y + t * (b.y - a.y);
      distToSeg = Math.hypot(point.x - projX, point.y - projY);
    }
    if (distToSeg < minDepthPixels) {
      return false;
    }
  }

  return true;
}
