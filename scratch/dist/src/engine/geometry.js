"use strict";
var __assign = (this && this.__assign) || function () {
    __assign = Object.assign || function(t) {
        for (var s, i = 1, n = arguments.length; i < n; i++) {
            s = arguments[i];
            for (var p in s) if (Object.prototype.hasOwnProperty.call(s, p))
                t[p] = s[p];
        }
        return t;
    };
    return __assign.apply(this, arguments);
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.distance = distance;
exports.lineSegmentsIntersect = lineSegmentsIntersect;
exports.countBuildingWallIntersections = countBuildingWallIntersections;
exports.isPointInPolygon = isPointInPolygon;
exports.findNearestPoint = findNearestPoint;
exports.signedCrossProduct = signedCrossProduct;
function distance(p1, p2) {
    var dx = p1.x - p2.x;
    var dy = p1.y - p2.y;
    return Math.sqrt(dx * dx + dy * dy);
}
/**
 * Checks if two line segments (p1-p2) and (p3-p4) intersect.
 */
function lineSegmentsIntersect(p1, p2, p3, p4) {
    function ccw(a, b, c) {
        return (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x);
    }
    var c1 = ccw(p1, p3, p4);
    var c2 = ccw(p2, p3, p4);
    var c3 = ccw(p1, p2, p3);
    var c4 = ccw(p1, p2, p4);
    return c1 !== c2 && c3 !== c4;
}
/**
 * Counts how many wall segments of a building polygon intersect the line segment between tag and stop.
 */
function countBuildingWallIntersections(tag, stop, building) {
    if (!building.closed || building.points.length < 3)
        return 0;
    var intersections = 0;
    var pts = building.points;
    for (var i = 0; i < pts.length; i++) {
        var nextIdx = (i + 1) % pts.length;
        if (lineSegmentsIntersect(tag, stop, pts[i], pts[nextIdx])) {
            intersections++;
        }
    }
    return intersections;
}
/**
 * Checks if a point is inside a closed polygon (Ray casting algorithm).
 */
function isPointInPolygon(point, polygon) {
    if (polygon.length < 3)
        return false;
    var inside = false;
    for (var i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
        var xi = polygon[i].x, yi = polygon[i].y;
        var xj = polygon[j].x, yj = polygon[j].y;
        var intersect = ((yi > point.y) !== (yj > point.y))
            && (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi);
        if (intersect)
            inside = !inside;
    }
    return inside;
}
/**
 * Finds the nearest point or vertex to a given coordinate within a tolerance radius.
 */
function findNearestPoint(target, points, maxDistance) {
    if (maxDistance === void 0) { maxDistance = 20; }
    var nearest = null;
    var minDist = maxDistance;
    for (var _i = 0, points_1 = points; _i < points_1.length; _i++) {
        var item = points_1[_i];
        var d = distance(target, item.point);
        if (d < minDist) {
            minDist = d;
            nearest = __assign(__assign({}, item), { dist: d });
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
function signedCrossProduct(a, b, p) {
    return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}
