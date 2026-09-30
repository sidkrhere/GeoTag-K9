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
exports.resetDogTrajectory = resetDogTrajectory;
exports.findTwoCircleIntersections = findTwoCircleIntersections;
exports.solveTrilateration = solveTrilateration;
var geometry_1 = require("./geometry");
var dogTrajectoryCache = new Map();
/**
 * Resets the kinematic trajectory filter for a dog when manually moved/dragged.
 * Prevents mouse drag displacement from being counted as physical dog velocity.
 */
function resetDogTrajectory(tagId, pos) {
    dogTrajectoryCache.set(tagId, {
        lastPosition: { x: pos.x, y: pos.y },
        lastTimestamp: Date.now(),
        velocity: { x: 0, y: 0 },
        initialized: true,
        historyTrail: [{ x: pos.x, y: pos.y }],
        lastDualCandidates: undefined,
        lastChosenCandidateIdx: undefined,
        lastAnchorCount: undefined
    });
}
/**
 * Calculates the Geometric Dilution of Precision (GDOP) for a 2D anchor constellation.
 */
function calculateGDOP(anchors, target) {
    if (anchors.length < 3)
        return 99.9;
    // Build matrix H of unit vectors from target to anchors
    var H = [];
    for (var _i = 0, anchors_1 = anchors; _i < anchors_1.length; _i++) {
        var a = anchors_1[_i];
        var d = Math.max(0.1, (0, geometry_1.distance)(target, a));
        H.push([(target.x - a.x) / d, (target.y - a.y) / d]);
    }
    // Compute H^T * H (2x2 matrix)
    var a11 = 0, a12 = 0, a21 = 0, a22 = 0;
    for (var i = 0; i < H.length; i++) {
        a11 += H[i][0] * H[i][0];
        a12 += H[i][0] * H[i][1];
        a21 += H[i][1] * H[i][0];
        a22 += H[i][1] * H[i][1];
    }
    var det = a11 * a22 - a12 * a21;
    if (Math.abs(det) < 1e-5) {
        return 99.9; // Collinear degeneracy
    }
    var invA11 = a22 / det;
    var invA22 = a11 / det;
    var trace = invA11 + invA22;
    return trace > 0 ? Math.min(99.9, Math.round(Math.sqrt(trace) * 100) / 100) : 99.9;
}
/**
 * Computes exact analytical intersection of two circles.
 * Returns two symmetric candidate points across the axis of centers.
 */
function findTwoCircleIntersections(a1, r1, a2, r2) {
    var d = (0, geometry_1.distance)(a1, a2);
    if (d < 1e-4) {
        return [
            { x: a1.x + r1, y: a1.y },
            { x: a1.x - r1, y: a1.y }
        ];
    }
    // Handle case where circles do not intersect due to noise
    var effectiveR1 = r1;
    var effectiveR2 = r2;
    if (d > r1 + r2) {
        var scale = (d / (r1 + r2)) * 1.002;
        effectiveR1 *= scale;
        effectiveR2 *= scale;
    }
    else if (d < Math.abs(r1 - r2)) {
        if (r1 > r2)
            effectiveR1 = d + r2 - 0.001;
        else
            effectiveR2 = d + r1 - 0.001;
    }
    var a = (effectiveR1 * effectiveR1 - effectiveR2 * effectiveR2 + d * d) / (2 * d);
    var h = Math.sqrt(Math.max(0, effectiveR1 * effectiveR1 - a * a));
    var p0x = a1.x + (a / d) * (a2.x - a1.x);
    var p0y = a1.y + (a / d) * (a2.y - a1.y);
    // Perpendicular vector
    var rx = -(a2.y - a1.y) * (h / d);
    var ry = (a2.x - a1.x) * (h / d);
    return [
        { x: p0x + rx, y: p0y + ry },
        { x: p0x - rx, y: p0y - ry }
    ];
}
/**
 * Weighted Damped Gauss-Newton Least Squares Solver.
 * Supports individual anchor weights to downweight wall-attenuated signals.
 */
function solveWeightedGaussNewton(anchors, radii, weights, initialGuess) {
    var estX = initialGuess.x;
    var estY = initialGuess.y;
    var maxIterations = 20;
    var damping = 0.6;
    for (var iter = 0; iter < maxIterations; iter++) {
        var j11 = 0, j12 = 0, j22 = 0;
        var r1 = 0, r2 = 0;
        for (var i = 0; i < anchors.length; i++) {
            var w = weights[i] || 1.0;
            var dx = estX - anchors[i].x;
            var dy = estY - anchors[i].y;
            var currentDist = Math.max(0.1, Math.sqrt(dx * dx + dy * dy));
            var residual = currentDist - radii[i];
            var jx = (dx / currentDist) * w;
            var jy = (dy / currentDist) * w;
            j11 += jx * jx;
            j12 += jx * jy;
            j22 += jy * jy;
            r1 += jx * (residual * w);
            r2 += jy * (residual * w);
        }
        var det = j11 * j22 - j12 * j12;
        if (Math.abs(det) < 1e-6)
            break;
        var deltaX = -(j22 * r1 - j12 * r2) / det;
        var deltaY = -(-j12 * r1 + j11 * r2) / det;
        estX += deltaX * damping;
        estY += deltaY * damping;
        if (deltaX * deltaX + deltaY * deltaY < 0.05)
            break;
    }
    var totalResidual = 0;
    for (var i = 0; i < anchors.length; i++) {
        var d = (0, geometry_1.distance)({ x: estX, y: estY }, anchors[i]);
        totalResidual += Math.abs(d - radii[i]);
    }
    return {
        point: { x: estX, y: estY },
        residualNorm: totalResidual / anchors.length
    };
}
/**
 * Master Multilateration Solver with Temporal Kinematic History & Innovation Gating.
 * Prevents "majority corrupted consensus" when anchors behind walls outvote clean line-of-sight anchors.
 */
function solveTrilateration(tag, measurements, stopsMap, config, buildings) {
    if (buildings === void 0) { buildings = []; }
    var now = Date.now();
    // Retrieve or initialize temporal state
    var state = dogTrajectoryCache.get(tag.id);
    if (!state) {
        state = {
            lastPosition: { x: tag.x, y: tag.y },
            lastTimestamp: now - 500,
            velocity: { x: 0, y: 0 },
            initialized: false,
            historyTrail: [{ x: tag.x, y: tag.y }]
        };
        dogTrajectoryCache.set(tag.id, state);
    }
    // Calculate elapsed time dt and predict position horizon
    var dt = Math.min(1.5, Math.max(0.05, (now - state.lastTimestamp) / 1000));
    var predX = state.lastPosition.x + state.velocity.x * dt;
    var predY = state.lastPosition.y + state.velocity.y * dt;
    var predPos = { x: predX, y: predY };
    // Canine sprint maximum speed: 7.5 m/s (~27 km/h)
    var maxSpeedMetersPerSec = 7.5;
    var maxDisplacementPixels = (maxSpeedMetersPerSec * dt + 3.0) * config.pixelsPerMeter;
    // Case 0: Blind Zone (0 Anchors in range)
    if (measurements.length === 0) {
        state.lastDualCandidates = undefined;
        state.lastChosenCandidateIdx = undefined;
        state.lastAnchorCount = 0;
        return {
            tagId: tag.id,
            estimatedX: state.lastPosition.x,
            estimatedY: state.lastPosition.y,
            errorMeters: 0,
            lockType: 'BLIND_ZONE',
            contributingAnchorIds: [],
            gdop: 99.9
        };
    }
    // Case 1: Exactly 1 Anchor in range
    if (measurements.length === 1) {
        state.lastDualCandidates = undefined;
        state.lastChosenCandidateIdx = undefined;
        state.lastAnchorCount = 1;
        var m = measurements[0];
        var stop_1 = stopsMap.get(m.stopId);
        if (!stop_1) {
            return {
                tagId: tag.id,
                estimatedX: tag.x,
                estimatedY: tag.y,
                errorMeters: 0,
                lockType: 'BLIND_ZONE',
                contributingAnchorIds: [],
                gdop: 99.9
            };
        }
        var radiusPixels = m.noisyDistanceMeters * config.pixelsPerMeter;
        var radialError = Math.abs(m.noisyDistanceMeters - m.trueDistanceMeters);
        return {
            tagId: tag.id,
            estimatedX: stop_1.x,
            estimatedY: stop_1.y,
            errorMeters: Math.round(radialError * 10) / 10,
            lockType: 'RADIAL_RING',
            radialRing: {
                center: { x: stop_1.x, y: stop_1.y },
                radiusPixels: radiusPixels,
                radiusMeters: Math.round(m.noisyDistanceMeters * 10) / 10,
                stopId: stop_1.id
            },
            contributingAnchorIds: [stop_1.id],
            gdop: 99.9
        };
    }
    // Case 2: Exactly 2 Anchors in range
    if (measurements.length === 2) {
        var m1 = measurements[0];
        var m2 = measurements[1];
        var stop1 = stopsMap.get(m1.stopId);
        var stop2 = stopsMap.get(m2.stopId);
        if (!stop1 || !stop2) {
            return {
                tagId: tag.id,
                estimatedX: tag.x,
                estimatedY: tag.y,
                errorMeters: 0,
                lockType: 'BLIND_ZONE',
                contributingAnchorIds: [],
                gdop: 99.9
            };
        }
        var r1Pixels = m1.noisyDistanceMeters * config.pixelsPerMeter;
        var r2Pixels = m2.noisyDistanceMeters * config.pixelsPerMeter;
        var _a = findTwoCircleIntersections({ x: stop1.x, y: stop1.y }, r1Pixels, { x: stop2.x, y: stop2.y }, r2Pixels), p1_1 = _a[0], p2_1 = _a[1];
        var stop1Pt = { x: stop1.x, y: stop1.y };
        var stop2Pt = { x: stop2.x, y: stop2.y };
        // 1. Campus Building Map-Matching Exclusion:
        // Dogs cannot run inside solid brick classrooms while on open campus quads!
        var p1InBuilding = buildings.some(function (b) { return b.closed && (0, geometry_1.isPointInPolygon)(p1_1, b.points); });
        var p2InBuilding = buildings.some(function (b) { return b.closed && (0, geometry_1.isPointInPolygon)(p2_1, b.points); });
        // 2. Baseline Half-Plane Invariant & Topological Side Tracking:
        // Directed line from stop1 -> stop2 forms the baseline axis.
        var cp1 = (0, geometry_1.signedCrossProduct)(stop1Pt, stop2Pt, p1_1);
        var cp2 = (0, geometry_1.signedCrossProduct)(stop1Pt, stop2Pt, p2_1);
        var p1SideSign = cp1 >= 0 ? 1 : -1;
        var p2SideSign = cp2 >= 0 ? 1 : -1;
        // Check which side of the baseline the historical trail has lived on
        var posCount = 0;
        var negCount = 0;
        var trail = state.historyTrail && state.historyTrail.length > 0
            ? state.historyTrail
            : [state.lastPosition];
        for (var _i = 0, trail_1 = trail; _i < trail_1.length; _i++) {
            var pt = trail_1[_i];
            var cp = (0, geometry_1.signedCrossProduct)(stop1Pt, stop2Pt, pt);
            if (cp > 0.01)
                posCount++;
            else if (cp < -0.01)
                negCount++;
        }
        var trailSideSign = posCount >= negCount ? 1 : -1;
        // Separation between the two symmetric intersection candidates
        var separationChordPixels = (0, geometry_1.distance)(p1_1, p2_1);
        var separationChordMeters = Math.round((separationChordPixels / config.pixelsPerMeter) * 10) / 10;
        var isNearChordCrossing = separationChordMeters < 8.0;
        // Physical baseline segment crossing check
        var p1CrossedSegment = (0, geometry_1.lineSegmentsIntersect)(state.lastPosition, p1_1, stop1Pt, stop2Pt);
        var p2CrossedSegment = (0, geometry_1.lineSegmentsIntersect)(state.lastPosition, p2_1, stop1Pt, stop2Pt);
        // 3. Multi-Point Trail Proximity Metric:
        // Minimum distance from each candidate to any point on the historical trail
        var minTrailDistP1 = Infinity;
        var minTrailDistP2 = Infinity;
        for (var _b = 0, trail_2 = trail; _b < trail_2.length; _b++) {
            var h = trail_2[_b];
            var d1 = (0, geometry_1.distance)(h, p1_1);
            var d2 = (0, geometry_1.distance)(h, p2_1);
            if (d1 < minTrailDistP1)
                minTrailDistP1 = d1;
            if (d2 < minTrailDistP2)
                minTrailDistP2 = d2;
        }
        // Kinematic predicted position distance
        var distPredP1 = (0, geometry_1.distance)(predPos, p1_1);
        var distPredP2 = (0, geometry_1.distance)(predPos, p2_1);
        // Composite candidate scoring: lower score is better
        // Base score combines trail proximity and forward kinematic innovation
        var scoreP1 = minTrailDistP1 * 0.7 + distPredP1 * 0.3;
        var scoreP2 = minTrailDistP2 * 0.7 + distPredP2 * 0.3;
        // Map exclusion penalty (cannot be inside solid building)
        if (p1InBuilding && !p2InBuilding)
            scoreP1 += 2500;
        if (p2InBuilding && !p1InBuilding)
            scoreP2 += 2500;
        // Baseline Half-Plane Invariant Penalty:
        // Candidate on the opposite side of the baseline without crossing the baseline segment
        // receives a massive 5000px penalty. Noise can NEVER flip across the baseline!
        if (!isNearChordCrossing) {
            if (p1SideSign !== trailSideSign && !p1CrossedSegment) {
                scoreP1 += 5000;
            }
            if (p2SideSign !== trailSideSign && !p2CrossedSegment) {
                scoreP2 += 5000;
            }
        }
        // Selection
        var validatedIdx = scoreP1 <= scoreP2 ? 0 : 1;
        var outcastIdx = validatedIdx === 0 ? 1 : 0;
        var chosen = validatedIdx === 0 ? p1_1 : p2_1;
        var outcastCandidate = validatedIdx === 0 ? p2_1 : p1_1;
        var outcastLeapMeters = Math.round(((0, geometry_1.distance)(chosen, outcastCandidate) / config.pixelsPerMeter) * 10) / 10;
        var lockType = 'DUAL_KINEMATIC_LOCK';
        var reason = '';
        if (!state.initialized) {
            lockType = 'DUAL_COLD_BOOT';
            reason = 'Cold Boot: Establishing topological baseline reference';
        }
        else if (p1InBuilding || p2InBuilding) {
            lockType = 'DUAL_KINEMATIC_LOCK';
            reason = "Map Occlusion: Candidate #".concat(outcastIdx + 1, " inside solid campus building wall");
        }
        else if (isNearChordCrossing) {
            lockType = 'DUAL_AMBIGUOUS_CHORD';
            reason = "Baseline Chord Crossing: Circles converge (Separation ".concat(separationChordMeters, "m)");
        }
        else {
            lockType = 'DUAL_KINEMATIC_LOCK';
            reason = "Baseline Invariant: Candidate #".concat(outcastIdx + 1, " rejected across anchor baseline (\u0394").concat(outcastLeapMeters, "m leap)");
        }
        // Velocity filtering & state update
        var rawVx = (chosen.x - state.lastPosition.x) / dt;
        var rawVy = (chosen.y - state.lastPosition.y) / dt;
        var rawSpeed = Math.sqrt(rawVx * rawVx + rawVy * rawVy);
        var maxSpeedPx_1 = maxSpeedMetersPerSec * config.pixelsPerMeter;
        var clampedVx_1 = rawSpeed > maxSpeedPx_1 ? (rawVx / rawSpeed) * maxSpeedPx_1 : rawVx;
        var clampedVy_1 = rawSpeed > maxSpeedPx_1 ? (rawVy / rawSpeed) * maxSpeedPx_1 : rawVy;
        state.velocity = {
            x: clampedVx_1 * 0.35 + state.velocity.x * 0.65,
            y: clampedVy_1 * 0.35 + state.velocity.y * 0.65
        };
        if (!state.historyTrail)
            state.historyTrail = [];
        state.historyTrail.push(__assign({}, chosen));
        if (state.historyTrail.length > 8)
            state.historyTrail.shift();
        state.lastDualCandidates = [p1_1, p2_1];
        state.lastChosenCandidateIdx = validatedIdx;
        state.lastAnchorCount = 2;
        state.lastPosition = chosen;
        state.lastTimestamp = now;
        state.initialized = true;
        var trueErrMeters = (0, geometry_1.distance)({ x: tag.x, y: tag.y }, chosen) / config.pixelsPerMeter;
        return {
            tagId: tag.id,
            estimatedX: chosen.x,
            estimatedY: chosen.y,
            errorMeters: Math.round(trueErrMeters * 10) / 10,
            lockType: lockType,
            dualCandidates: [p1_1, p2_1],
            dualInfo: {
                candidate1: p1_1,
                candidate2: p2_1,
                validatedIndex: validatedIdx,
                outcastIndex: outcastIdx,
                outcastLeapMeters: outcastLeapMeters,
                separationChordMeters: separationChordMeters,
                reason: reason
            },
            contributingAnchorIds: [stop1.id, stop2.id],
            gdop: 4.8
        };
    }
    // Case 3: 3 or More Anchors (Trilateration with Kinematic Innovation Gating & Hypothesis Re-alignment)
    var hypothesisFlipDetected = false;
    // 1. Evaluate if a hypothesis flip occurred from the previous 2-anchor state
    // Resolves: "when the dog moves from 2 to 3 geostops, and the previously outcast one is actually nearer to the historical path"
    if (state.lastDualCandidates && state.lastChosenCandidateIdx !== undefined) {
        var _c = state.lastDualCandidates, prevP1 = _c[0], prevP2 = _c[1];
        var prevChosen = state.lastChosenCandidateIdx === 0 ? prevP1 : prevP2;
        var prevOutcast = state.lastChosenCandidateIdx === 0 ? prevP2 : prevP1;
        var errChosenTotal = 0;
        var errOutcastTotal = 0;
        var evalCount = 0;
        for (var _d = 0, measurements_1 = measurements; _d < measurements_1.length; _d++) {
            var m = measurements_1[_d];
            var stop_2 = stopsMap.get(m.stopId);
            if (!stop_2)
                continue;
            var stopPt = { x: stop_2.x, y: stop_2.y };
            var measuredPx = m.noisyDistanceMeters * config.pixelsPerMeter;
            errChosenTotal += Math.abs((0, geometry_1.distance)(prevChosen, stopPt) - measuredPx);
            errOutcastTotal += Math.abs((0, geometry_1.distance)(prevOutcast, stopPt) - measuredPx);
            evalCount++;
        }
        if (evalCount >= 3) {
            var avgChosenM = (errChosenTotal / evalCount) / config.pixelsPerMeter;
            var avgOutcastM = (errOutcastTotal / evalCount) / config.pixelsPerMeter;
            // If the previously outcast candidate fits all 3 anchors significantly better:
            if (avgOutcastM < 8.0 && avgChosenM > avgOutcastM + 8.0) {
                // HYPOTHESIS FLIP DETECTED!
                // The 3rd anchor proves that the dog was actually at prevOutcast all along!
                hypothesisFlipDetected = true;
                predPos.x = prevOutcast.x;
                predPos.y = prevOutcast.y;
                state.historyTrail = [__assign({}, prevOutcast)];
            }
        }
        state.lastDualCandidates = undefined;
    }
    var allAnchors = [];
    for (var _e = 0, measurements_2 = measurements; _e < measurements_2.length; _e++) {
        var m = measurements_2[_e];
        var stop_3 = stopsMap.get(m.stopId);
        if (!stop_3)
            continue;
        var stopPoint = { x: stop_3.x, y: stop_3.y };
        var radiusPx = m.noisyDistanceMeters * config.pixelsPerMeter;
        // Innovation residual: difference between measured radius and predicted distance
        var expectedDistPx = (0, geometry_1.distance)(predPos, stopPoint);
        var expectedDistM = expectedDistPx / config.pixelsPerMeter;
        var innovationM = Math.abs(m.noisyDistanceMeters - expectedDistM);
        allAnchors.push({
            id: stop_3.id,
            point: stopPoint,
            radius: radiusPx,
            measurement: m,
            innovationMeters: innovationM
        });
    }
    if (allAnchors.length < 3) {
        return {
            tagId: tag.id,
            estimatedX: tag.x,
            estimatedY: tag.y,
            errorMeters: 0,
            lockType: 'BLIND_ZONE',
            contributingAnchorIds: [],
            gdop: 99.9
        };
    }
    // Kinematic Innovation Gating:
    // Identify anchors whose distance jumped impossibly due to wall attenuation or multipath
    var outlierIds = [];
    var inliers = [];
    // Gating threshold: normal noise produces ~4m residual; wall loss produces > 10-25m jump!
    var innovationThresholdMeters = Math.max(6.5, config.wallAttenuationDb / 2.2);
    for (var _f = 0, allAnchors_1 = allAnchors; _f < allAnchors_1.length; _f++) {
        var a = allAnchors_1[_f];
        // If a hypothesis flip was just corrected, do not prune inliers aggressively
        if (state.initialized && a.innovationMeters > innovationThresholdMeters && !hypothesisFlipDetected) {
            outlierIds.push(a.id);
            a.measurement.isOutlier = true;
        }
        else {
            inliers.push(a);
            a.measurement.isOutlier = false;
        }
    }
    // Fallback: If innovation gating pruned too aggressively, use soft weights
    var activeAnchors = inliers.length >= 3 ? inliers : allAnchors;
    var weights = activeAnchors.map(function (a) {
        if (a.measurement.isOutlier)
            return 0.1; // heavily discounted
        return 1.0 / (1.0 + Math.max(0, a.innovationMeters - 2.0));
    });
    var initialGuess = state.initialized
        ? predPos
        : {
            x: activeAnchors.reduce(function (acc, a) { return acc + a.point.x; }, 0) / activeAnchors.length,
            y: activeAnchors.reduce(function (acc, a) { return acc + a.point.y; }, 0) / activeAnchors.length
        };
    var solution = solveWeightedGaussNewton(activeAnchors.map(function (a) { return a.point; }), activeAnchors.map(function (a) { return a.radius; }), weights, initialGuess);
    var solvedPoint = solution.point;
    // Enforce canine velocity limit: Dog cannot teleport!
    if (state.initialized) {
        var stepDist = (0, geometry_1.distance)(state.lastPosition, solvedPoint);
        var maxAllowableStep = hypothesisFlipDetected
            ? maxDisplacementPixels * 1.5
            : maxDisplacementPixels;
        if (stepDist > maxAllowableStep) {
            var scale = maxAllowableStep / stepDist;
            solvedPoint = {
                x: state.lastPosition.x + (solvedPoint.x - state.lastPosition.x) * scale,
                y: state.lastPosition.y + (solvedPoint.y - state.lastPosition.y) * scale
            };
        }
    }
    // Update temporal trajectory cache
    var newVx = (solvedPoint.x - state.lastPosition.x) / dt;
    var newVy = (solvedPoint.y - state.lastPosition.y) / dt;
    var newSpeed = Math.sqrt(newVx * newVx + newVy * newVy);
    var maxSpeedPx = maxSpeedMetersPerSec * config.pixelsPerMeter;
    var clampedVx = newSpeed > maxSpeedPx ? (newVx / newSpeed) * maxSpeedPx : newVx;
    var clampedVy = newSpeed > maxSpeedPx ? (newVy / newSpeed) * maxSpeedPx : newVy;
    state.velocity = {
        x: clampedVx * 0.4 + state.velocity.x * 0.6,
        y: clampedVy * 0.4 + state.velocity.y * 0.6
    };
    state.lastPosition = solvedPoint;
    state.lastTimestamp = now;
    state.initialized = true;
    state.lastAnchorCount = activeAnchors.length;
    if (!state.historyTrail)
        state.historyTrail = [];
    state.historyTrail.push(__assign({}, solvedPoint));
    if (state.historyTrail.length > 8)
        state.historyTrail.shift();
    var errPixels = (0, geometry_1.distance)({ x: tag.x, y: tag.y }, solvedPoint);
    var errMeters = errPixels / config.pixelsPerMeter;
    var gdop = calculateGDOP(activeAnchors.map(function (a) { return a.point; }), solvedPoint);
    return {
        tagId: tag.id,
        estimatedX: solvedPoint.x,
        estimatedY: solvedPoint.y,
        errorMeters: Math.round(errMeters * 10) / 10,
        lockType: 'FULL_3D_LOCK',
        contributingAnchorIds: activeAnchors.map(function (a) { return a.id; }),
        outlierAnchorIds: outlierIds,
        hypothesisFlipDetected: hypothesisFlipDetected,
        gdop: gdop
    };
}
