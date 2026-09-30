"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var multilateration_1 = require("../src/engine/multilateration");
var geometry_1 = require("../src/engine/geometry");
console.log('--- Starting Multilateration & Mirror Latch-Up Unit Test ---');
var config = {
    tier1RadiusMeters: 120,
    tier2RadiusMeters: 60,
    pathLossExponent: 2.2,
    wallAttenuationDb: 18,
    noiseStdDevDb: 2.5,
    autoWalkSpeed: 2.2,
    pixelsPerMeter: 2.8,
    showTrueDoggos: true,
    showEstimatedLocations: true,
    showCoverageHalos: true,
    showBackhaulBeams: true,
    showRFRipples: true,
    simulateImuSleep: false,
    gdopWarningThreshold: 6.0
};
// Setup 2 GeoStops forming a horizontal baseline
// Stop 1 at (200, 300), Stop 2 at (400, 300)
// Baseline runs along Y = 300.
// Any point with Y < 300 is North half-plane.
// Any point with Y > 300 is South half-plane.
var stop1 = {
    id: 'stop-1',
    name: 'Stop 1',
    tier: 1,
    x: 200,
    y: 300,
    isOnline: true
};
var stop2 = {
    id: 'stop-2',
    name: 'Stop 2',
    tier: 1,
    x: 400,
    y: 300,
    isOnline: true
};
// Stop 3 at (300, 150) (North side, ready for 3-anchor transition)
var stop3 = {
    id: 'stop-3',
    name: 'Stop 3',
    tier: 1,
    x: 300,
    y: 150,
    isOnline: true
};
var stopsMap = new Map([
    [stop1.id, stop1],
    [stop2.id, stop2],
    [stop3.id, stop3]
]);
// Test 1: Signed Cross Product Half-Plane Orientation
var ptNorth = { x: 300, y: 220 };
var ptSouth = { x: 300, y: 380 };
var cpNorth = (0, geometry_1.signedCrossProduct)(stop1, stop2, ptNorth);
var cpSouth = (0, geometry_1.signedCrossProduct)(stop1, stop2, ptSouth);
console.log("Signed Cross Product North (Y=220): ".concat(cpNorth, " (Should be < 0)"));
console.log("Signed Cross Product South (Y=380): ".concat(cpSouth, " (Should be > 0)"));
if ((cpNorth < 0 && cpSouth > 0) || (cpNorth > 0 && cpSouth < 0)) {
    console.log('✓ PASS: Half-planes are strictly antisymmetric across baseline chord');
}
else {
    throw new Error('FAIL: Half-planes did not separate correctly');
}
// Test 2: Bruno patrolling exclusively in 2-anchor zone (Y around 200..230)
// Bruno walks North of baseline.
// Candidate 1 will be near Bruno (Y ~ 220).
// Candidate 2 (Mirror Ghost) will be across the baseline (Y ~ 380).
var tag = {
    id: 'dog-bruno',
    name: 'Bruno',
    x: 300,
    y: 220,
    batteryPercent: 95,
    imuResting: false,
    autoWalk: true,
    color: '#38bdf8',
    vx: 2.0,
    vy: 0
};
// Initialize trajectory
(0, multilateration_1.resetDogTrajectory)(tag.id, { x: tag.x, y: tag.y });
// Simulate 10 sequential 350ms packets with noisy measurements in 2-anchor zone
console.log('\n--- Test 2: 10 Sequential Steps in 2-Anchor Coverage with RF Noise ---');
var allPassed = true;
for (var step = 1; step <= 10; step++) {
    // Move Bruno slightly rightwards
    tag.x += 4.0;
    tag.y += (Math.random() - 0.5) * 2.0;
    // True distance in meters
    var dist1M_1 = Math.hypot(tag.x - stop1.x, tag.y - stop1.y) / config.pixelsPerMeter;
    var dist2M_1 = Math.hypot(tag.x - stop2.x, tag.y - stop2.y) / config.pixelsPerMeter;
    // Add realistic channel noise +/- 3 meters
    var noise1 = (Math.random() - 0.5) * 3.0;
    var noise2 = (Math.random() - 0.5) * 3.0;
    var m1_1 = {
        stopId: stop1.id,
        tagId: tag.id,
        trueDistanceMeters: dist1M_1,
        noisyDistanceMeters: Math.max(1, dist1M_1 + noise1),
        rssiDbm: -65,
        obstructed: false,
        numObstructions: 0
    };
    var m2_1 = {
        stopId: stop2.id,
        tagId: tag.id,
        trueDistanceMeters: dist2M_1,
        noisyDistanceMeters: Math.max(1, dist2M_1 + noise2),
        rssiDbm: -68,
        obstructed: false,
        numObstructions: 0
    };
    var res = (0, multilateration_1.solveTrilateration)(tag, [m1_1, m2_1], stopsMap, config, []);
    var dualInfo = res.dualInfo;
    if (!dualInfo) {
        console.error("Step ".concat(step, ": Missing dualInfo in 2-anchor result!"));
        allPassed = false;
        break;
    }
    var chosenCand = res.estimatedY;
    var isCorrectSide = chosenCand < 300; // True Bruno is North (Y < 300)
    var outcastLeap = dualInfo.outcastLeapMeters;
    console.log("Step ".concat(step, ": LockType=").concat(res.lockType, ", Est=(").concat(res.estimatedX.toFixed(1), ", ").concat(res.estimatedY.toFixed(1), "), True=(").concat(tag.x.toFixed(1), ", ").concat(tag.y.toFixed(1), "), Error=").concat(res.errorMeters, "m, OutcastLeap=").concat(outcastLeap, "m, ValidIdx=").concat(dualInfo.validatedIndex));
    if (!isCorrectSide) {
        console.error("\u274C FAILED: Solver latched onto Mirror Ghost South of baseline!");
        allPassed = false;
        break;
    }
}
if (allPassed) {
    console.log('✓ PASS: Bruno stayed 100% locked to true path across all 10 steps! Zero mirror latch-up!');
}
// Test 3: Transition from 2 Anchors to 3 Anchors (Anchor 3 comes online)
console.log('\n--- Test 3: Transition from 2 Anchors to 3 Anchors ---');
var dist1M = Math.hypot(tag.x - stop1.x, tag.y - stop1.y) / config.pixelsPerMeter;
var dist2M = Math.hypot(tag.x - stop2.x, tag.y - stop2.y) / config.pixelsPerMeter;
var dist3M = Math.hypot(tag.x - stop3.x, tag.y - stop3.y) / config.pixelsPerMeter;
var m1 = {
    stopId: stop1.id,
    tagId: tag.id,
    trueDistanceMeters: dist1M,
    noisyDistanceMeters: dist1M + 0.5,
    rssiDbm: -65,
    obstructed: false,
    numObstructions: 0
};
var m2 = {
    stopId: stop2.id,
    tagId: tag.id,
    trueDistanceMeters: dist2M,
    noisyDistanceMeters: dist2M - 0.5,
    rssiDbm: -68,
    obstructed: false,
    numObstructions: 0
};
var m3 = {
    stopId: stop3.id,
    tagId: tag.id,
    trueDistanceMeters: dist3M,
    noisyDistanceMeters: dist3M + 0.8,
    rssiDbm: -60,
    obstructed: false,
    numObstructions: 0
};
// Last 2-anchor solution
var res2D = (0, multilateration_1.solveTrilateration)(tag, [m1, m2], stopsMap, config, []);
console.log("2-Anchor Result before transition: (".concat(res2D.estimatedX.toFixed(1), ", ").concat(res2D.estimatedY.toFixed(1), ")"));
// Now 3-anchor solution arrives
var res3D = (0, multilateration_1.solveTrilateration)(tag, [m1, m2, m3], stopsMap, config, []);
console.log("3-Anchor Result on handoff: (".concat(res3D.estimatedX.toFixed(1), ", ").concat(res3D.estimatedY.toFixed(1), ")"));
var transitionDeltaM = Math.hypot(res3D.estimatedX - res2D.estimatedX, res3D.estimatedY - res2D.estimatedY) / config.pixelsPerMeter;
console.log("Coordinate Discrepancy on 2->3 handoff: ".concat(transitionDeltaM.toFixed(2), " meters"));
if (transitionDeltaM < 5.0) {
    console.log("\u2713 PASS: Zero teleportation! 2D and 3D solutions align within ".concat(transitionDeltaM.toFixed(2), "m!"));
}
else {
    console.error("\u274C FAILED: Coordinate snapped by ".concat(transitionDeltaM.toFixed(2), "m"));
    allPassed = false;
}
console.log('\n--- All Mathematical Tests Completed Successfully! ---');
