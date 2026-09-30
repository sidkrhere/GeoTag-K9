// Test recreating the exact user screenshot constellation:
// Multiple anchors (some far away, some behind building, some clear LOS)
// Verify that:
// 1. Clear LOS anchors are INLIERS (never flagged as outliers).
// 2. The estimated position stays on the walkway with Bruno (error < 1.5m), NOT jumping 38m North!
// 3. Outlier badges only appear on truly obstructed anchors.

import { solveTrilateration } from '../src/engine/multilateration.ts';
