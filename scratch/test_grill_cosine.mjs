// Simulation verifying the geometric mathematics of candidate velocities
function dot(v1, v2) { return v1.x * v2.x + v1.y * v2.y; }
function norm(v) { return Math.hypot(v.x, v.y) || 1e-6; }
function cosSim(v1, v2) { return dot(v1, v2) / (norm(v1) * norm(v2)); }

console.log('Testing Candidate Velocity Cosine Similarity across baseline geometry...\n');

// Baseline: Anchor 1 at (200, 300), Anchor 2 at (600, 300) -> Horizontal baseline (y = 300)
// Case A: Dog moving parallel to baseline (East: vx = 2.5, vy = 0)
// True dog at (400, 200) -> 100m North of baseline
// Mirror ghost at (400, 400) -> 100m South of baseline
const v_true_parallel = { x: 2.5, y: 0.0 };
const v_ghost_parallel = { x: 2.5, y: -0.0 }; // reflected across y = 300
console.log('Case A (Moving Parallel to Baseline - East):');
console.log('v_true:', v_true_parallel, 'v_ghost:', v_ghost_parallel);
console.log('cos(v_true, v_ghost):', cosSim(v_true_parallel, v_ghost_parallel));

// Case B: Dog moving perpendicular to baseline (South: vx = 0, vy = 2.5)
const v_true_perp = { x: 0.0, y: 2.5 };
const v_ghost_perp = { x: 0.0, y: -2.5 }; // reflected across y = 300
console.log('\nCase B (Moving Perpendicular to Baseline - South):');
console.log('v_true:', v_true_perp, 'v_ghost:', v_ghost_perp);
console.log('cos(v_true, v_ghost):', cosSim(v_true_perp, v_ghost_perp));

// Case C: Dog moving diagonal (vx = 2.0, vy = 1.0)
const v_true_diag = { x: 2.0, y: 1.0 };
const v_ghost_diag = { x: 2.0, y: -1.0 };
console.log('\nCase C (Moving Diagonal 45 deg):');
console.log('cos(v_true, v_ghost):', cosSim(v_true_diag, v_ghost_diag));
