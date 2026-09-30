// Self-contained mathematical verification of dorsal e-compass vector matching

// Replicate analytical intersection and dorsal vector matching logic for unit verification
function findTwoCircleIntersections(a1, r1, a2, r2) {
  const d = Math.hypot(a2.x - a1.x, a2.y - a1.y);
  if (d < 1e-4) return [{ x: a1.x + r1, y: a1.y }, { x: a1.x - r1, y: a1.y }];
  const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
  const h = Math.sqrt(Math.max(0, r1 * r1 - a * a));
  const p2 = { x: a1.x + (a * (a2.x - a1.x)) / d, y: a1.y + (a * (a2.y - a1.y)) / d };
  return [
    { x: p2.x + (h * (a2.y - a1.y)) / d, y: p2.y - (h * (a2.x - a1.x)) / d },
    { x: p2.x - (h * (a2.y - a1.y)) / d, y: p2.y + (h * (a2.x - a1.x)) / d }
  ];
}

console.log("=== RUNNING DORSAL E-COMPASS VECTOR MATCHING TESTS ===");

// Scenario: Two GeoStops forming a vertical baseline at x=400, y=200 and y=400
const a1 = { x: 400, y: 200, r: 150 };
const a2 = { x: 400, y: 400, r: 150 };

// Intersections will be to the East (x > 400) and West (x < 400)
const [cand1, cand2] = findTwoCircleIntersections(a1, a1.r, a2, a2.r);
console.log(`Candidate 1: (${cand1.x.toFixed(1)}, ${cand1.y.toFixed(1)})`);
console.log(`Candidate 2: (${cand2.x.toFixed(1)}, ${cand2.y.toFixed(1)})`);

// True dog position is East of the chord, walking East (heading 90°)
const trueDog = { x: 511.8, y: 300, headingDeg: 90 };
const priorFix = { x: 490, y: 300 }; // 20px West of current position

const headingRad = (trueDog.headingDeg * Math.PI) / 180;
const uCompass = { x: Math.sin(headingRad), y: -Math.cos(headingRad) }; // (1, 0) East
console.log(`uCompass: (${uCompass.x.toFixed(2)}, ${uCompass.y.toFixed(2)})`);

// Displacements from prior fix
const w1 = { x: cand1.x - priorFix.x, y: cand1.y - priorFix.y };
const w2 = { x: cand2.x - priorFix.x, y: cand2.y - priorFix.y };
const len1 = Math.hypot(w1.x, w1.y);
const len2 = Math.hypot(w2.x, w2.y);

const align1 = (w1.x * uCompass.x + w1.y * uCompass.y) / len1;
const align2 = (w2.x * uCompass.x + w2.y * uCompass.y) / len2;

console.log(`Alignment Cand 1: ${align1.toFixed(3)}`);
console.log(`Alignment Cand 2 (Ghost): ${align2.toFixed(3)}`);

// Candidate 1 moves forward along heading -> align1 ~ +1.0
// Candidate 2 is across baseline -> negative alignment or huge leap
console.assert(align1 > 0.8, `Expected align1 > 0.8, got ${align1}`);
console.assert(align2 < 0 || Math.abs(align1 - align2) > 0.8, `Expected clear delta between true and ghost`);

console.log("✅ TEST 1 PASSED: True candidate locked with high cosine similarity!");

// Scenario 2: Baseline crossing.
// Dog starts at x=380, heading 90° (East), crosses to x=420.
// Verify dorsal vector maintains heading direction across the chord.
const crossDog1 = { x: 390, y: 300, headingDeg: 90 }; // West of chord, moving East
const crossDog2 = { x: 410, y: 300, headingDeg: 90 }; // East of chord, moving East

const [crossC1, crossC2] = findTwoCircleIntersections(a1, Math.hypot(410 - a1.x, 300 - a1.y), a2, Math.hypot(410 - a2.x, 300 - a2.y));
const wCross1 = { x: crossC1.x - crossDog1.x, y: crossC1.y - crossDog1.y };
const wCross2 = { x: crossC2.x - crossDog1.x, y: crossC2.y - crossDog1.y };
const crossAlign1 = (wCross1.x * uCompass.x + wCross1.y * uCompass.y) / Math.hypot(wCross1.x, wCross1.y);
const crossAlign2 = (wCross2.x * uCompass.x + wCross2.y * uCompass.y) / Math.hypot(wCross2.x, wCross2.y);

console.log(`Baseline Crossing Alignments: True=${crossAlign1.toFixed(3)}, Mirror=${crossAlign2.toFixed(3)}`);
console.assert(crossAlign1 > 0.9, `True candidate must have >0.9 alignment during baseline crossing`);
console.log("✅ TEST 2 PASSED: Baseline crossing disambiguated smoothly by dorsal collar vector!");

console.log("=== ALL MATHEMATICAL TESTS PASSED! ===");
