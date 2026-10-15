// Inspect step 50 in test_baseline_crossing_clean.mjs
import fs from 'fs';

let content = fs.readFileSync('scratch/test_baseline_crossing_clean.mjs', 'utf8');
content = content.replace(
  'const chosen = score1 <= score2 ? c1 : c2;',
  `if (step === 49 || step === 50) {
    console.log(\`Step \${step} Debug:\`);
    console.log(\`  extrapolatedRef: (\${extrapolatedRef.x.toFixed(1)}, \${extrapolatedRef.y.toFixed(1)})\`);
    console.log(\`  c1: (\${c1.x.toFixed(1)}, \${c1.y.toFixed(1)}) | dRef1: \${dRef1.toFixed(2)}m | align1: \${align1.toFixed(2)} | penalty1: \${halfPlanePenalty1.toFixed(1)} | score1: \${score1.toFixed(2)}\`);
    console.log(\`  c2: (\${c2.x.toFixed(1)}, \${c2.y.toFixed(1)}) | dRef2: \${dRef2.toFixed(2)}m | align2: \${align2.toFixed(2)} | penalty2: \${halfPlanePenalty2.toFixed(1)} | score2: \${score2.toFixed(2)}\`);
  }
  const chosen = score1 <= score2 ? c1 : c2;`
);
fs.writeFileSync('scratch/test_baseline_crossing_clean.mjs', content);
