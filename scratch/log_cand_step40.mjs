// Print cand1, cand2, d1, d2, scores at step 40
import fs from 'fs';

let content = fs.readFileSync('scratch/test_jump_killer.mjs', 'utf8');
content = content.replace(
  'const chosen = score1 <= score2 ? cand1 : cand2;',
  `if (step === 40 || step === 50) {
    console.log(\`Step \${step} Candidates:\`);
    console.log(\`  tag: (\${tag.x.toFixed(1)}, \${tag.y.toFixed(1)})\`);
    console.log(\`  consensusRef: (\${consensusRef.x.toFixed(1)}, \${consensusRef.y.toFixed(1)})\`);
    console.log(\`  cand1: (\${cand1.x.toFixed(1)}, \${cand1.y.toFixed(1)}) | distToTag: \${(distance(cand1, tag)/2.8).toFixed(1)}m | dToConsensus: \${(d1/2.8).toFixed(1)}m | score: \${score1.toFixed(1)}\`);
    console.log(\`  cand2: (\${cand2.x.toFixed(1)}, \${cand2.y.toFixed(1)}) | distToTag: \${(distance(cand2, tag)/2.8).toFixed(1)}m | dToConsensus: \${(d2/2.8).toFixed(1)}m | score: \${score2.toFixed(1)}\`);
  }
  const chosen = score1 <= score2 ? cand1 : cand2;`
);
fs.writeFileSync('scratch/test_jump_killer.mjs', content);
