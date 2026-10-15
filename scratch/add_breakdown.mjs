// Log summary per 100 steps in test_jump_killer.mjs
import fs from 'fs';

let content = fs.readFileSync('scratch/test_jump_killer.mjs', 'utf8');

// Replace bottom logging to print breakdown per 100 steps
content = content.replace(
  'console.log(`\\n=== RESULTS WITH NEW LOGIC ===`);',
  `console.log('\\n=== BREAKDOWN PER 100 STEPS ===');
for (let b = 0; b < 6; b++) {
  const bStart = b * 100;
  const bEnd = bStart + 100;
  console.log(\`Steps \${bStart}-\${bEnd}: Check segment metrics...\`);
}
console.log('\\n=== OVERALL RESULTS ===');`
);

fs.writeFileSync('scratch/test_jump_killer.mjs', content);
