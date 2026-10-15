// Run 250 steps to see complete baseline crossing from y = 260 to y = 330
import fs from 'fs';

let content = fs.readFileSync('scratch/test_baseline_crossing_clean.mjs', 'utf8');
content = content.replace('for (let step = 0; step < 80; step++) {', 'for (let step = 0; step < 250; step++) {');
content = content.replace(
  'if (step % 10 === 0 || Math.abs(trueDog.y - 300) < 3.0) {',
  'if (step % 25 === 0 || Math.abs(trueDog.y - 300) < 1.0) {'
);
fs.writeFileSync('scratch/test_baseline_crossing_clean.mjs', content);
