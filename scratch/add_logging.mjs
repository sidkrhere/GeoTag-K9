// Check what is happening step by step in test_jump_killer.mjs
import fs from 'fs';

// Let's modify test_jump_killer to log first 30 steps
let content = fs.readFileSync('scratch/test_jump_killer.mjs', 'utf8');

// Replace the loop to log steps 0-25
content = content.replace(
  'for (let step = 0; step < 600; step++) {',
  `for (let step = 0; step < 600; step++) {
    const isLog = step < 25;`
);

content = content.replace(
  'lastEstimatedPos = { ...estPos };\n}',
  `if (isLog) {
    console.log(\`Step \${step}: tag=(\${Math.round(tag.x)}, \${Math.round(tag.y)}) | est=(\${Math.round(estPos.x)}, \${Math.round(estPos.y)}) | err=\${errorMeters.toFixed(1)}m | anchors=\${validAnchors.length}\`);
  }
  lastEstimatedPos = { ...estPos };
}`
);

fs.writeFileSync('scratch/test_jump_killer.mjs', content);
