// Log steps 40 to 80
import fs from 'fs';

let content = fs.readFileSync('scratch/test_jump_killer.mjs', 'utf8');
content = content.replace(
  'lastEstimatedPos = { ...estPos };',
  `if (step >= 40 && step <= 70) {
    console.log(\`Step \${step}: tag=(\${Math.round(tag.x)}, \${Math.round(tag.y)}) | est=(\${Math.round(estPos.x)}, \${Math.round(estPos.y)}) | err=\${errorMeters.toFixed(1)}m | anchors=\${validAnchors.length} (\${validAnchors.map(a => a.id).join(',')})\`);
  }
  lastEstimatedPos = { ...estPos };`
);
fs.writeFileSync('scratch/test_jump_killer.mjs', content);
