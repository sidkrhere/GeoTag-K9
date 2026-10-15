// Fix logging in test_jump_killer.mjs
import fs from 'fs';

let content = fs.readFileSync('scratch/test_jump_killer.mjs', 'utf8');
content = content.replace('const isLog = errorMeters > 10.0;', '// log after calculation');
content = content.replace(
  'const errorMeters = Math.hypot(estPos.x - tag.x, estPos.y - tag.y) / config.pixelsPerMeter;',
  `const errorMeters = Math.hypot(estPos.x - tag.x, estPos.y - tag.y) / config.pixelsPerMeter;
  if (errorMeters > 5.0 && !global.firstLogged) {
    global.firstLogged = true;
    console.log(\`FIRST HIGH ERROR AT STEP \${step}: tag=(\${Math.round(tag.x)}, \${Math.round(tag.y)}) | est=(\${Math.round(estPos.x)}, \${Math.round(estPos.y)}) | err=\${errorMeters.toFixed(1)}m | anchors=\${validAnchors.length} (\${validAnchors.map(a => a.id).join(',')})\`);
  }`
);
fs.writeFileSync('scratch/test_jump_killer.mjs', content);
