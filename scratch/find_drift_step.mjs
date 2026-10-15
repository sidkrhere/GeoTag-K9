// Find which step error first exceeded 10m
import fs from 'fs';

let content = fs.readFileSync('scratch/test_jump_killer.mjs', 'utf8');
content = content.replace(
  'const isLog = step < 25;',
  'const isLog = errorMeters > 10.0;'
);
content = content.replace(
  'if (isLog) {',
  'if (isLog && errorMeters > 10.0) {\n    if (!global.firstLogged) { global.firstLogged = true; console.log("FIRST HIGH ERROR AT STEP " + step); }\n'
);
fs.writeFileSync('scratch/test_jump_killer.mjs', content);
