// Test with transit zone chordMeters < 6.0
import fs from 'fs';

let content = fs.readFileSync('scratch/test_compass_guided_crossing.mjs', 'utf8');
content = content.replace(
  'const isTransitZone = chordMeters < 15.0;',
  'const isTransitZone = chordMeters < 6.0;'
);
fs.writeFileSync('scratch/test_compass_guided_crossing.mjs', content);
