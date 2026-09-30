// Verify the new starting UI preset layout specifications
import { readFileSync } from 'fs';

const presetSource = readFileSync('./src/engine/presetLayouts.ts', 'utf8');

console.log('--- PRESET LAYOUT VERIFICATION ---');
const hasSingleBuilding = presetSource.includes('K9 Central Innovation Complex') && !presetSource.includes('Central Library');
console.log('Single Clean Building:', hasSingleBuilding ? '✓ PASS' : '✗ FAIL');

const stopMatches = [...presetSource.matchAll(/id:\s*'(stop-[^']+)'/g)].map(m => m[1]);
console.log('Stops found (count = ' + stopMatches.length + '):', stopMatches.join(', '));
const has4Stops = stopMatches.length === 4;
console.log('Exactly 4 GeoStops:', has4Stops ? '✓ PASS' : '✗ FAIL');

const hasLapPath = presetSource.includes('lapPath: Point2D[]') && presetSource.includes('pathWaypoints: lapPath');
console.log('Designated Perimeter Lap Path:', hasLapPath ? '✓ PASS' : '✗ FAIL');

const hasGeoCentre = presetSource.includes('GeoCentre (Rooftop Master Hub)');
console.log('Rooftop Master GeoCentre:', hasGeoCentre ? '✓ PASS' : '✗ FAIL');

if (hasSingleBuilding && has4Stops && hasLapPath && hasGeoCentre) {
  console.log('\n✓ ALL PRESET VERIFICATIONS PASSED 100%!');
} else {
  console.error('\n✗ VERIFICATION FAILED!');
  process.exit(1);
}
