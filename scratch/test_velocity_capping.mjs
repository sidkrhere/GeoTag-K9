// Test velocity capping on position jumps
const ppm = 2.8;
const maxSpeedMps = 8.5;
const maxPixPerSec = maxSpeedMps * ppm;

let state = [300, 300, 0, 0];
let lastPos = { x: 300, y: 300 };
const dt = 0.08;

// Simulate extreme 48-meter noise spike from Gauss-Newton
const spikeMeas = { x: 300 + 48.2 * ppm, y: 300 }; // 48.2 meters away!

console.log('Initial Pos:', lastPos);
console.log('Extreme Spike Measurement:', spikeMeas);
console.log('Distance of Spike:', ((spikeMeas.x - lastPos.x) / ppm).toFixed(1), 'meters');

// Apply position displacement bounding:
const maxDisplacementPix = maxPixPerSec * dt;
console.log(`Max allowed physical canine displacement in ${dt}s: ${(maxDisplacementPix / ppm).toFixed(2)} meters (${maxDisplacementPix.toFixed(1)} px)`);

// Attempt to jump to spikeMeas:
let dx = spikeMeas.x - lastPos.x;
let dy = spikeMeas.y - lastPos.y;
let disp = Math.sqrt(dx * dx + dy * dy);

if (disp > maxDisplacementPix) {
  const scale = maxDisplacementPix / disp;
  state[0] = lastPos.x + dx * scale;
  state[1] = lastPos.y + dy * scale;
  state[2] = (dx * scale) / dt;
  state[3] = (dy * scale) / dt;
}

const finalDistMeters = Math.sqrt((state[0] - lastPos.x)**2 + (state[1] - lastPos.y)**2) / ppm;
const impliedSpeedMps = finalDistMeters / dt;

console.log('Position after velocity bounding:', { x: state[0].toFixed(1), y: state[1].toFixed(1) });
console.log('Actual Distance Moved:', finalDistMeters.toFixed(2), 'meters');
console.log('Implied Speed:', impliedSpeedMps.toFixed(2), 'm/s');
console.log('Cap correctly enforced:', impliedSpeedMps <= maxSpeedMps + 1e-4);
