# GeoTag-K9: Tactical Real-Time Location System (RTLS) Simulator
> **High-Fidelity CAD/GIS Simulation Engine for GPS-Denied Canine Tracking**

[![TypeScript](https://img.shields.io/badge/TypeScript-5.2-blue.svg)](https://www.typescriptlang.org/)
[![React](https://img.shields.io/badge/React-18.2-purple.svg)](https://reactjs.org/)
[![Vite](https://img.shields.io/badge/Vite-5.4-yellow.svg)](https://vitejs.dev/)
[![Tailwind CSS](https://img.shields.io/badge/Tailwind-3.4-cyan.svg)](https://tailwindcss.com/)
[![License: Academic View-Only](https://img.shields.io/badge/License-Academic%20View--Only-red.svg)](LICENSE)

> **GeoTag-K9** is an interactive, mathematical CAD/GIS simulation platform designed to model and validate Real-Time Location System (RTLS) tracking for working canines (K9s) in GPS-denied environments (such as collapsed buildings, subterranean tunnels, and reinforced concrete complexes). The simulator emulates **Bluetooth Low Energy (BLE 5.4 Coded PHY)** signal propagation from Seeed Studio XIAO ESP32C6 transponders, 2D vector ray-casting obstacle attenuation, closed-form 3D multilateration, two-anchor mirror ghost reflection disambiguation, and Continuous-Discrete Extended Kalman Filtering (EKF).

---

## Table of Contents
1. [Simulator Overview and Core Objectives](#1-simulator-overview-and-core-objectives)
2. [Simulation Loop and Architectural Pipeline](#2-simulation-loop-and-architectural-pipeline)
3. [RF Channel Physics and Noise Implementation](#3-rf-channel-physics-and-noise-implementation)
   - [3.1 Log-Distance Path Loss Model](#31-log-distance-path-loss-model)
   - [3.2 2D Vector Ray-Casting Obstacle Attenuation](#32-2d-vector-ray-casting-obstacle-attenuation)
   - [3.3 Box-Muller Gaussian Noise Generation](#33-box-muller-gaussian-noise-generation)
   - [3.4 10 Hz Packet Clock Discretization Cache](#34-10-hz-packet-clock-discretization-cache)
   - [3.5 Temporal Exponential Moving Average (EMA) Smoothing](#35-temporal-exponential-moving-average-ema-smoothing)
   - [3.6 Nonlinear Log-Normal Distance Distortion](#36-nonlinear-log-normal-distance-distortion)
4. [Multilateration Solvers and Outlier Rejection](#4-multilateration-solvers-and-outlier-rejection)
   - [4.1 3D Linear Least Squares (LLS) Solver](#41-3d-linear-least-squares-lls-solver)
   - [4.2 Median Absolute Deviation (MAD) Outlier Filtering](#42-median-absolute-deviation-mad-outlier-filtering)
   - [4.3 Geometric Dilution of Precision (GDOP) Calculation](#43-geometric-dilution-of-precision-gdop-calculation)
5. [The 2-Anchor Mirror Ambiguity Engine](#5-the-2-anchor-mirror-ambiguity-engine)
   - [5.1 The Two-Circle Intersection Ambiguity](#51-the-two-circle-intersection-ambiguity)
   - [5.2 Mode 1: Quadratic MAP Optimization](#52-mode-1-quadratic-map-optimization)
   - [5.3 Mode 2: Hierarchical Spatial Gate](#53-mode-2-hierarchical-spatial-gate)
6. [Continuous-Discrete Extended Kalman Filter (EKF)](#6-continuous-discrete-extended-kalman-filter-ekf)
7. [User Interface and CAD/GIS Instrumentation](#7-user-interface-and-cadgis-instrumentation)
   - [7.1 Locked 290px Telemetry HUD](#71-locked-290px-telemetry-hud)
   - [7.2 Campus Canvas and Technical CAD Symbols](#72-campus-canvas-and-technical-cad-symbols)
   - [7.3 Interactive Drawing and Configuration Tools](#73-interactive-drawing-and-configuration-tools)
8. [Quickstart and Development](#8-quickstart-and-development)
9. [Automated Verification and Benchmark Suite](#9-automated-verification-and-benchmark-suite)
10. [Codebase Architecture and Directory Map](#10-codebase-architecture-and-directory-map)
11. [License and Legal Terms](#12-license-and-legal-terms)

---

## 1. Simulator Overview and Core Objectives

Working search-and-rescue and tactical canines operate in GPS-denied buildings with high velocities (0 to 35 km/h in bursts) and sharp pivot turns. When structural blast walls block signals, the number of line-of-sight anchors frequently drops to two, creating severe mathematical mirror reflections.

The **GeoTag-K9 Simulator** provides an environment to study, benchmark, and visualize:
- **Physical RF Shadowing**: Real-time vector ray-intersection tests through complex architectural CAD blueprints.
- **Sparse Constellation Solvers**: Seamless degradation and recovery when moving between 6 anchors, 3 anchors, and 2 anchors.
- **Sensor Fusion Resiliency**: How dorsal electronic compass heading constraints resolve geometric ghost reflections.
- **Kinematic Filtering**: How continuous state extrapolation bridges multi-second line-of-sight signal outages.

```text
┌────────────────────────────────────────────────────────────────────────────────┐
│                           SIMULATOR CAPABILITIES                               │
├───────────────────────┬────────────────────────────────────────────────────────┤
│ Rendering Engine      │ 60 FPS HTML5 Canvas with smooth sub-pixel interpolation│
│ Physics Rate          │ Discrete 10 Hz RF packet clock (100 ms micro-bursts)   │
│ RF Propagation        │ Log-Distance Path Loss with vector polygon ray-casting │
│ Noise Engine          │ Box-Muller Gaussian + 10 Hz cache + EMA temporal filter│
│ Solvers Implemented   │ 3D Linear Least Squares (LLS), Quadratic MAP, Gate, EKF│
│ Visual Architecture   │ Gruvbox Light technical parchment CAD aesthetic        │
│ Layout Stability      │ Rigid 290px telemetry HUD with tabular-num formatting  │
└───────────────────────┴────────────────────────────────────────────────────────┘
```

---

## 2. Simulation Loop and Architectural Pipeline

The simulator decouples high-frequency visual rendering from physical radio packet events:

```text
[60 FPS RequestAnimationFrame Loop]
  │
  ├─► 1. Interpolate Ground-Truth Canine Position (Bruno 🐶)
  │      - Follows waypoint trajectory with burst acceleration (up to 35 km/h)
  │      - Computes instantaneous forward heading vector (0° to 359°)
  │
  ├─► 2. Execute RF Propagation Engine (rfPropagation.ts)
  │      - Casts rays from canine to each deployed anchor
  │      - Computes line-segment intersections with Admin Block walls
  │      - Evaluates 10 Hz packet noise cache (Box-Muller + EMA smoothing)
  │      - Applies -105.5 dBm receiver sensitivity horizon gating
  │
  ├─► 3. Anchor Constellation Gating & Solvers (multilateration.ts)
  │      - If Visible Anchors >= 3:
  │          • Linear Least Squares (LLS) 3D Multilateration
  │          • Median Absolute Deviation (MAD) outlier filtering
  │          • Geometric Dilution of Precision (GDOP) matrix computation
  │      - If Visible Anchors == 2:
  │          • Compute two-circle intersection candidates (p1, p2)
  │          • Disambiguation Engine: Quadratic MAP vs. Hierarchical Spatial Gate
  │      - If Visible Anchors < 2:
  │          • Pure dead-reckoning extrapolation
  │
  ├─► 4. Continuous-Discrete Extended Kalman Filter (kalmanFilter.ts)
  │      - Predict step: updates state x = [x, y, vx, vy]^T using physical kinematics
  │      - Update step: scales measurement covariance R dynamically based on GDOP
  │
  └─► 5. Canvas CAD/GIS & Telemetry HUD Render
         - Renders locked Gruvbox Light interface, lattice towers, and beam rays
```

---

## 3. RF Channel Physics and Noise Implementation

All RF signal propagation logic is housed in [`src/engine/rfPropagation.ts`](src/engine/rfPropagation.ts).

### 3.1 Log-Distance Path Loss Model

The received signal strength indicator (RSSI) from the collar to any anchor at distance $d$ meters is calculated as:

$$\text{RSSI}(d) = A - 10 \cdot n \cdot \log_{10}(d) - L_{\text{wall}} + X_\sigma$$

Where:
- $A = -42 \text{ dBm}$: Received signal power at a reference distance of $d_0 = 1.0\text{ m}$.
- $n = 3.0$: Path loss exponent modeling outdoor campus ground with moderate scattering.
- $d = \max(0.2, d_{\text{meters}})$: Euclidean distance between collar and anchor.
- $L_{\text{wall}}$: Total attenuation in dB caused by concrete wall crossings.
- $X_\sigma$: Zero-mean Gaussian shadow fading noise (dB).

### 3.2 2D Vector Ray-Casting Obstacle Attenuation

Whenever a ray travels from the collar position $\mathbf{p}_{\text{tag}}$ to an anchor $\mathbf{p}_{\text{anchor}}$, the simulator evaluates parametric line-segment intersection tests against all polygon edges of the Admin Block building:

```typescript
// geometry.ts - Parametric 2D line-segment intersection
export function linesIntersect(p1: Point2D, p2: Point2D, p3: Point2D, p4: Point2D): boolean {
  const denom = (p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y);
  if (Math.abs(denom) < 1e-10) return false; // Parallel lines
  
  const ua = ((p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x)) / denom;
  const ub = ((p2.x - p1.x) * (p1.y - p3.y) - (p2.y - p1.y) * (p1.x - p3.x)) / denom;
  
  return (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1);
}
```

- Each concrete wall intersection adds $14.0\text{ dB}$ of attenuation ($L_{\text{wall}} = N_{\text{intersections}} \times 14\text{ dB}$).
- If the resulting RSSI drops below the receiver sensitivity horizon ($-105.5\text{ dBm}$ for BLE Coded PHY), the anchor cannot decode the packet and is unweighted.

---

### 3.3 Box-Muller Gaussian Noise Generation

To model physical thermal noise and log-normal shadow fading, the simulator implements a closed-form **Box-Muller Transform** in `randomGaussian()`:

```typescript
// rfPropagation.ts - Box-Muller transform
function randomGaussian(mean: number = 0, stdDev: number = 1): number {
  const u1 = Math.max(1e-10, Math.random());
  const u2 = Math.random();
  const z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
  return z0 * stdDev + mean;
}
```

This transforms two uniformly distributed pseudo-random numbers $u_1, u_2 \sim \mathcal{U}(0, 1)$ into an exact standard normal random variable $z_0 \sim \mathcal{N}(0, 1)$, scaled by `config.noiseStdDevDb` (default $\sigma = 0.5\text{ dB}$).

---

### 3.4 10 Hz Packet Clock Discretization Cache

If Gaussian noise were sampled at 60 FPS inside the browser's `requestAnimationFrame` loop, two severe simulation artifacts would occur:
1. The noise frequency would depend on the user's screen refresh rate (60 Hz vs 144 Hz).
2. The Kalman filter would average out unphysically fast white noise fluctuations, giving an artificially optimistic tracking accuracy.

In reality, embedded collar transponders transmit discrete RF chirps at **10 Hz (every 100 ms)**. The simulator models this using a global `packetNoiseCache`:

```typescript
// rfPropagation.ts - Packet clock discretization
interface NoiseCacheEntry {
  noiseDb: number;
  lastUpdateTime: number;
}
const packetNoiseCache = new Map<string, NoiseCacheEntry>();

const cacheKey = `${tag.id}-${stop.id}`;
const now = Date.now();
let cached = packetNoiseCache.get(cacheKey);

// Only sample new noise if 100 ms has passed since the last packet chirp
if (!cached || now - cached.lastUpdateTime > 100) {
  const freshNoise = randomGaussian(0, config.noiseStdDevDb);
  const smoothed = cached ? (cached.noiseDb * 0.35 + freshNoise * 0.65) : freshNoise;
  cached = { noiseDb: smoothed, lastUpdateTime: now };
  packetNoiseCache.set(cacheKey, cached);
}
```

---

### 3.5 Temporal Exponential Moving Average (EMA) Smoothing

In physical outdoor RF environments, multipath interference is temporally correlated over short intervals (coherence time). Rather than stepping discontinuously between packet chirps, fresh noise samples are blended with the previous packet state via an Exponential Moving Average (EMA):

$$\text{Noise}_{\text{current}} = 0.35 \cdot \text{Noise}_{\text{prev}} + 0.65 \cdot \text{Noise}_{\text{fresh}}$$

This preserves authentic short-term fading correlation while accurately representing continuous physical RF jitter.

---

### 3.6 Nonlinear Log-Normal Distance Distortion

When an anchor receives a noisy RSSI reading, the solver estimates distance by inverting the path loss formula:

$$\hat{d} = 10^{\frac{A - \text{RSSI}}{10 \cdot n}}$$

Because distance is an exponential function of RSSI, symmetric Gaussian noise $X_\sigma \sim \mathcal{N}(0, \sigma^2)$ in the decibel domain converts into an **asymmetric, right-skewed log-normal error distribution** in meters. The simulator naturally reproduces the phenomenon where distance estimates overestimate rather than underestimate when signal drops.

---

## 4. Multilateration Solvers and Outlier Rejection

All constellation solvers are implemented in [`src/engine/multilateration.ts`](src/engine/multilateration.ts).

### 4.1 3-Anchor Linear Least Squares (LLS) Solver

Given $N \ge 3$ anchors at coordinates $(x_i, y_i)$ with measured ranges $r_i$, each circle equation is:

$$(x - x_i)^2 + (y - y_i)^2 = r_i^2$$

Subtracting the equation of a reference anchor $N$ eliminates the nonlinear quadratic terms $x^2 + y^2$, yielding a linear system $\mathbf{A}\mathbf{x} = \mathbf{b}$:

$$\mathbf{A} = 2 \begin{bmatrix} x_1 - x_N & y_1 - y_N \\ x_2 - x_N & y_2 - y_N \\ \vdots & \vdots \\ x_{N-1} - x_N & y_{N-1} - y_N \end{bmatrix}, \quad \mathbf{b} = \begin{bmatrix} (r_N^2 - r_1^2) + (x_1^2 + y_1^2) - (x_N^2 + y_N^2) \\ \vdots \\ (r_N^2 - r_{N-1}^2) + (x_{N-1}^2 + y_{N-1}^2) - (x_N^2 + y_N^2) \end{bmatrix}$$

The optimal 2D coordinates are solved via the Moore-Penrose pseudo-inverse:

$$\hat{\mathbf{x}} = (\mathbf{A}^T \mathbf{A})^{-1} \mathbf{A}^T \mathbf{b}$$

This computes in **$< 0.05\text{ ms}$**, enabling real-time edge execution.

### 4.2 Median Absolute Deviation (MAD) Outlier Filtering

When signals reflect off concrete structures, multipath delays produce positive ranging outliers. The simulator implements robust statistical outlier rejection using Median Absolute Deviation:

$$\text{MAD} = \text{median}\Big(\big|r_i - \text{median}(\mathbf{r})\big|\Big)$$

Measurements with residual errors exceeding $2.5 \times \text{MAD}$ are dynamically pruned from the active solver matrix before least-squares computation.

### 4.3 Geometric Dilution of Precision (GDOP) Calculation

GDOP quantifies the geometric vulnerability of the anchor constellation. The geometry matrix $\mathbf{H}$ contains unit line-of-sight direction vectors:

$$\mathbf{H} = \begin{bmatrix} \frac{x - x_1}{d_1} & \frac{y - y_1}{d_1} \\ \vdots & \vdots \\ \frac{x - x_N}{d_N} & \frac{y - y_N}{d_N} \end{bmatrix}, \quad \text{GDOP} = \sqrt{\text{Trace}\big((\mathbf{H}^T \mathbf{H})^{-1}\big)}$$

- $\text{GDOP} < 2.0$: Optimal geometry (wide angular spread).
- $\text{GDOP} > 5.0$: Poor collinear geometry (high position uncertainty).
- The simulator uses GDOP in real time to adapt the Kalman filter measurement covariance.

---

## 5. The 2-Anchor Mirror Ambiguity Engine

When building shadows reduce visibility to only two anchors, the multilateration matrix becomes underdetermined.

### 5.1 The Two-Circle Intersection Ambiguity

Two circles of radii $r_1, r_2$ centered at anchors $\mathbf{a}_1, \mathbf{a}_2$ separated by distance $d_b = \|\mathbf{a}_2 - \mathbf{a}_1\|$ intersect at **two distinct symmetric points**:

$$\mathbf{p}_{1,2} = \mathbf{a}_1 + \frac{a}{d_b}(\mathbf{a}_2 - \mathbf{a}_1) \pm \frac{h}{d_b}\begin{bmatrix} -(y_2 - y_1) \\ (x_2 - x_1) \end{bmatrix}$$

Where $a = \frac{r_1^2 - r_2^2 + d_b^2}{2 d_b}$ and $h = \sqrt{r_1^2 - a^2}$.

Both candidates share **identical RF ranges**. In commercial RTLS, picking the wrong candidate causes sudden **30m–60m teleportation jumps** across the baseline chord.

```text
       Candidate 1 (Real Dog 🐶)
              ▲
              │   r1
    Anchor 1 ──────── Anchor 2  (Baseline Chord)
              │   r2
              ▼
       Candidate 2 (Mirror Ghost 👻)
```

The simulator features two selectable disambiguation algorithms:

### 5.2 Mode 1: Quadratic MAP Optimization

A joint Bayesian cost function balances kinematic extrapolation against the collar's dorsal electronic compass heading:

$$J(\mathbf{p}_i) = W_{\text{dist}} \cdot \|\mathbf{p}_i - \mathbf{p}_{\text{pred}}\|^2 + W_{\text{angle}} \cdot (1 - \cos(\theta_i - \theta_{\text{compass}}))$$

Where:
- $\mathbf{p}\_{\text{pred}} = \mathbf{p}\_{k-1} + \mathbf{v}_{k-1} \cdot \Delta t$ is the extrapolated position from kinematics.
- $\theta_i = \text{atan2}(y_i - y_{k-1}, x_i - x_{k-1})$ is the candidate displacement bearing.
- $\theta_{\text{compass}}$ is the actual dorsal heading measured by the collar's IMU.
- $W_{\text{angle}} = 350.0$: Heavily penalizes candidates that require the dog to run backwards or jump sideways.

### 5.3 Mode 2: Hierarchical Spatial Gate

Enforces signed half-plane persistence relative to the anchor chord line with a **120 ms dead-zone buffer**:

$$\text{Side}(\mathbf{p}) = \text{sgn}\big((x_2 - x_1)(y - y_1) - (y_2 - y_1)(x - x_1)\big)$$

When the dog crosses within $2.5\text{ m}$ of the baseline chord, the 120 ms buffer suppresses boundary chatter, guaranteeing continuous straight-line corridor navigation.

---

## 6. Continuous-Discrete Extended Kalman Filter (EKF)

All raw position solutions pass through a continuous-discrete Extended Kalman Filter ([`src/engine/kalmanFilter.ts`](src/engine/kalmanFilter.ts)):

### State Vector

$$\mathbf{x} = \begin{bmatrix} x & y & v_x & v_y \end{bmatrix}^T$$

### State Transition

$$\mathbf{x}_k = \mathbf{F} \mathbf{x}_{k-1} + \mathbf{w}_k, \quad \mathbf{F} = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### Dynamic Measurement Covariance (R)

To handle varying constellation geometry, the measurement covariance adapts continuously:

$$\mathbf{R}_k = \mathbf{R}_0 \cdot \max(1.0, \text{GDOP}) \cdot \alpha_{\text{anchors}}$$

- When 5+ anchors are visible with low GDOP, $\mathbf{R}_k$ shrinks, allowing the filter to track rapid canine sprints.
- When GDOP spikes or only 2 anchors are visible, $\mathbf{R}_k$ expands, shifting trust to kinematic extrapolation and preventing RF noise from disturbing the track.

---

## 7. User Interface and CAD/GIS Instrumentation

The interface is built using a **Gruvbox Light technical parchment aesthetic** (`#FBF1C7`, `#076678`, `#D65D0E`):

```text
┌───────────────────────────┬────────────────────────────────────────────────────────┐
│   LOCKED TELEMETRY HUD    │               CAD/GIS CAMPUS CANVAS                    │
│      (Width: 290px)       │                                                        │
├───────────────────────────┤   [Anchor 1]                    [Anchor 2]             │
│ • ERROR & ACCURACY        │        \                            /                  │
│   - Est Err: 2.14 m       │         \                          /                   │
│   - GDOP: 1.82 (Optimal)  │          \                        /                    │
├───────────────────────────┤           \                      /                     │
│ • PATROL BENCHMARK        │            ▼                    ▼                      │
│   - Laps: 3 completed     │         ┌──────────────────────────┐                   │
│   - Mean Err: 2.21 m      │         │   ADMIN BLOCK COMPLEX    │                   │
├───────────────────────────┤         │   (14 dB Concrete Loss)  │                   │
│ • TARGET KINEMATICS       │         │       [GeoCentre ⛯]      │                   │
│   - Actual: 4.8 m/s       │         └──────────────────────────┘                   │
│   - Est:    4.6 m/s       │                      ▲                                 │
├───────────────────────────┤                      │                                 │
│ • ANCHORS IN VIEW: 6 / 6  │                [Bruno 🐶 (90° E)]                      │
└───────────────────────────┴────────────────────────────────────────────────────────┘
```

### 7.1 Locked 290px Telemetry HUD
- The left HUD is strictly locked to `w-[290px]` (`min-width: 290px`, `max-width: 290px`) to prevent dynamic resizing when numbers change.
- Strict `tabular-nums` CSS font formatting prevents decimal digit jitter.

### 7.2 Campus Canvas and Technical CAD Symbols
- **Bruno Token (`🐶`)**: Friendly circular badge with an outer perimeter compass needle indicating current travel heading.
- **Admin Block Complex**: Authentic CAD blueprint with thick structural exterior walls producing 14 dB attenuation.
- **Central Gateway**: Placed on the Admin Block roof with a distinct server cluster icon.
- **CAD Lattice Radio Towers**: Anchors rendered as high-contrast RF beacons with line-of-sight status rays.

### 7.3 Interactive Drawing and Configuration Tools
- **Draw Patrol Waypoints**: Click on canvas to define custom canine search paths.
- **Add Concrete Obstacles**: Draw custom polygon walls to test attenuation.
- **Reposition Anchors**: Drag anchors across the campus map to analyze GDOP changes.
- **Disambiguation Toggle**: Switch between **Quadratic MAP** and **Spatial Gate** in real time.

---

## 8. Quickstart and Development

### Prerequisites
- Node.js 18.x or higher
- npm 9.x or higher

### Installation
```bash
# Clone repository
git clone https://github.com/sidkrhere/GeoTag-K9.git
cd GeoTag-K9

# Install dependencies
npm install
```

### Run Local Development Server
```bash
npm run dev
```
Open [http://localhost:5173](http://localhost:5173) in your browser.

### Production Build
```bash
npm run build
npm run preview
```

---

## 9. Automated Verification and Benchmark Suite

The repository includes a benchmark audit script ([`scratch/audit_disambiguation.ts`](scratch/audit_disambiguation.ts)) that executes 1200 continuous simulated steps around the Admin Block:

```bash
npx tsx scratch/audit_disambiguation.ts
```

### Benchmark Results (1200 Continuous Steps)

| Evaluation Metric | Quadratic MAP | Spatial Gate | Specification Goal | Pass / Fail |
| :--- | :---: | :---: | :---: | :---: |
| **Total Steps Evaluated** | 1200 steps | 1200 steps | > 1000 steps | **PASSED** |
| **2-Anchor Steps Audited** | 149 steps | 149 steps | Active coverage | **VERIFIED** |
| **Distant Choice Anomalies** | **0 anomalies** | **0 anomalies** | **0 anomalies** | **100% PASS** |
| **Average Tracking Error** | **2.61 meters** | **2.61 meters** | $< 3.50\text{ meters}$ | **EXCEEDED** |
| **Maximum Step Jump** | **4.39 meters** | **4.39 meters** | $< 5.00\text{ meters}$ | **EXCEEDED** |
| **Discontinuous Jumps > 5m** | **0 jumps** | **0 jumps** | **0 jumps** | **PERFECT** |

---

## 10. Codebase Architecture and Directory Map

```text
GeoTag-K9/
├── src/
│   ├── components/                 # React UI & CAD/GIS Canvas
│   │   ├── ConfigPanel.tsx         # RF noise & physics parameter sliders
│   │   ├── SimulationCanvas.tsx    # HTML5 Canvas 60 FPS rendering
│   │   ├── TelemetryFeed.tsx       # Real-time packet telemetry feed
│   │   ├── TelemetryHUD.tsx        # Locked 290px telemetry HUD
│   │   └── TopToolbar.tsx          # Playback & route drawing toolbar
│   ├── engine/                     # Mathematical Algorithms & Solvers
│   │   ├── geometry.ts             # 2D segment intersection & distance math
│   │   ├── kalmanFilter.ts         # Continuous-Discrete EKF kinematic state
│   │   ├── multilateration.ts      # 3D LLS, MAD filter, GDOP matrix & disambiguation
│   │   ├── presetLayouts.ts        # IIT Ropar campus & anchor coordinates
│   │   └── rfPropagation.ts        # Box-Muller noise, path loss, ray-casting
│   ├── types/
│   │   └── simulation.ts           # Type definitions for state vectors & anchors
│   ├── App.tsx                     # Main application layout & simulation loop
│   ├── index.css                   # Gruvbox Light styles & Tailwind directives
│   └── main.tsx                    # React DOM entrypoint
├── scratch/
│   └── audit_disambiguation.ts     # 1200-step automated benchmark audit script
├── scripts/
│   └── generate_pptx.mjs           # Automated PowerPoint slide deck compiler
├── HARDWARE_SPECS.md               # Proposed Hardware Requirements for Prototyping
├── PRESENTATION_DECK.md            # 12-Slide Technical Presentation Deck Blueprint
├── GeoTag_K9_Presentation.pptx     # 16:9 PowerPoint Presentation File
├── LICENSE                         # Academic View-Only & Evaluation License
└── package.json
```

---

## 11. License and Legal Terms

This software and its mathematical algorithms are released strictly under the **Academic View-Only & Evaluation License**.

- **Permitted**: You may view, read, and inspect the code and documentation solely for personal academic evaluation, peer review, grading, or non-commercial research demonstration.
- **Strictly Prohibited**: Copying, reproducing, extracting, modifying, redistributing, commercializing, sublicensing, or using any part of this codebase for training artificial intelligence / machine learning models is strictly prohibited without prior written permission from the author.

For the full legal text, see [`LICENSE`](LICENSE).
