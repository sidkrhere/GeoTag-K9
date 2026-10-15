import React, { useRef, useEffect, useState, useCallback } from 'react';
import {
  Point2D,
  GeoStop,
  GeoCentre,
  GeoTag,
  BuildingPolygon,
  SimulationConfig,
  ActiveTool,
  TrilaterationResult,
  SignalMeasurement,
  LapMetrics
} from '../types/simulation';
import { distance } from '../engine/geometry';
import { calculateSignalMeasurement } from '../engine/rfPropagation';
import { solveTrilateration, resetDogTrajectory } from '../engine/multilateration';

function getCardinalDirection(deg: number): 'N' | 'NE' | 'E' | 'SE' | 'S' | 'SW' | 'W' | 'NW' {
  const norm = ((deg % 360) + 360) % 360;
  if (norm >= 337.5 || norm < 22.5) return 'N';
  if (norm < 67.5) return 'NE';
  if (norm < 112.5) return 'E';
  if (norm < 157.5) return 'SE';
  if (norm < 202.5) return 'S';
  if (norm < 247.5) return 'SW';
  if (norm < 292.5) return 'W';
  return 'NW';
}

interface SimulationCanvasProps {
  activeTool: ActiveTool;
  setActiveTool: (tool: ActiveTool) => void;
  config: SimulationConfig;
  buildings: BuildingPolygon[];
  setBuildings: React.Dispatch<React.SetStateAction<BuildingPolygon[]>>;
  stops: GeoStop[];
  setStops: React.Dispatch<React.SetStateAction<GeoStop[]>>;
  geoCentre: GeoCentre | null;
  setGeoCentre: React.Dispatch<React.SetStateAction<GeoCentre | null>>;
  tags: GeoTag[];
  setTags: React.Dispatch<React.SetStateAction<GeoTag[]>>;
  dogPath: Point2D[];
  setDogPath: React.Dispatch<React.SetStateAction<Point2D[]>>;
  autoWalk: boolean;
  selectedEntityId: string | null;
  setSelectedEntityId: (id: string | null) => void;
  onTrilaterationUpdate: (results: TrilaterationResult[]) => void;
  onLapMetricsUpdate?: (metrics: LapMetrics) => void;
  resetKey?: number;
}

export const SimulationCanvas: React.FC<SimulationCanvasProps> = ({
  activeTool,
  setActiveTool,
  config,
  buildings,
  setBuildings,
  stops,
  setStops,
  geoCentre,
  setGeoCentre,
  tags,
  setTags,
  dogPath,
  setDogPath,
  autoWalk,
  selectedEntityId,
  setSelectedEntityId,
  onTrilaterationUpdate,
  onLapMetricsUpdate,
  resetKey
}) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  // Dragging & Interaction State
  const [draggedEntity, setDraggedEntity] = useState<{
    type: 'STOP' | 'TAG' | 'CENTRE' | 'BUILDING_VERTEX' | 'WAYPOINT';
    id: string;
    vertexIdx?: number;
    offsetX: number;
    offsetY: number;
  } | null>(null);

  // Active Building Drawing State
  const [currentBuildingVertices, setCurrentBuildingVertices] = useState<Point2D[]>([]);
  const [mousePos, setMousePos] = useState<Point2D>({ x: 0, y: 0 });

  // Animation frame reference
  const animationFrameRef = useRef<number | null>(null);
  const pulsePhaseRef = useRef<number>(0);

  // Low-pass filter / Kalman smoothing cache to eliminate 60Hz jitter
  const smoothedDualCandidatesRef = useRef<Map<string, [Point2D, Point2D]>>(new Map());
  // Unified Continuous Display Filter across 1, 2, and 3-anchor transitions
  const unifiedDisplayCoordRef = useRef<Map<string, Point2D>>(new Map());

  // Discrete Solver Packet Clock (~3 Hz / 300ms) decoupled from 60 FPS animation loop
  const lastSolverRunTimeRef = useRef<number>(0);
  const cachedNetworkStateRef = useRef<{ results: TrilaterationResult[]; allSignals: SignalMeasurement[] }>({
    results: [],
    allSignals: []
  });

  // Multi-frame progressive transition blending (400ms) across anchor count changes
  const anchorTransitionMapRef = useRef<Map<string, {
    prevAnchorCount: number;
    prevCoord: Point2D;
    transitionStartTime: number;
  }>>(new Map());

  const onLapMetricsUpdateRef = useRef(onLapMetricsUpdate);
  onLapMetricsUpdateRef.current = onLapMetricsUpdate;

  // Real-time error & lap accumulator
  const lapTrackerRef = useRef<{
    currentLapNumber: number;
    currentLapErrorSum: number;
    currentLapSamples: number;
    lastCompletedLapAvgError: number | null;
    lifetimeErrorSum: number;
    lifetimeSamples: number;
    lastEmittedTime: number;
  }>({
    currentLapNumber: 1,
    currentLapErrorSum: 0,
    currentLapSamples: 0,
    lastCompletedLapAvgError: null,
    lifetimeErrorSum: 0,
    lifetimeSamples: 0,
    lastEmittedTime: 0
  });

  // Reset lap metrics whenever preset or layout is cleared/reset
  useEffect(() => {
    lapTrackerRef.current = {
      currentLapNumber: 1,
      currentLapErrorSum: 0,
      currentLapSamples: 0,
      lastCompletedLapAvgError: null,
      lifetimeErrorSum: 0,
      lifetimeSamples: 0,
      lastEmittedTime: 0
    };
    if (onLapMetricsUpdateRef.current) {
      onLapMetricsUpdateRef.current({
        currentLapNumber: 1,
        currentLapAvgError: 0,
        lastCompletedLapAvgError: null,
        lifetimeAvgError: 0,
        currentLapSamples: 0,
        totalSamples: 0
      });
    }
  }, [resetKey, dogPath.length === 0]);

  // Compute live signals and multilateration
  const computeNetworkState = useCallback(() => {
    const stopsMap = new Map<string, GeoStop>(stops.map(s => [s.id, s]));
    const results: TrilaterationResult[] = [];
    const allSignals: SignalMeasurement[] = [];

    for (const tag of tags) {
      const tagSignals: SignalMeasurement[] = [];
      for (const stop of stops) {
        const sig = calculateSignalMeasurement(tag, stop, buildings, config);
        if (sig) {
          tagSignals.push(sig);
          allSignals.push(sig);
        }
      }

      const res = solveTrilateration(tag, tagSignals, stopsMap, config, buildings);
      results.push(res);
    }

    onTrilaterationUpdate(results);

    // Accumulate error for primary dog (Bruno) when locked
    const primaryRes = results.find(r => r.tagId === tags[0]?.id) || results[0];
    if (
      primaryRes &&
      (primaryRes.lockType === 'FULL_3D_LOCK' ||
        primaryRes.lockType === 'DUAL_PINPOINT_LOCK' ||
        primaryRes.lockType === 'DUAL_KINEMATIC_LOCK') &&
      typeof primaryRes.errorMeters === 'number' &&
      !isNaN(primaryRes.errorMeters)
    ) {
      const tracker = lapTrackerRef.current;
      tracker.currentLapErrorSum += primaryRes.errorMeters;
      tracker.currentLapSamples += 1;
      tracker.lifetimeErrorSum += primaryRes.errorMeters;
      tracker.lifetimeSamples += 1;

      const now = performance.now();
      if (now - tracker.lastEmittedTime >= 250) {
        tracker.lastEmittedTime = now;
        if (onLapMetricsUpdateRef.current) {
          onLapMetricsUpdateRef.current({
            currentLapNumber: tracker.currentLapNumber,
            currentLapAvgError: tracker.currentLapSamples > 0
              ? Math.round((tracker.currentLapErrorSum / tracker.currentLapSamples) * 100) / 100
              : 0,
            lastCompletedLapAvgError: tracker.lastCompletedLapAvgError,
            lifetimeAvgError: tracker.lifetimeSamples > 0
              ? Math.round((tracker.lifetimeErrorSum / tracker.lifetimeSamples) * 100) / 100
              : 0,
            currentLapSamples: tracker.currentLapSamples,
            totalSamples: tracker.lifetimeSamples
          });
        }
      }
    }

    return { results, allSignals };
  }, [tags, stops, buildings, config, onTrilaterationUpdate]);

  // Handle autonomous path traversal along traced waypoints
  useEffect(() => {
    if (!autoWalk) return;

    const interval = setInterval(() => {
      setTags(prevTags => prevTags.map(tag => {
        if (!tag.autoWalk || tag.imuResting) return tag;

        let newVx = 0;
        let newVy = 0;
        let nextWaypointIdx = tag.currentWaypointIndex ?? 0;
        let newX = tag.x;
        let newY = tag.y;

        // Kinematic time step: 50ms interval (dt = 0.050s)
        const dt = 0.050;
        // Bounded strictly between physical walk (0.8 m/s) and max sprint (12.0 m/s)
        const targetSpeedMps = Math.max(0.8, Math.min(12.0, config.autoWalkSpeed ?? 2.5));
        const stepPixels = targetSpeedMps * config.pixelsPerMeter * dt;

        // If a traced waypoint path exists, navigate along waypoints!
        if (dogPath.length >= 2) {
          const wpIdx = tag.currentWaypointIndex ?? 0;
          const targetWp = dogPath[wpIdx % dogPath.length];
          const dx = targetWp.x - tag.x;
          const dy = targetWp.y - tag.y;
          const dist = Math.sqrt(dx * dx + dy * dy);

          if (dist < 10) {
            // Reached waypoint! Advance to next node in loop and continue fluid motion
            const nextIdx = (wpIdx + 1) % dogPath.length;
            const nextTarget = dogPath[nextIdx];
            const ndx = nextTarget.x - tag.x;
            const ndy = nextTarget.y - tag.y;
            const ndist = Math.sqrt(ndx * ndx + ndy * ndy) || 1;
            newVx = (ndx / ndist) * stepPixels;
            newVy = (ndy / ndist) * stepPixels;
            nextWaypointIdx = nextIdx;

            // Check if dog completed a lap around the perimeter circuit!
            // When reaching Turn #1 (wpIdx === 0 && nextIdx === 1) after traversing all legs:
            if (wpIdx === 0 && nextIdx === 1 && tag.id === tags[0]?.id) {
              const tracker = lapTrackerRef.current;
              if (tracker.currentLapSamples > 15) {
                tracker.lastCompletedLapAvgError = Math.round((tracker.currentLapErrorSum / tracker.currentLapSamples) * 100) / 100;
                tracker.currentLapNumber += 1;
                tracker.currentLapErrorSum = 0;
                tracker.currentLapSamples = 0;

                if (onLapMetricsUpdateRef.current) {
                  onLapMetricsUpdateRef.current({
                    currentLapNumber: tracker.currentLapNumber,
                    currentLapAvgError: 0,
                    lastCompletedLapAvgError: tracker.lastCompletedLapAvgError,
                    lifetimeAvgError: tracker.lifetimeSamples > 0
                      ? Math.round((tracker.lifetimeErrorSum / tracker.lifetimeSamples) * 100) / 100
                      : 0,
                    currentLapSamples: 0,
                    totalSamples: tracker.lifetimeSamples
                  });
                }
              }
            }
          } else {
            newVx = (dx / dist) * stepPixels;
            newVy = (dy / dist) * stepPixels;
          }
          newX = tag.x + newVx;
          newY = tag.y + newVy;
        } else {
          // Fallback: gentle boundary bounce if no path drawn yet
          newVx = tag.vx + (Math.random() - 0.5) * 0.1;
          newVy = tag.vy + (Math.random() - 0.5) * 0.1;
          const speed = Math.sqrt(newVx * newVx + newVy * newVy);
          if (speed > stepPixels) {
            newVx = (newVx / speed) * stepPixels;
            newVy = (newVy / speed) * stepPixels;
          }

          newX = tag.x + newVx;
          newY = tag.y + newVy;

          const canvas = canvasRef.current;
          if (canvas) {
            const padding = 60;
            if (newX < padding) { newX = padding; newVx = Math.abs(newVx); }
            if (newX > canvas.width - padding) { newX = canvas.width - padding; newVx = -Math.abs(newVx); }
            if (newY < padding + 40) { newY = padding + 40; newVy = Math.abs(newVy); }
            if (newY > canvas.height - padding - 40) { newY = canvas.height - padding - 40; newVy = -Math.abs(newVy); }
          }
        }

        // Calculate dog dorsal e-compass heading angle from movement vector
        // 0° = North (-y), 90° = East (+x), 180° = South (+y), 270° = West (-x)
        let newHeading = tag.headingDeg ?? 90;
        let turnRate = 0;
        if (Math.hypot(newVx, newVy) > 0.05) {
          const targetHeading = (Math.atan2(newVx, -newVy) * (180 / Math.PI) + 360) % 360;
          const curHeading = tag.headingDeg ?? targetHeading;
          const diff = ((targetHeading - curHeading + 540) % 360) - 180;
          const maxTurnPerTick = 15; // Smooth angular interpolation
          const turn = Math.max(-maxTurnPerTick, Math.min(maxTurnPerTick, diff * 0.35));
          newHeading = (curHeading + turn + 360) % 360;
          turnRate = turn / dt;
        }

        const cardinal = getCardinalDirection(newHeading);
        const dorsalRad = (newHeading * Math.PI) / 180;
        const nowMs = Date.now();
        const stepDistPix = Math.hypot(newVx, newVy);
        const actualSpeedMps = Math.round((stepDistPix / dt / config.pixelsPerMeter) * 10) / 10;

        return {
          ...tag,
          currentWaypointIndex: nextWaypointIdx,
          x: newX,
          y: newY,
          vx: newVx,
          vy: newVy,
          speedMps: actualSpeedMps,
          headingDeg: Math.round(newHeading * 10) / 10,
          compassData: {
            headingDeg: Math.round(newHeading * 10) / 10,
            cardinal,
            pitchDeg: Math.round(Math.sin(nowMs / 140) * 1.5 * 10) / 10,
            rollDeg: Math.round(Math.cos(nowMs / 140) * 1.2 * 10) / 10,
            yawRateDegPerSec: Math.round(turnRate * 10) / 10,
            dorsalVector: {
              x: Math.round(Math.sin(dorsalRad) * 1000) / 1000,
              y: Math.round(-Math.cos(dorsalRad) * 1000) / 1000
            },
            sensorHealth: 'CALIBRATED'
          }
        };
      }));
    }, 33); // ~30 fps update

    return () => clearInterval(interval);
  }, [autoWalk, config.autoWalkSpeed, dogPath, setTags]);

  // Main 60 FPS Render Loop
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    let isMounted = true;

    const render = () => {
      if (!isMounted) return;

      pulsePhaseRef.current += 0.04;
      const pulse = Math.sin(pulsePhaseRef.current);
      const flowOffset = (pulsePhaseRef.current * 25) % 40;

      // 1. Clear Canvas & Fill Tube Dark Black
      ctx.fillStyle = '#010602';
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      // 1950s Plan Position Indicator (PPI) Scope Geometry
      const scopeCx = geoCentre ? geoCentre.x : canvas.width / 2;
      const scopeCy = geoCentre ? geoCentre.y : canvas.height / 2;
      const maxScopeR = Math.max(canvas.width, canvas.height);

      // A. Fine Coordinate Reticle Grid (Intersections with '+' pips)
      const gridSize = 40;
      ctx.strokeStyle = 'rgba(0, 255, 65, 0.04)';
      ctx.lineWidth = 1;
      for (let x = 0; x < canvas.width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
      }
      for (let y = 0; y < canvas.height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
      }

      // B. Concentric Calibrated Radar Range Rings (25m to 250m)
      const rangeIntervalsM = [25, 50, 75, 100, 125, 150, 175, 200, 250];
      for (const rM of rangeIntervalsM) {
        const rPx = rM * config.pixelsPerMeter;
        if (rPx > maxScopeR) continue;

        const isMajor = rM % 50 === 0;
        ctx.beginPath();
        ctx.arc(scopeCx, scopeCy, rPx, 0, Math.PI * 2);
        ctx.strokeStyle = isMajor ? 'rgba(0, 255, 65, 0.22)' : 'rgba(0, 255, 65, 0.08)';
        ctx.lineWidth = isMajor ? 1.5 : 1;
        if (!isMajor) {
          ctx.setLineDash([3, 5]);
        }
        ctx.stroke();
        ctx.setLineDash([]);

        // Range callout tag along East Axis
        ctx.fillStyle = 'rgba(0, 255, 65, 0.45)';
        ctx.font = '9px monospace';
        ctx.fillText(`RNG ${rM}M`, scopeCx + rPx + 4, scopeCy - 3);
      }

      // C. Cardinal Radar Crosshairs (North, South, East, West axes)
      ctx.beginPath();
      ctx.moveTo(scopeCx, 0); ctx.lineTo(scopeCx, canvas.height);
      ctx.moveTo(0, scopeCy); ctx.lineTo(canvas.width, scopeCy);
      ctx.strokeStyle = 'rgba(0, 255, 65, 0.16)';
      ctx.lineWidth = 1;
      ctx.stroke();

      // Cardinal axis tick marks every 10 meters
      const tickStepPx = 10 * config.pixelsPerMeter;
      ctx.strokeStyle = 'rgba(0, 255, 65, 0.28)';
      for (let d = tickStepPx; d < maxScopeR; d += tickStepPx) {
        // North/South ticks
        ctx.beginPath();
        ctx.moveTo(scopeCx - 3, scopeCy - d); ctx.lineTo(scopeCx + 3, scopeCy - d);
        ctx.moveTo(scopeCx - 3, scopeCy + d); ctx.lineTo(scopeCx + 3, scopeCy + d);
        // East/West ticks
        ctx.moveTo(scopeCx + d, scopeCy - 3); ctx.lineTo(scopeCx + d, scopeCy + 3);
        ctx.moveTo(scopeCx - d, scopeCy - 3); ctx.lineTo(scopeCx - d, scopeCy + 3);
        ctx.stroke();
      }

      // D. Outer Azimuth Degree Dial (000° to 350° at 10° intervals)
      const dialRadius = Math.min(canvas.width, canvas.height) * 0.44;
      if (dialRadius > 140) {
        ctx.beginPath();
        ctx.arc(scopeCx, scopeCy, dialRadius, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(0, 255, 65, 0.3)';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        ctx.font = '8.5px monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = 'rgba(0, 255, 65, 0.6)';

        for (let deg = 0; deg < 360; deg += 10) {
          const rad = ((deg - 90) * Math.PI) / 180;
          const isMajorDeg = deg % 30 === 0;
          const tickLen = isMajorDeg ? 9 : 4;
          const x1 = scopeCx + Math.cos(rad) * dialRadius;
          const y1 = scopeCy + Math.sin(rad) * dialRadius;
          const x2 = scopeCx + Math.cos(rad) * (dialRadius - tickLen);
          const y2 = scopeCy + Math.sin(rad) * (dialRadius - tickLen);

          ctx.beginPath();
          ctx.moveTo(x1, y1);
          ctx.lineTo(x2, y2);
          ctx.strokeStyle = isMajorDeg ? 'rgba(0, 255, 65, 0.6)' : 'rgba(0, 255, 65, 0.25)';
          ctx.lineWidth = isMajorDeg ? 1.5 : 1;
          ctx.stroke();

          if (isMajorDeg) {
            const labelR = dialRadius + 14;
            const lx = scopeCx + Math.cos(rad) * labelR;
            const ly = scopeCy + Math.sin(rad) * labelR;
            const degStr = deg === 0 ? '000°' : deg < 100 ? `0${deg}°` : `${deg}°`;
            ctx.fillText(degStr, lx, ly);
          }
        }
      }

      // E. Rotating Radar Sweep Beam & Phosphor Persistence Wedge
      const sweepAngle = (pulsePhaseRef.current * 0.45) % (Math.PI * 2);
      ctx.beginPath();
      ctx.moveTo(scopeCx, scopeCy);
      ctx.arc(scopeCx, scopeCy, maxScopeR, sweepAngle - 0.4, sweepAngle);
      ctx.closePath();
      const sweepGrad = ctx.createRadialGradient(scopeCx, scopeCy, 10, scopeCx, scopeCy, maxScopeR);
      sweepGrad.addColorStop(0, 'rgba(0, 255, 65, 0.12)');
      sweepGrad.addColorStop(1, 'rgba(0, 255, 65, 0.01)');
      ctx.fillStyle = sweepGrad;
      ctx.fill();

      // Sharp sweep line
      ctx.beginPath();
      ctx.moveTo(scopeCx, scopeCy);
      ctx.lineTo(scopeCx + Math.cos(sweepAngle) * maxScopeR, scopeCy + Math.sin(sweepAngle) * maxScopeR);
      ctx.strokeStyle = 'rgba(0, 255, 65, 0.85)';
      ctx.lineWidth = 1.8;
      ctx.stroke();

      // Discrete Packet Clock: Multilateration solver update at ~12.5 Hz (~80ms)
      const now = performance.now();
      const isManualInteracting = draggedEntity !== null || activeTool !== 'SELECT_MOVE';
      if (
        now - lastSolverRunTimeRef.current >= 80 ||
        cachedNetworkStateRef.current.results.length === 0 ||
        isManualInteracting
      ) {
        cachedNetworkStateRef.current = computeNetworkState();
        lastSolverRunTimeRef.current = now;
      }
      const { results, allSignals } = cachedNetworkStateRef.current;

      // 2. Render Existing Buildings (Vector Military Radar Obstacle Maps - Alert Red)
      for (const b of buildings) {
        if (b.points.length < 2) continue;

        ctx.beginPath();
        ctx.moveTo(b.points[0].x, b.points[0].y);
        for (let i = 1; i < b.points.length; i++) {
          ctx.lineTo(b.points[i].x, b.points[i].y);
        }

        if (b.closed) {
          ctx.closePath();
          // Dark red vector chassis fill
          ctx.fillStyle = 'rgba(40, 10, 8, 0.45)';
          ctx.fill();
          ctx.strokeStyle = '#ff3b30';
          ctx.lineWidth = 1.8;
          ctx.stroke();

          // 45-degree diagonal radar hatching inside the obstacle (Alert Red)
          ctx.save();
          ctx.beginPath();
          ctx.moveTo(b.points[0].x, b.points[0].y);
          for (let i = 1; i < b.points.length; i++) {
            ctx.lineTo(b.points[i].x, b.points[i].y);
          }
          ctx.closePath();
          ctx.clip();

          // Calculate bounding box for hatching
          let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
          for (const p of b.points) {
            if (p.x < minX) minX = p.x;
            if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y;
            if (p.y > maxY) maxY = p.y;
          }
          const bH = maxY - minY;
          ctx.strokeStyle = 'rgba(255, 59, 48, 0.18)';
          ctx.lineWidth = 1;
          for (let lx = minX - bH; lx < maxX + bH; lx += 14) {
            ctx.beginPath();
            ctx.moveTo(lx, minY);
            ctx.lineTo(lx + bH, maxY);
            ctx.stroke();
          }
          ctx.restore();

          // Centroid Military Stamped Label (Alert Red)
          let cx = 0, cy = 0;
          for (const p of b.points) { cx += p.x; cy += p.y; }
          cx /= b.points.length;
          cy /= b.points.length;

          ctx.fillStyle = '#ff3b30';
          ctx.font = 'bold 10px monospace';
          ctx.textAlign = 'center';
          ctx.textBaseline = 'alphabetic';
          ctx.fillText(`[${b.name.toUpperCase()}]`, cx, cy);

          ctx.fillStyle = '#ff7066';
          ctx.font = '9px monospace';
          ctx.fillText(`ATTEN: -${b.attenuationDb}dB WALL`, cx, cy + 13);
        } else {
          ctx.strokeStyle = '#ff3b30';
          ctx.setLineDash([4, 4]);
          ctx.lineWidth = 1.5;
          ctx.stroke();
          ctx.setLineDash([]);
        }

        // Square vector vertex handles
        for (let i = 0; i < b.points.length; i++) {
          const pt = b.points[i];
          ctx.fillStyle = '#ff3b30';
          ctx.fillRect(pt.x - 3, pt.y - 3, 6, 6);
          ctx.strokeStyle = '#010602';
          ctx.lineWidth = 1;
          ctx.strokeRect(pt.x - 3, pt.y - 3, 6, 6);
        }
      }

      // 3. Render Active Building Drawing (In-Progress Lasso - Alert Red)
      if (activeTool === 'DRAW_BUILDING' && currentBuildingVertices.length > 0) {
        ctx.beginPath();
        ctx.moveTo(currentBuildingVertices[0].x, currentBuildingVertices[0].y);
        for (let i = 1; i < currentBuildingVertices.length; i++) {
          ctx.lineTo(currentBuildingVertices[i].x, currentBuildingVertices[i].y);
        }
        ctx.lineTo(mousePos.x, mousePos.y);

        ctx.strokeStyle = '#ff3b30';
        ctx.setLineDash([5, 4]);
        ctx.lineWidth = 1.8;
        ctx.stroke();
        ctx.setLineDash([]);

        for (let i = 0; i < currentBuildingVertices.length; i++) {
          const p = currentBuildingVertices[i];
          ctx.fillStyle = i === 0 ? '#ff3b30' : '#ff7066';
          ctx.fillRect(p.x - 4, p.y - 4, 8, 8);
          ctx.strokeStyle = '#010602';
          ctx.strokeRect(p.x - 4, p.y - 4, 8, 8);

          if (i === 0) {
            ctx.fillStyle = '#ff3b30';
            ctx.font = 'bold 9px monospace';
            ctx.textAlign = 'center';
            ctx.fillText('[ORIGIN: CLICK TO CLOSE]', p.x, p.y - 12);
          }
        }
      }

      // 4. Render GeoStop Coverage Halos (Cyan for T1, Orange for T2)
      if (config.showCoverageHalos) {
        for (const stop of stops) {
          const radiusPixels = (stop.tier === 1 ? config.tier1RadiusMeters : config.tier2RadiusMeters) * config.pixelsPerMeter;
          const isTier1 = stop.tier === 1;

          ctx.beginPath();
          ctx.arc(stop.x, stop.y, radiusPixels, 0, Math.PI * 2);
          ctx.fillStyle = isTier1 ? 'rgba(0, 242, 254, 0.02)' : 'rgba(255, 153, 0, 0.02)';
          ctx.fill();

          ctx.strokeStyle = isTier1
            ? `rgba(0, 242, 254, ${0.24 + pulse * 0.06})`
            : `rgba(255, 153, 0, ${0.22 + pulse * 0.06})`;
          ctx.lineWidth = 1.2;
          ctx.setLineDash([5, 5]);
          ctx.stroke();
          ctx.setLineDash([]);
        }
      }

      // 5. Render Active RF Signals (Cyan for T1 LOS, Orange for T2 LOS, Red for NLOS)
      if (config.showRFRipples) {
        for (const sig of allSignals) {
          const tag = tags.find(t => t.id === sig.tagId);
          const stop = stops.find(s => s.id === sig.stopId);
          if (!tag || !stop) continue;

          ctx.beginPath();
          ctx.moveTo(tag.x, tag.y);
          ctx.lineTo(stop.x, stop.y);

          const isTier1 = stop.tier === 1;

          if (sig.isNlosZeroWeighted) {
            ctx.strokeStyle = '#ff3b30'; // Alert Red for zero-weighted wall penetration
            ctx.lineWidth = 2.2;
            ctx.setLineDash([8, 3, 2, 3]);
          } else if (sig.isOutlier) {
            ctx.strokeStyle = '#ff9900'; // Orange for statistical outlier
            ctx.lineWidth = 1.8;
            ctx.setLineDash([4, 4]);
          } else if (sig.obstructed) {
            ctx.strokeStyle = '#ff3b30'; // Red warning for wall attenuation
            ctx.lineWidth = 1.5;
            ctx.setLineDash([3, 5]);
          } else {
            ctx.strokeStyle = isTier1 ? 'rgba(0, 242, 254, 0.65)' : 'rgba(255, 153, 0, 0.65)';
            ctx.lineWidth = 1.8;
            ctx.setLineDash([6, 6]);
            ctx.lineDashOffset = -flowOffset;
          }
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.lineDashOffset = 0;

          // Mid-beam RSSI badge
          const midX = (tag.x + stop.x) / 2;
          const midY = (tag.y + stop.y) / 2;

          let badgeText = `SIG: ${sig.rssiDbm}dBm [${isTier1 ? 'T1' : 'T2'}-LOS]`;
          let badgeWidth = 100;
          let badgeColor = isTier1 ? '#00f2fe' : '#ff9900';
          let borderColor = isTier1 ? '#007780' : '#804c00';

          if (sig.isNlosZeroWeighted) {
            badgeText = `SIG: ${sig.rssiDbm}dBm [NLOS-REJ WT:0]`;
            badgeWidth = 168;
            badgeColor = '#ff3b30';
            borderColor = '#ff3b30';
          } else if (sig.isOutlier) {
            badgeText = `SIG: ${sig.rssiDbm}dBm [OUTLIER]`;
            badgeWidth = 132;
            badgeColor = '#ff9900';
            borderColor = '#ff9900';
          } else if (sig.obstructed) {
            badgeText = `SIG: ${sig.rssiDbm}dBm [WALL ATTEN]`;
            badgeWidth = 148;
            badgeColor = '#ff3b30';
            borderColor = '#ff3b30';
          }

          ctx.fillStyle = '#010803';
          ctx.fillRect(midX - badgeWidth / 2, midY - 9, badgeWidth, 16);
          ctx.strokeStyle = borderColor;
          ctx.lineWidth = 1;
          ctx.strokeRect(midX - badgeWidth / 2, midY - 9, badgeWidth, 16);

          ctx.fillStyle = badgeColor;
          ctx.font = '9px monospace';
          ctx.textAlign = 'center';
          ctx.fillText(badgeText, midX, midY + 3);
        }
      }

      // 6. Render LoRa / Wi-Fi Backhaul Beams (GeoStops -> GeoCentre in Electric Cyan)
      if (config.showBackhaulBeams && geoCentre) {
        const activeStopIds = new Set(allSignals.map(s => s.stopId));

        for (const stopId of activeStopIds) {
          const stop = stops.find(s => s.id === stopId);
          if (!stop) continue;

          ctx.beginPath();
          ctx.moveTo(stop.x, stop.y);
          ctx.lineTo(geoCentre.x, geoCentre.y);

          ctx.strokeStyle = 'rgba(0, 242, 254, 0.45)';
          ctx.lineWidth = 1.5;
          ctx.setLineDash([4, 6]);
          ctx.lineDashOffset = -flowOffset * 1.5;
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.lineDashOffset = 0;

          // Packet data pip
          const t = (pulsePhaseRef.current * 0.8) % 1;
          const px = stop.x + (geoCentre.x - stop.x) * t;
          const py = stop.y + (geoCentre.y - stop.y) * t;

          ctx.fillStyle = '#00f2fe';
          ctx.fillRect(px - 2, py - 2, 4, 4);
        }
      }

      // 7. Render GeoCentre (Command Bunker / Master Station [⊕] in Electric Cyan)
      if (geoCentre) {
        const isSelected = selectedEntityId === geoCentre.id;

        // Outer pulse ring
        ctx.beginPath();
        ctx.arc(geoCentre.x, geoCentre.y, 22 + pulse * 2.5, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(0, 242, 254, 0.35)';
        ctx.lineWidth = 1.2;
        ctx.stroke();

        // 1950s Fixed Radar Master Command Glyph: Inscribed crosshair station [⊕]
        ctx.beginPath();
        ctx.arc(geoCentre.x, geoCentre.y, 14, 0, Math.PI * 2);
        ctx.fillStyle = '#001418';
        ctx.fill();
        ctx.strokeStyle = isSelected ? '#ffffff' : '#00f2fe';
        ctx.lineWidth = isSelected ? 2.5 : 1.8;
        ctx.stroke();

        // Center crosshair lines
        ctx.beginPath();
        ctx.moveTo(geoCentre.x - 18, geoCentre.y); ctx.lineTo(geoCentre.x + 18, geoCentre.y);
        ctx.moveTo(geoCentre.x, geoCentre.y - 18); ctx.lineTo(geoCentre.x, geoCentre.y + 18);
        ctx.strokeStyle = '#00f2fe';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Corner calibration ticks [ ⌖ ]
        const cTick = 8;
        ctx.strokeStyle = '#007780';
        ctx.strokeRect(geoCentre.x - cTick, geoCentre.y - cTick, cTick * 2, cTick * 2);

        // Center pip
        ctx.fillStyle = '#00f2fe';
        ctx.fillRect(geoCentre.x - 2, geoCentre.y - 2, 4, 4);

        // Nomenclature Stamped Box
        ctx.fillStyle = '#00f2fe';
        ctx.font = 'bold 9.5px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(`[HQ-BASE // AN/FPS-52]`, geoCentre.x, geoCentre.y - 22);
      }

      // 8. Render GeoStops (Tier-1 Cyan Masts ▲ and Tier-2 Orange Relays ◈)
      for (const stop of stops) {
        const isSelected = selectedEntityId === stop.id;
        const isTier1 = stop.tier === 1;

        if (isTier1) {
          // Tier-1: High-Power Solar Radar Mast Delta [▲] - Electric Cyan
          ctx.beginPath();
          ctx.moveTo(stop.x, stop.y - 9);
          ctx.lineTo(stop.x + 8, stop.y + 7);
          ctx.lineTo(stop.x - 8, stop.y + 7);
          ctx.closePath();
          ctx.fillStyle = isSelected ? '#003840' : '#001920';
          ctx.fill();
          ctx.strokeStyle = isSelected ? '#ffffff' : '#00f2fe';
          ctx.lineWidth = isSelected ? 2.5 : 1.8;
          ctx.stroke();

          // Radiating broadcast arcs: (( ▲ )) in Cyan
          ctx.strokeStyle = '#00f2fe';
          ctx.lineWidth = 1.2;
          ctx.beginPath();
          ctx.arc(stop.x, stop.y, 14, -Math.PI * 0.75, -Math.PI * 0.25);
          ctx.stroke();
          ctx.beginPath();
          ctx.arc(stop.x, stop.y, 18, -Math.PI * 0.7, -Math.PI * 0.3);
          ctx.stroke();

          // Nomenclature tag in Cyan
          ctx.fillStyle = '#00f2fe';
          ctx.font = 'bold 9px monospace';
          ctx.textAlign = 'center';
          ctx.fillText(`[T-1: ${stop.name.toUpperCase()}]`, stop.x, stop.y - 17);
        } else {
          // Tier-2: Auxiliary Diamond Relay [◈] - Amber/Orange
          ctx.beginPath();
          ctx.moveTo(stop.x, stop.y - 8);
          ctx.lineTo(stop.x + 8, stop.y);
          ctx.lineTo(stop.x, stop.y + 8);
          ctx.lineTo(stop.x - 8, stop.y);
          ctx.closePath();
          ctx.fillStyle = isSelected ? '#402600' : '#1a1000';
          ctx.fill();
          ctx.strokeStyle = isSelected ? '#ffffff' : '#ff9900';
          ctx.lineWidth = isSelected ? 2.5 : 1.8;
          ctx.stroke();

          // Inner pip in Orange
          ctx.fillStyle = '#ff9900';
          ctx.fillRect(stop.x - 2, stop.y - 2, 4, 4);

          // Nomenclature tag in Orange
          ctx.fillStyle = '#ff9900';
          ctx.font = '9px monospace';
          ctx.textAlign = 'center';
          ctx.fillText(`[T-2: ${stop.name.toUpperCase()}]`, stop.x, stop.y - 15);
        }
      }

      // 8.5 Render Traced Doggo Waypoint Route & Turn Badges (Phosphor Green)
      if (dogPath.length > 0) {
        if (dogPath.length >= 2) {
          ctx.beginPath();
          ctx.moveTo(dogPath[0].x, dogPath[0].y);
          for (let i = 1; i < dogPath.length; i++) {
            ctx.lineTo(dogPath[i].x, dogPath[i].y);
          }
          if (dogPath.length >= 3) {
            ctx.closePath();
          }
          ctx.strokeStyle = 'rgba(0, 255, 65, 0.55)';
          ctx.lineWidth = 1.8;
          ctx.setLineDash([8, 6]);
          ctx.lineDashOffset = -flowOffset * 0.7;
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.lineDashOffset = 0;
        }

        // Rubberband line if actively tracing path
        if (activeTool === 'TRACE_PATH') {
          const lastWp = dogPath[dogPath.length - 1];
          ctx.beginPath();
          ctx.moveTo(lastWp.x, lastWp.y);
          ctx.lineTo(mousePos.x, mousePos.y);
          ctx.strokeStyle = '#00ff41';
          ctx.lineWidth = 1.5;
          ctx.setLineDash([4, 4]);
          ctx.stroke();
          ctx.setLineDash([]);
        }

        // Vector Waypoint Turn Badges: [01], [02], [03]...
        for (let i = 0; i < dogPath.length; i++) {
          const wp = dogPath[i];
          const isCurrentTarget = tags[0]?.currentWaypointIndex === i;

          if (isCurrentTarget) {
            ctx.beginPath();
            ctx.arc(wp.x, wp.y, 16 + pulse * 2.5, 0, Math.PI * 2);
            ctx.strokeStyle = '#00ff41';
            ctx.lineWidth = 1.2;
            ctx.stroke();
          }

          // Node Box
          ctx.fillStyle = isCurrentTarget ? '#003310' : '#010c04';
          ctx.fillRect(wp.x - 12, wp.y - 8, 24, 16);
          ctx.strokeStyle = isCurrentTarget ? '#00ff41' : '#00aa33';
          ctx.lineWidth = isCurrentTarget ? 2 : 1;
          ctx.strokeRect(wp.x - 12, wp.y - 8, 24, 16);

          ctx.fillStyle = '#00ff41';
          ctx.font = 'bold 9px monospace';
          ctx.textAlign = 'center';
          const wpIndexStr = i < 9 ? `0${i + 1}` : `${i + 1}`;
          ctx.fillText(`[${wpIndexStr}]`, wp.x, wp.y + 3);

          ctx.fillStyle = isCurrentTarget ? '#00ff41' : '#008826';
          ctx.font = '8px monospace';
          ctx.fillText(isCurrentTarget ? `NEXT WP` : `WP #${wpIndexStr}`, wp.x, wp.y - 12);
        }
      }

      // Cursor hint for Trace Path tool
      if (activeTool === 'TRACE_PATH') {
        ctx.fillStyle = '#010903';
        ctx.fillRect(mousePos.x + 12, mousePos.y + 12, 160, 20);
        ctx.strokeStyle = '#00ff41';
        ctx.lineWidth = 1;
        ctx.strokeRect(mousePos.x + 12, mousePos.y + 12, 160, 20);
        ctx.fillStyle = '#00ff41';
        ctx.font = '9px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(`[CLICK TO ADD WP #${dogPath.length + 1}]`, mousePos.x + 16, mousePos.y + 26);
      }

      // 9. Render Ground-Truth Target Track & Calculated Localization
      for (const tag of tags) {
        const isSelected = selectedEntityId === tag.id;
        const res = results.find(r => r.tagId === tag.id);

        // A. True Doggo Position: Vector Target Track Blip [⨀] in Electric Cyan
        if (config.showTrueDoggos) {
          const heading = tag.headingDeg ?? 90;
          const headingRad = ((heading - 90) * Math.PI) / 180;

          // Forward Sensor Cone (60° FOV) in Electric Cyan
          if (config.showHeadingVisionCone !== false) {
            const coneFovRad = (60 * Math.PI) / 180;
            const coneDist = 48;
            const halfFov = coneFovRad / 2;

            ctx.save();
            ctx.beginPath();
            ctx.moveTo(tag.x, tag.y);
            ctx.arc(tag.x, tag.y, coneDist, headingRad - halfFov, headingRad + halfFov);
            ctx.closePath();

            const coneGrad = ctx.createRadialGradient(tag.x, tag.y, 4, tag.x, tag.y, coneDist);
            coneGrad.addColorStop(0, 'rgba(0, 242, 254, 0.28)');
            coneGrad.addColorStop(0.7, 'rgba(0, 242, 254, 0.08)');
            coneGrad.addColorStop(1, 'rgba(0, 242, 254, 0.0)');
            ctx.fillStyle = coneGrad;
            ctx.fill();

            // Arc perimeter dashed rim
            ctx.beginPath();
            ctx.arc(tag.x, tag.y, coneDist, headingRad - halfFov, headingRad + halfFov);
            ctx.strokeStyle = 'rgba(0, 242, 254, 0.55)';
            ctx.lineWidth = 1.2;
            ctx.setLineDash([3, 3]);
            ctx.stroke();
            ctx.setLineDash([]);
            ctx.restore();
          }

          // Active Track Blip: Vector glyph [⨀] in Electric Cyan
          ctx.beginPath();
          ctx.arc(tag.x, tag.y, 9, 0, Math.PI * 2);
          ctx.fillStyle = '#001820';
          ctx.fill();
          ctx.strokeStyle = isSelected ? '#ffffff' : '#00f2fe';
          ctx.lineWidth = isSelected ? 2.5 : 1.8;
          ctx.stroke();

          // Center solid pip in Cyan
          ctx.beginPath();
          ctx.arc(tag.x, tag.y, 3.5, 0, Math.PI * 2);
          ctx.fillStyle = '#00f2fe';
          ctx.fill();

          // Forward Vector Needle (Dorsal Spine Collar Axis in Cyan)
          const needleLen = 20;
          const tipX = tag.x + Math.cos(headingRad) * needleLen;
          const tipY = tag.y + Math.sin(headingRad) * needleLen;

          ctx.beginPath();
          ctx.moveTo(tag.x, tag.y);
          ctx.lineTo(tipX, tipY);
          ctx.strokeStyle = '#00f2fe';
          ctx.lineWidth = 2;
          ctx.stroke();

          // Vector needle arrowhead
          const arrowHead = 5;
          const leftHeadAngle = headingRad + Math.PI * 0.82;
          const rightHeadAngle = headingRad - Math.PI * 0.82;
          ctx.beginPath();
          ctx.moveTo(tipX, tipY);
          ctx.lineTo(tipX + Math.cos(leftHeadAngle) * arrowHead, tipY + Math.sin(leftHeadAngle) * arrowHead);
          ctx.lineTo(tipX + Math.cos(rightHeadAngle) * arrowHead, tipY + Math.sin(rightHeadAngle) * arrowHead);
          ctx.closePath();
          ctx.fillStyle = '#00f2fe';
          ctx.fill();

          // Dorsal e-Compass Telemetry Block in Cyan
          const cardinal = tag.compassData?.cardinal ?? getCardinalDirection(heading);
          ctx.fillStyle = '#001218';
          ctx.fillRect(tag.x - 48, tag.y + 16, 96, 16);
          ctx.strokeStyle = '#007780';
          ctx.lineWidth = 1;
          ctx.strokeRect(tag.x - 48, tag.y + 16, 96, 16);

          ctx.font = 'bold 8.5px monospace';
          ctx.fillStyle = '#00f2fe';
          ctx.textAlign = 'center';
          ctx.fillText(`TRK: ${Math.round(heading)}° ${cardinal} ${tag.imuResting ? '[STN]' : '[RUN]'}`, tag.x, tag.y + 27);
        }

        // B. Server Calculated Localization Geometric Manifolds
        if (config.showEstimatedLocations && res && res.lockType !== 'BLIND_ZONE') {
          const currentAnchorCount = res.contributingAnchorIds.length;
          let transitionInfo = anchorTransitionMapRef.current.get(tag.id);

          let displayPos = unifiedDisplayCoordRef.current.get(tag.id);
          if (!displayPos) {
            displayPos = { x: res.estimatedX, y: res.estimatedY };
            unifiedDisplayCoordRef.current.set(tag.id, displayPos);
            anchorTransitionMapRef.current.set(tag.id, {
              prevAnchorCount: currentAnchorCount,
              prevCoord: { ...displayPos },
              transitionStartTime: 0
            });
            transitionInfo = anchorTransitionMapRef.current.get(tag.id);
          }

          if (transitionInfo && transitionInfo.prevAnchorCount !== currentAnchorCount) {
            transitionInfo.prevAnchorCount = currentAnchorCount;
            transitionInfo.prevCoord = { ...displayPos };
            transitionInfo.transitionStartTime = Date.now();
          }

          const nowMs = Date.now();
          const elapsedSec = Math.max(0, Math.min(0.20, (nowMs - (res.timestamp || nowMs)) / 1000));
          const vx = res.velocity?.x || 0;
          const vy = res.velocity?.y || 0;
          const deadReckonedX = res.estimatedX + vx * elapsedSec;
          const deadReckonedY = res.estimatedY + vy * elapsedSec;

          let targetX = deadReckonedX;
          let targetY = deadReckonedY;

          if (transitionInfo && transitionInfo.transitionStartTime > 0) {
            const elapsed = nowMs - transitionInfo.transitionStartTime;
            if (elapsed < 160) {
              const t = elapsed / 160;
              const smoothT = t * t * (3 - 2 * t);
              targetX = transitionInfo.prevCoord.x * (1 - smoothT) + deadReckonedX * smoothT;
              targetY = transitionInfo.prevCoord.y * (1 - smoothT) + deadReckonedY * smoothT;
            } else {
              transitionInfo.transitionStartTime = 0;
            }
          }

          const dx = targetX - displayPos.x;
          const dy = targetY - displayPos.y;
          const dist = Math.sqrt(dx * dx + dy * dy);

          if (dist > 0.05) {
            if (dist < 1.5) {
              displayPos.x = targetX;
              displayPos.y = targetY;
            } else {
              const step = dist * 0.85;
              displayPos.x += (dx / dist) * step;
              displayPos.y += (dy / dist) * step;
            }
          }

          // B0. HISTORICAL TRAJECTORY PHOSPHOR PERSISTENCE TAIL (Green)
          if (config.showHistoricalBreadcrumbs && res.historyTrail && res.historyTrail.length >= 2) {
            const trail = res.historyTrail;
            ctx.save();

            ctx.beginPath();
            ctx.moveTo(trail[0].x, trail[0].y);
            for (let i = 1; i < trail.length; i++) {
              ctx.lineTo(trail[i].x, trail[i].y);
            }
            ctx.strokeStyle = 'rgba(0, 255, 65, 0.35)';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([3, 4]);
            ctx.stroke();
            ctx.setLineDash([]);

            // Progressive decaying phosphor breadcrumbs
            for (let i = 0; i < trail.length; i++) {
              const fix = trail[i];
              const progress = (i + 1) / trail.length;
              const dotRadius = 1.8 + progress * 2.2;
              const alpha = 0.15 + progress * 0.8;

              // Outer glow
              ctx.beginPath();
              ctx.arc(fix.x, fix.y, dotRadius + 1.5, 0, Math.PI * 2);
              ctx.fillStyle = `rgba(0, 255, 65, ${alpha * 0.3})`;
              ctx.fill();

              // Center dot
              ctx.beginPath();
              ctx.arc(fix.x, fix.y, dotRadius, 0, Math.PI * 2);
              ctx.fillStyle = `rgba(0, 255, 65, ${alpha})`;
              ctx.fill();
            }
            ctx.restore();
          }

          // B1. 1-ANCHOR RADIAL UNCERTAINTY RING (Amber/Orange)
          if (res.lockType === 'RADIAL_RING' && res.radialRing) {
            const { center, radiusPixels, radiusMeters } = res.radialRing;

            ctx.beginPath();
            ctx.arc(center.x, center.y, radiusPixels, 0, Math.PI * 2);
            ctx.strokeStyle = `rgba(255, 153, 0, ${0.45 + pulse * 0.15})`;
            ctx.lineWidth = 1.8;
            ctx.setLineDash([5, 5]);
            ctx.stroke();
            ctx.setLineDash([]);

            const angle = -Math.PI / 4;
            const lx = center.x + Math.cos(angle) * radiusPixels;
            const ly = center.y + Math.sin(angle) * radiusPixels;

            ctx.fillStyle = '#140b00';
            ctx.fillRect(lx - 60, ly - 9, 120, 18);
            ctx.strokeStyle = '#ff9900';
            ctx.strokeRect(lx - 60, ly - 9, 120, 18);

            ctx.fillStyle = '#ff9900';
            ctx.font = '600 9px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(`[1-BCN RING: ~${radiusMeters}M]`, lx, ly + 3);
          }

          // B2. 2-ANCHOR DUAL CANDIDATE INTERSECTIONS & MIRROR OUTCASTING
          else if (
            (res.lockType === 'DUAL_PINPOINT_LOCK' ||
              res.lockType === 'DUAL_KINEMATIC_LOCK' ||
              res.lockType === 'DUAL_AMBIGUOUS_CHORD' ||
              res.lockType === 'DUAL_COLD_BOOT' ||
              res.lockType === 'DUAL_AMBIGUITY') &&
            res.dualCandidates
          ) {
            let smoothCandidates = smoothedDualCandidatesRef.current.get(tag.id);
            if (!smoothCandidates) {
              smoothCandidates = [
                { x: res.dualCandidates[0].x, y: res.dualCandidates[0].y },
                { x: res.dualCandidates[1].x, y: res.dualCandidates[1].y }
              ];
              smoothedDualCandidatesRef.current.set(tag.id, smoothCandidates);
            } else {
              smoothCandidates[0].x += (res.dualCandidates[0].x - smoothCandidates[0].x) * 0.35;
              smoothCandidates[0].y += (res.dualCandidates[0].y - smoothCandidates[0].y) * 0.35;
              smoothCandidates[1].x += (res.dualCandidates[1].x - smoothCandidates[1].x) * 0.35;
              smoothCandidates[1].y += (res.dualCandidates[1].y - smoothCandidates[1].y) * 0.35;
            }

            const [p1, p2] = smoothCandidates;
            const dualInfo = res.dualInfo;
            const isPinpoint = res.lockType === 'DUAL_PINPOINT_LOCK';

            // Dashed baseline chord (Orange for Ambiguous, Cyan for Pinpoint)
            ctx.beginPath();
            ctx.moveTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.strokeStyle = isPinpoint ? 'rgba(0, 242, 254, 0.45)' : 'rgba(255, 153, 0, 0.45)';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([4, 4]);
            ctx.stroke();
            ctx.setLineDash([]);

            const midChordX = (p1.x + p2.x) / 2;
            const midChordY = (p1.y + p2.y) / 2;
            const separationM = dualInfo ? dualInfo.separationChordMeters : Math.round((distance(p1, p2) / config.pixelsPerMeter) * 10) / 10;
            const ghostErrM = dualInfo?.candidate2ErrorMeters ?? Math.round((distance({ x: tag.x, y: tag.y }, p2) / config.pixelsPerMeter) * 10) / 10;

            const chordText = dualInfo?.isTransitionBuffering
              ? `[TRANSITION BUFFER: 120ms MATCH | CHORD: ${separationM}M]`
              : isPinpoint
                ? `[FUSED 2-BCN LOCK | CHORD: ${separationM}M | GHOST: Δ${ghostErrM}M]`
                : res.lockType === 'DUAL_KINEMATIC_LOCK'
                  ? `[CHORD: ${separationM}M | GHOST: Δ${ghostErrM}M]`
                  : `[BASELINE CROSSING: ${separationM}M]`;

            const badgeWidth = isPinpoint ? 280 : 200;
            ctx.fillStyle = '#010803';
            ctx.fillRect(midChordX - badgeWidth / 2, midChordY - 8, badgeWidth, 16);
            ctx.strokeStyle = isPinpoint ? '#00f2fe' : '#ff9900';
            ctx.strokeRect(midChordX - badgeWidth / 2, midChordY - 8, badgeWidth, 16);
            ctx.fillStyle = isPinpoint ? '#00f2fe' : '#ff9900';
            ctx.font = '8.5px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(chordText, midChordX, midChordY + 4);

            const valIdx = res.dualInfo?.validatedIndex ?? 0;
            const outIdx = res.dualInfo?.outcastIndex ?? 1;

            [p1, p2].forEach((cand, idx) => {
              const isCandidateValidated = idx === valIdx && (
                res.lockType === 'DUAL_PINPOINT_LOCK' ||
                res.lockType === 'DUAL_KINEMATIC_LOCK' ||
                res.lockType === 'DUAL_COLD_BOOT' ||
                res.lockType === 'DUAL_AMBIGUOUS_CHORD'
              );
              const isCandidateOutcast = idx === outIdx && (
                res.lockType === 'DUAL_PINPOINT_LOCK' ||
                res.lockType === 'DUAL_KINEMATIC_LOCK' ||
                res.lockType === 'DUAL_COLD_BOOT' ||
                res.lockType === 'DUAL_AMBIGUOUS_CHORD'
              );

              if (isCandidateOutcast) {
                // Outcast Mirror Ghost: Struck-through radar echo [X-GHOST-VOID] in ALERT RED
                const thisGhostErrM = Math.round((distance({ x: tag.x, y: tag.y }, cand) / config.pixelsPerMeter) * 10) / 10;
                ctx.beginPath();
                ctx.arc(cand.x, cand.y, 8, 0, Math.PI * 2);
                ctx.fillStyle = 'rgba(50, 10, 8, 0.5)';
                ctx.fill();
                ctx.strokeStyle = '#ff3b30';
                ctx.lineWidth = 1.5;
                ctx.setLineDash([3, 3]);
                ctx.stroke();
                ctx.setLineDash([]);

                // Strikethrough X in Alert Red
                ctx.beginPath();
                ctx.moveTo(cand.x - 5, cand.y - 5); ctx.lineTo(cand.x + 5, cand.y + 5);
                ctx.moveTo(cand.x + 5, cand.y - 5); ctx.lineTo(cand.x - 5, cand.y + 5);
                ctx.strokeStyle = '#ff3b30';
                ctx.lineWidth = 1.8;
                ctx.stroke();

                const outcastLabel = `[X-GHOST-VOID: Δ${thisGhostErrM}M]`;
                const outcastBadgeW = 150;
                ctx.fillStyle = '#140504';
                ctx.fillRect(cand.x - outcastBadgeW / 2, cand.y - 22, outcastBadgeW, 15);
                ctx.strokeStyle = '#ff3b30';
                ctx.strokeRect(cand.x - outcastBadgeW / 2, cand.y - 22, outcastBadgeW, 15);

                ctx.fillStyle = '#ff3b30';
                ctx.font = '8.5px monospace';
                ctx.textAlign = 'center';
                ctx.fillText(outcastLabel, cand.x, cand.y - 11);
              } else if (isCandidateValidated) {
                // Validated Pinpoint Track Gate in Cyan or Green
                const vx = displayPos.x;
                const vy = displayPos.y;
                const gateColor = isPinpoint ? '#00f2fe' : '#00ff41';

                // Dashed error line
                ctx.beginPath();
                ctx.moveTo(tag.x, tag.y);
                ctx.lineTo(vx, vy);
                ctx.strokeStyle = isPinpoint ? 'rgba(0, 242, 254, 0.7)' : 'rgba(0, 255, 65, 0.7)';
                ctx.lineWidth = 1.5;
                ctx.setLineDash([3, 3]);
                ctx.stroke();
                ctx.setLineDash([]);

                // Mid-line error badge
                const midX = (tag.x + vx) / 2;
                const midY = (tag.y + vy) / 2;
                ctx.fillStyle = '#010803';
                ctx.fillRect(midX - 25, midY - 8, 50, 15);
                ctx.strokeStyle = gateColor;
                ctx.strokeRect(midX - 25, midY - 8, 50, 15);

                ctx.fillStyle = gateColor;
                ctx.font = 'bold 9px monospace';
                ctx.textAlign = 'center';
                ctx.fillText(`Δ ${res.errorMeters}M`, midX, midY + 3);

                // Tracking Reticle Gate: 4 Corner Brackets
                const gateR = 10;
                ctx.strokeStyle = gateColor;
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(vx - gateR, vy - gateR / 2); ctx.lineTo(vx - gateR, vy - gateR); ctx.lineTo(vx - gateR / 2, vy - gateR);
                ctx.moveTo(vx + gateR / 2, vy - gateR); ctx.lineTo(vx + gateR, vy - gateR); ctx.lineTo(vx + gateR, vy - gateR / 2);
                ctx.moveTo(vx - gateR, vy + gateR / 2); ctx.lineTo(vx - gateR, vy + gateR); ctx.lineTo(vx - gateR / 2, vy + gateR);
                ctx.moveTo(vx + gateR / 2, vy + gateR); ctx.lineTo(vx + gateR, vy + gateR); ctx.lineTo(vx + gateR, vy + gateR / 2);
                ctx.stroke();

                // Center pip
                ctx.fillStyle = gateColor;
                ctx.fillRect(vx - 2, vy - 2, 4, 4);

                // Forward Motion Vector Arrow
                if (res.velocity && (Math.abs(res.velocity.x) > 0.5 || Math.abs(res.velocity.y) > 0.5)) {
                  const arrowLen = 22;
                  const vSpeed = Math.sqrt(res.velocity.x * res.velocity.x + res.velocity.y * res.velocity.y) || 1;
                  const dirX = res.velocity.x / vSpeed;
                  const dirY = res.velocity.y / vSpeed;
                  const tipX = vx + dirX * arrowLen;
                  const tipY = vy + dirY * arrowLen;

                  ctx.beginPath();
                  ctx.moveTo(vx, vy);
                  ctx.lineTo(tipX, tipY);
                  ctx.strokeStyle = gateColor;
                  ctx.lineWidth = 2;
                  ctx.stroke();

                  const headLen = 5;
                  ctx.beginPath();
                  ctx.moveTo(tipX, tipY);
                  ctx.lineTo(tipX - headLen * (dirX + dirY * 0.5), tipY - headLen * (dirY - dirX * 0.5));
                  ctx.lineTo(tipX - headLen * (dirX - dirY * 0.5), tipY - headLen * (dirY + dirX * 0.5));
                  ctx.closePath();
                  ctx.fillStyle = gateColor;
                  ctx.fill();
                }

                // Validated Label
                const valLabel = isPinpoint
                  ? `[FUSED PINPOINT // Δ ${res.errorMeters}M]`
                  : `[2-BCN LOCK // Δ ${res.errorMeters}M]`;
                const valBadgeW = isPinpoint ? 190 : 160;
                ctx.fillStyle = '#010803';
                ctx.fillRect(vx - valBadgeW / 2, vy - 24, valBadgeW, 15);
                ctx.strokeStyle = gateColor;
                ctx.strokeRect(vx - valBadgeW / 2, vy - 24, valBadgeW, 15);

                ctx.fillStyle = gateColor;
                ctx.font = 'bold 8.5px monospace';
                ctx.textAlign = 'center';
                ctx.fillText(valLabel, vx, vy - 13);
              } else {
                // Ambiguous Candidate Pip (Orange)
                ctx.beginPath();
                ctx.arc(cand.x, cand.y, 6, 0, Math.PI * 2);
                ctx.strokeStyle = '#ff9900';
                ctx.lineWidth = 1.5;
                ctx.stroke();

                ctx.fillStyle = '#ff9900';
                ctx.font = '8.5px monospace';
                ctx.textAlign = 'center';
                ctx.fillText(`? [0${idx + 1}]`, cand.x, cand.y - 10);
              }
            });
          }

          // B3. FULL 3D MULTILATERATION LOCK [ ⌖ ] (Phosphor Green)
          else if (res.lockType === 'FULL_3D_LOCK') {
            const estX = displayPos.x;
            const estY = displayPos.y;

            // Connect true to estimated with dashed error line
            ctx.beginPath();
            ctx.moveTo(tag.x, tag.y);
            ctx.lineTo(estX, estY);
            ctx.strokeStyle = 'rgba(0, 255, 65, 0.7)';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([3, 3]);
            ctx.stroke();
            ctx.setLineDash([]);

            // Error Badge
            const midX = (tag.x + estX) / 2;
            const midY = (tag.y + estY) / 2;
            ctx.fillStyle = '#010803';
            ctx.fillRect(midX - 25, midY - 8, 50, 15);
            ctx.strokeStyle = '#00ff41';
            ctx.strokeRect(midX - 25, midY - 8, 50, 15);

            ctx.fillStyle = '#00ff41';
            ctx.font = 'bold 9px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(`Δ ${res.errorMeters}M`, midX, midY + 3);

            // 1950s Tracking Gate: [ ⌖ ] in Phosphor Green
            const gateR = 12;
            ctx.strokeStyle = '#00ff41';
            ctx.lineWidth = 2;

            // 4 Corner Brackets
            ctx.beginPath();
            ctx.moveTo(estX - gateR, estY - gateR / 2); ctx.lineTo(estX - gateR, estY - gateR); ctx.lineTo(estX - gateR / 2, estY - gateR);
            ctx.moveTo(estX + gateR / 2, estY - gateR); ctx.lineTo(estX + gateR, estY - gateR); ctx.lineTo(estX + gateR, estY - gateR / 2);
            ctx.moveTo(estX - gateR, estY + gateR / 2); ctx.lineTo(estX - gateR, estY + gateR); ctx.lineTo(estX - gateR / 2, estY + gateR);
            ctx.moveTo(estX + gateR / 2, estY + gateR); ctx.lineTo(estX + gateR, estY + gateR); ctx.lineTo(estX + gateR, estY + gateR / 2);
            ctx.stroke();

            // Crosshair tick marks
            ctx.beginPath();
            ctx.moveTo(estX - 16, estY); ctx.lineTo(estX - 7, estY);
            ctx.moveTo(estX + 7, estY); ctx.lineTo(estX + 16, estY);
            ctx.moveTo(estX, estY - 16); ctx.lineTo(estX, estY - 7);
            ctx.moveTo(estX, estY + 7); ctx.lineTo(estX, estY + 16);
            ctx.stroke();

            // Center target pip
            ctx.fillStyle = '#00ff41';
            ctx.fillRect(estX - 2, estY - 2, 4, 4);

            // 3D Lock Label
            const lockLabel = res.hypothesisFlipDetected
              ? '[3D-LOCK: RE-ALIGNED]'
              : `[3D-LOCK: MULTILAT // Δ ${res.errorMeters}M]`;
            const badgeW = res.hypothesisFlipDetected ? 160 : 180;
            ctx.fillStyle = '#010803';
            ctx.fillRect(estX - badgeW / 2, estY - 26, badgeW, 16);
            ctx.strokeStyle = '#00ff41';
            ctx.strokeRect(estX - badgeW / 2, estY - 26, badgeW, 16);

            ctx.fillStyle = '#00ff41';
            ctx.font = 'bold 8.5px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(lockLabel, estX, estY - 14);
          }
        } else if (config.showEstimatedLocations && res && res.lockType === 'BLIND_ZONE' && res.borderExit) {
          // B4. BLIND ZONE: 0-ANCHOR BORDER EXIT WITH LAST KNOWN POSITION (LKP) in ALERT RED
          const lkp = res.borderExit.lastKnownPos;
          const exitSpeedMps = res.borderExit.exitSpeedMps;
          const elapsedSec = res.borderExit.elapsedSeconds;
          const headingDeg = res.borderExit.exitHeadingDeg;

          // Expanding dashed horizon ring in Alert Red
          const uncertRadius = Math.max(16, Math.min(180, elapsedSec * exitSpeedMps * config.pixelsPerMeter));
          ctx.beginPath();
          ctx.arc(lkp.x, lkp.y, uncertRadius, 0, Math.PI * 2);
          ctx.strokeStyle = `rgba(255, 59, 48, ${Math.max(0.15, 0.45 - elapsedSec * 0.03)})`;
          ctx.lineWidth = 1.5;
          ctx.setLineDash([4, 4]);
          ctx.stroke();
          ctx.setLineDash([]);

          // Pulsing LKP Border Beacon in Alert Red
          const beaconPulse = 0.5 + Math.sin(pulsePhaseRef.current * 1.5) * 0.5;
          ctx.beginPath();
          ctx.arc(lkp.x, lkp.y, 8 + beaconPulse * 4, 0, Math.PI * 2);
          ctx.strokeStyle = 'rgba(255, 59, 48, 0.5)';
          ctx.lineWidth = 1.2;
          ctx.stroke();

          ctx.beginPath();
          ctx.arc(lkp.x, lkp.y, 4, 0, Math.PI * 2);
          ctx.fillStyle = '#ff3b30';
          ctx.fill();

          // Directional Velocity Vector Arrow at Border Exit in Red
          const velLength = Math.max(28, Math.min(85, exitSpeedMps * 18));
          const exitRad = ((headingDeg - 90) * Math.PI) / 180;
          const tipX = lkp.x + Math.cos(exitRad) * velLength;
          const tipY = lkp.y + Math.sin(exitRad) * velLength;

          ctx.beginPath();
          ctx.moveTo(lkp.x, lkp.y);
          ctx.lineTo(tipX, tipY);
          ctx.strokeStyle = '#ff3b30';
          ctx.lineWidth = 2.5;
          ctx.stroke();

          const arrowHead = 6;
          const leftAngle = exitRad + Math.PI * 0.82;
          const rightAngle = exitRad - Math.PI * 0.82;
          ctx.beginPath();
          ctx.moveTo(tipX, tipY);
          ctx.lineTo(tipX + Math.cos(leftAngle) * arrowHead, tipY + Math.sin(leftAngle) * arrowHead);
          ctx.lineTo(tipX + Math.cos(rightAngle) * arrowHead, tipY + Math.sin(rightAngle) * arrowHead);
          ctx.closePath();
          ctx.fillStyle = '#ff3b30';
          ctx.fill();

          // LKP Badge in Alert Red
          ctx.fillStyle = '#140504';
          ctx.fillRect(lkp.x - 85, lkp.y - 28, 170, 18);
          ctx.strokeStyle = '#ff3b30';
          ctx.lineWidth = 1.2;
          ctx.strokeRect(lkp.x - 85, lkp.y - 28, 170, 18);

          ctx.fillStyle = '#ff3b30';
          ctx.font = 'bold 8.5px monospace';
          ctx.textAlign = 'center';
          ctx.fillText(`[CONTACT LOST: T-${elapsedSec}s | θ=${Math.round(headingDeg)}° | ${exitSpeedMps}M/S]`, lkp.x, lkp.y - 16);
        }
      }

      animationFrameRef.current = requestAnimationFrame(render);
    };

    render();

    return () => {
      isMounted = false;
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [
    buildings,
    currentBuildingVertices,
    mousePos,
    stops,
    tags,
    geoCentre,
    activeTool,
    config,
    selectedEntityId,
    computeNetworkState,
    dogPath
  ]);

  // Handle Canvas Resize
  useEffect(() => {
    const handleResize = () => {
      const canvas = canvasRef.current;
      if (!canvas) return;
      canvas.width = window.innerWidth;
      canvas.height = window.innerHeight;
    };
    handleResize();
    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  // Mouse Event Handlers
  const handleMouseDown = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;
    const clickPoint: Point2D = { x: mouseX, y: mouseY };

    // 1. TOOL: DRAW BUILDING
    if (activeTool === 'DRAW_BUILDING') {
      // Check if clicking near the start vertex to close polygon
      if (currentBuildingVertices.length >= 3) {
        const startVertex = currentBuildingVertices[0];
        if (distance(clickPoint, startVertex) < 20) {
          // Close building polygon!
          const newBuilding: BuildingPolygon = {
            id: `b-${Date.now()}`,
            name: `Building #${buildings.length + 1}`,
            closed: true,
            attenuationDb: config.wallAttenuationDb,
            points: currentBuildingVertices,
            color: 'rgba(59, 130, 246, 0.15)'
          };
          setBuildings(prev => [...prev, newBuilding]);
          setCurrentBuildingVertices([]);
          setActiveTool('SELECT_MOVE');
          return;
        }
      }

      // Otherwise, append vertex
      setCurrentBuildingVertices(prev => [...prev, clickPoint]);
      return;
    }

    // 1.5 TOOL: TRACE DOG PATH
    if (activeTool === 'TRACE_PATH') {
      setDogPath(prev => [...prev, clickPoint]);
      return;
    }

    // 2. TOOL: ADD TIER 1 POLE
    if (activeTool === 'ADD_TIER1_STOP') {
      const newStop: GeoStop = {
        id: `pole-${Date.now()}`,
        name: `Solar Pole #${stops.filter(s => s.tier === 1).length + 1}`,
        tier: 1,
        x: mouseX,
        y: mouseY,
        isOnline: true,
        batteryMv: 4100
      };
      setStops(prev => [...prev, newStop]);
      setSelectedEntityId(newStop.id);
      setActiveTool('SELECT_MOVE');
      return;
    }

    // 3. TOOL: ADD TIER 2 EATERY
    if (activeTool === 'ADD_TIER2_STOP') {
      const newStop: GeoStop = {
        id: `eatery-${Date.now()}`,
        name: `Eatery GeoStop #${stops.filter(s => s.tier === 2).length + 1}`,
        tier: 2,
        x: mouseX,
        y: mouseY,
        isOnline: true
      };
      setStops(prev => [...prev, newStop]);
      setSelectedEntityId(newStop.id);
      setActiveTool('SELECT_MOVE');
      return;
    }

    // 4. TOOL: ADD GEOCENTRE (STRICTLY 1 INSTANCE)
    if (activeTool === 'ADD_GEOCENTRE') {
      const newCentre: GeoCentre = {
        id: 'geocentre-core',
        name: 'GeoCentre Central Server',
        x: mouseX,
        y: mouseY,
        isOnline: true
      };
      setGeoCentre(newCentre);
      setSelectedEntityId(newCentre.id);
      setActiveTool('SELECT_MOVE');
      return;
    }

    // 5. TOOL: SELECT / MOVE
    // Check if clicked on a GeoTag
    for (const tag of tags) {
      if (distance(clickPoint, { x: tag.x, y: tag.y }) < 18) {
        setSelectedEntityId(tag.id);
        setDraggedEntity({
          type: 'TAG',
          id: tag.id,
          offsetX: mouseX - tag.x,
          offsetY: mouseY - tag.y
        });
        return;
      }
    }

    // Check if clicked on a Traced Waypoint node
    for (let i = 0; i < dogPath.length; i++) {
      if (distance(clickPoint, dogPath[i]) < 16) {
        setDraggedEntity({
          type: 'WAYPOINT',
          id: `wp-${i}`,
          vertexIdx: i,
          offsetX: mouseX - dogPath[i].x,
          offsetY: mouseY - dogPath[i].y
        });
        return;
      }
    }

    // Check if clicked on a GeoStop
    for (const stop of stops) {
      if (distance(clickPoint, { x: stop.x, y: stop.y }) < 18) {
        setSelectedEntityId(stop.id);
        setDraggedEntity({
          type: 'STOP',
          id: stop.id,
          offsetX: mouseX - stop.x,
          offsetY: mouseY - stop.y
        });
        return;
      }
    }

    // Check if clicked on GeoCentre
    if (geoCentre && distance(clickPoint, { x: geoCentre.x, y: geoCentre.y }) < 22) {
      setSelectedEntityId(geoCentre.id);
      setDraggedEntity({
        type: 'CENTRE',
        id: geoCentre.id,
        offsetX: mouseX - geoCentre.x,
        offsetY: mouseY - geoCentre.y
      });
      return;
    }

    // Check if clicked on a building vertex
    for (const b of buildings) {
      for (let i = 0; i < b.points.length; i++) {
        if (distance(clickPoint, b.points[i]) < 12) {
          setSelectedEntityId(b.id);
          setDraggedEntity({
            type: 'BUILDING_VERTEX',
            id: b.id,
            vertexIdx: i,
            offsetX: mouseX - b.points[i].x,
            offsetY: mouseY - b.points[i].y
          });
          return;
        }
      }
    }

    // Clicked empty canvas
    setSelectedEntityId(null);
  };

  const handleMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;
    setMousePos({ x: mouseX, y: mouseY });

    if (!draggedEntity) return;

    if (draggedEntity.type === 'TAG') {
      const newX = mouseX - draggedEntity.offsetX;
      const newY = mouseY - draggedEntity.offsetY;

      // Keep Kalman Filter and unified display coordinate synchronized with live drag
      resetDogTrajectory(draggedEntity.id, { x: newX, y: newY });
      unifiedDisplayCoordRef.current.set(draggedEntity.id, { x: newX, y: newY });

      setTags(prev => prev.map(t => {
        if (t.id !== draggedEntity.id) return t;
        const dx = newX - t.x;
        const dy = newY - t.y;
        let newHeading = t.headingDeg ?? 90;
        if (Math.hypot(dx, dy) > 1.5) {
          newHeading = (Math.atan2(dx, -dy) * (180 / Math.PI) + 360) % 360;
        }
        const cardinal = getCardinalDirection(newHeading);
        const dorsalRad = (newHeading * Math.PI) / 180;
        return {
          ...t,
          x: newX,
          y: newY,
          headingDeg: Math.round(newHeading * 10) / 10,
          compassData: {
            headingDeg: Math.round(newHeading * 10) / 10,
            cardinal,
            pitchDeg: 0,
            rollDeg: 0,
            yawRateDegPerSec: 0,
            dorsalVector: {
              x: Math.round(Math.sin(dorsalRad) * 1000) / 1000,
              y: Math.round(-Math.cos(dorsalRad) * 1000) / 1000
            },
            sensorHealth: 'CALIBRATED'
          }
        };
      }));
    } else if (draggedEntity.type === 'WAYPOINT' && draggedEntity.vertexIdx !== undefined) {
      setDogPath(prev => {
        const next = [...prev];
        next[draggedEntity.vertexIdx!] = {
          x: mouseX - draggedEntity.offsetX,
          y: mouseY - draggedEntity.offsetY
        };
        return next;
      });
    } else if (draggedEntity.type === 'STOP') {
      setStops(prev => prev.map(s =>
        s.id === draggedEntity.id
          ? { ...s, x: mouseX - draggedEntity.offsetX, y: mouseY - draggedEntity.offsetY }
          : s
      ));
    } else if (draggedEntity.type === 'CENTRE') {
      setGeoCentre(prev => prev ? {
        ...prev,
        x: mouseX - draggedEntity.offsetX,
        y: mouseY - draggedEntity.offsetY
      } : null);
    } else if (draggedEntity.type === 'BUILDING_VERTEX' && draggedEntity.vertexIdx !== undefined) {
      setBuildings(prev => prev.map(b => {
        if (b.id !== draggedEntity.id) return b;
        const newPts = [...b.points];
        newPts[draggedEntity.vertexIdx!] = {
          x: mouseX - draggedEntity.offsetX,
          y: mouseY - draggedEntity.offsetY
        };
        return { ...b, points: newPts };
      }));
    }
  };

  const handleMouseUp = () => {
    if (draggedEntity?.type === 'TAG') {
      const tag = tags.find(t => t.id === draggedEntity.id);
      if (tag) {
        // Crucial fix: Reset kinematic velocity and innovation filter on manual drag & drop
        resetDogTrajectory(tag.id, { x: tag.x, y: tag.y });
        unifiedDisplayCoordRef.current.set(tag.id, { x: tag.x, y: tag.y });
        anchorTransitionMapRef.current.delete(tag.id);
        setTags(prev => prev.map(t => t.id === tag.id ? { ...t, vx: 0, vy: 0 } : t));
      }
    }
    setDraggedEntity(null);
  };

  const handleDoubleClick = () => {
    // Double click to close building drawing
    if (activeTool === 'DRAW_BUILDING' && currentBuildingVertices.length >= 3) {
      const newBuilding: BuildingPolygon = {
        id: `b-${Date.now()}`,
        name: `Building #${buildings.length + 1}`,
        closed: true,
        attenuationDb: config.wallAttenuationDb,
        points: currentBuildingVertices,
        color: 'rgba(59, 130, 246, 0.15)'
      };
      setBuildings(prev => [...prev, newBuilding]);
      setCurrentBuildingVertices([]);
      setActiveTool('SELECT_MOVE');
    }
  };

  return (
    <div className="relative w-full h-full cursor-crosshair">
      <canvas
        ref={canvasRef}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onDoubleClick={handleDoubleClick}
        className="w-full h-full block"
      />
    </div>
  );
};
