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
import { distance, isPointInPolygon } from '../engine/geometry';
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
  const lastFrameTimeRef = useRef<number>(performance.now());
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
    unifiedDisplayCoordRef.current.clear();
    smoothedDualCandidatesRef.current.clear();
    anchorTransitionMapRef.current.clear();
    cachedNetworkStateRef.current = { results: [], allSignals: [] };
    tags.forEach(t => {
      resetDogTrajectory(t.id, { x: t.x, y: t.y });
      unifiedDisplayCoordRef.current.set(t.id, { x: t.x, y: t.y });
    });

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

      const nowMs = performance.now();
      const frameDt = Math.max(0.001, Math.min(0.05, (nowMs - lastFrameTimeRef.current) / 1000));
      lastFrameTimeRef.current = nowMs;

      pulsePhaseRef.current += 0.04;
      const pulse = Math.sin(pulsePhaseRef.current);
      const flowOffset = (pulsePhaseRef.current * 25) % 40;

      // 1. Clear Canvas & Fill Gruvbox Light Paper Parchment
      ctx.fillStyle = '#fbf1c7';
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      // Fine Technical Coordinate Grid
      const gridSize = 40;
      ctx.strokeStyle = 'rgba(189, 174, 147, 0.4)';
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
          // Warm Gruvbox Sandstone fill
          ctx.fillStyle = 'rgba(235, 219, 178, 0.75)';
          ctx.fill();
          ctx.strokeStyle = '#cc241d';
          ctx.lineWidth = 1.8;
          ctx.stroke();

          // 45-degree diagonal radar hatching inside the obstacle
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
          ctx.strokeStyle = 'rgba(204, 36, 29, 0.2)';
          ctx.lineWidth = 1;
          for (let lx = minX - bH; lx < maxX + bH; lx += 14) {
            ctx.beginPath();
            ctx.moveTo(lx, minY);
            ctx.lineTo(lx + bH, maxY);
            ctx.stroke();
          }
          ctx.restore();

          // Centroid Label
          let cx = 0, cy = 0;
          for (const p of b.points) { cx += p.x; cy += p.y; }
          cx /= b.points.length;
          cy /= b.points.length;

          const hasGatewayInside = geoCentre && isPointInPolygon({ x: geoCentre.x, y: geoCentre.y }, b.points);
          const labelY = hasGatewayInside ? cy - 36 : cy;
          const subLabelY = hasGatewayInside ? cy - 23 : cy + 13;

          ctx.fillStyle = '#9d0006';
          ctx.font = 'bold 10px monospace';
          ctx.textAlign = 'center';
          ctx.textBaseline = 'alphabetic';
          ctx.fillText(b.name.toUpperCase(), cx, labelY);

          ctx.fillStyle = '#7c6f64';
          ctx.font = '9px monospace';
          ctx.fillText(`WALL LOSS: ${b.attenuationDb} dB`, cx, subLabelY);
        } else {
          ctx.strokeStyle = '#cc241d';
          ctx.setLineDash([4, 4]);
          ctx.lineWidth = 1.5;
          ctx.stroke();
          ctx.setLineDash([]);
        }

        // Square vector vertex handles
        for (let i = 0; i < b.points.length; i++) {
          const pt = b.points[i];
          ctx.fillStyle = '#cc241d';
          ctx.fillRect(pt.x - 3, pt.y - 3, 6, 6);
          ctx.strokeStyle = '#fbf1c7';
          ctx.lineWidth = 1;
          ctx.strokeRect(pt.x - 3, pt.y - 3, 6, 6);
        }
      }

      // 3. Render Active Building Drawing (In-Progress Lasso - Gruvbox Red)
      if (activeTool === 'DRAW_BUILDING' && currentBuildingVertices.length > 0) {
        ctx.beginPath();
        ctx.moveTo(currentBuildingVertices[0].x, currentBuildingVertices[0].y);
        for (let i = 1; i < currentBuildingVertices.length; i++) {
          ctx.lineTo(currentBuildingVertices[i].x, currentBuildingVertices[i].y);
        }
        ctx.lineTo(mousePos.x, mousePos.y);

        ctx.strokeStyle = '#cc241d';
        ctx.setLineDash([5, 4]);
        ctx.lineWidth = 1.8;
        ctx.stroke();
        ctx.setLineDash([]);

        for (let i = 0; i < currentBuildingVertices.length; i++) {
          const p = currentBuildingVertices[i];
          ctx.fillStyle = i === 0 ? '#cc241d' : '#fb4934';
          ctx.fillRect(p.x - 4, p.y - 4, 8, 8);
          ctx.strokeStyle = '#282828';
          ctx.strokeRect(p.x - 4, p.y - 4, 8, 8);

          if (i === 0) {
            ctx.fillStyle = '#cc241d';
            ctx.font = 'bold 9px monospace';
            ctx.textAlign = 'center';
            ctx.fillText('ORIGIN: CLICK TO CLOSE', p.x, p.y - 12);
          }
        }
      }

      // 4. Render Anchor Coverage Halos (Gruvbox Blue)
      if (config.showCoverageHalos) {
        const radiusM = config.anchorRadiusMeters ?? config.tier1RadiusMeters ?? 90;
        const radiusPixels = radiusM * config.pixelsPerMeter;
        for (const stop of stops) {
          ctx.beginPath();
          ctx.arc(stop.x, stop.y, radiusPixels, 0, Math.PI * 2);
          ctx.fillStyle = 'rgba(69, 133, 136, 0.04)';
          ctx.fill();

          ctx.strokeStyle = `rgba(69, 133, 136, ${0.35 + pulse * 0.05})`;
          ctx.lineWidth = 1.2;
          ctx.setLineDash([5, 5]);
          ctx.stroke();
          ctx.setLineDash([]);
        }
      }

      // 5. Render Active RF Signals
      if (config.showRFRipples) {
        for (const sig of allSignals) {
          const tag = tags.find(t => t.id === sig.tagId);
          const stop = stops.find(s => s.id === sig.stopId);
          if (!tag || !stop) continue;

          ctx.beginPath();
          ctx.moveTo(tag.x, tag.y);
          ctx.lineTo(stop.x, stop.y);

          if (sig.isNlosZeroWeighted) {
            ctx.strokeStyle = '#cc241d'; // Gruvbox Red for NLOS zero-weighted
            ctx.lineWidth = 2.2;
            ctx.setLineDash([8, 3, 2, 3]);
          } else if (sig.isOutlier) {
            ctx.strokeStyle = '#d65d0e'; // Gruvbox Orange for statistical outlier
            ctx.lineWidth = 1.8;
            ctx.setLineDash([4, 4]);
          } else if (sig.obstructed) {
            ctx.strokeStyle = '#cc241d'; // Red warning for wall attenuation
            ctx.lineWidth = 1.5;
            ctx.setLineDash([3, 5]);
          } else {
            ctx.strokeStyle = 'rgba(69, 133, 136, 0.65)';
            ctx.lineWidth = 1.8;
            ctx.setLineDash([6, 6]);
            ctx.lineDashOffset = -flowOffset;
          }
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.lineDashOffset = 0;
        }
      }

      // 6. Render LoRa / Backhaul Beams (GeoStops -> GeoCentre)
      if (config.showBackhaulBeams && geoCentre) {
        const activeStopIds = new Set(allSignals.map(s => s.stopId));

        for (const stopId of activeStopIds) {
          const stop = stops.find(s => s.id === stopId);
          if (!stop) continue;

          ctx.beginPath();
          ctx.moveTo(stop.x, stop.y);
          ctx.lineTo(geoCentre.x, geoCentre.y);

          ctx.strokeStyle = 'rgba(69, 133, 136, 0.45)';
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

          ctx.fillStyle = '#076678';
          ctx.fillRect(px - 2, py - 2, 4, 4);
        }
      }

      // 7. Render GeoCentre Hub (Command Base Station / Rooftop Chassis)
      if (geoCentre) {
        const isSelected = selectedEntityId === geoCentre.id;

        // Outer pulse ring
        ctx.beginPath();
        ctx.arc(geoCentre.x, geoCentre.y, 22 + pulse * 2, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(7, 102, 120, 0.3)';
        ctx.lineWidth = 1.2;
        ctx.stroke();

        // Hub station glyph (Sleek Central Gateway rooftop chassis)
        const chassisW = 22;
        const chassisH = 14;
        ctx.fillStyle = isSelected ? '#dfebeb' : '#ebf2f2';
        ctx.strokeStyle = isSelected ? '#d65d0e' : '#076678';
        ctx.lineWidth = isSelected ? 2.5 : 1.8;
        ctx.beginPath();
        if (typeof (ctx as any).roundRect === 'function') {
          (ctx as any).roundRect(geoCentre.x - chassisW / 2, geoCentre.y - chassisH / 2, chassisW, chassisH, 3);
        } else {
          ctx.rect(geoCentre.x - chassisW / 2, geoCentre.y - chassisH / 2, chassisW, chassisH);
        }
        ctx.fill();
        ctx.stroke();

        // Gateway vertical antenna mast
        ctx.beginPath();
        ctx.moveTo(geoCentre.x, geoCentre.y - chassisH / 2);
        ctx.lineTo(geoCentre.x, geoCentre.y - chassisH / 2 - 8);
        ctx.strokeStyle = isSelected ? '#d65d0e' : '#076678';
        ctx.lineWidth = 1.8;
        ctx.stroke();

        // Gateway antenna radome top pip
        ctx.beginPath();
        ctx.arc(geoCentre.x, geoCentre.y - chassisH / 2 - 8, 2.5, 0, Math.PI * 2);
        ctx.fillStyle = isSelected ? '#d65d0e' : '#076678';
        ctx.fill();

        // Hub central status indicator
        ctx.beginPath();
        ctx.arc(geoCentre.x, geoCentre.y, 2.5, 0, Math.PI * 2);
        ctx.fillStyle = isSelected ? '#d65d0e' : '#076678';
        ctx.fill();

        // Label (Positioned cleanly below the server chassis)
        ctx.fillStyle = '#076678';
        ctx.font = 'bold 9px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(`CENTRAL GATEWAY`, geoCentre.x, geoCentre.y + 22);
      }

      // 8. Render Anchors (Unified Radio Mast Tower Glyph)
      for (const stop of stops) {
        const isSelected = selectedEntityId === stop.id;
        const x = stop.x;
        const y = stop.y;

        const mastColor = isSelected ? '#d65d0e' : '#076678';

        ctx.strokeStyle = mastColor;
        ctx.lineWidth = isSelected ? 2.2 : 1.6;
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';

        // 1. Tapered Lattice Tower Legs (A-frame structure)
        ctx.beginPath();
        ctx.moveTo(x - 7, y + 8);
        ctx.lineTo(x, y - 5);
        ctx.lineTo(x + 7, y + 8);
        ctx.stroke();

        // 2. Base plate & horizontal cross-strut
        ctx.beginPath();
        ctx.moveTo(x - 8, y + 8);
        ctx.lineTo(x + 8, y + 8);
        ctx.moveTo(x - 4.5, y + 1);
        ctx.lineTo(x + 4.5, y + 1);
        ctx.stroke();

        // 3. Vertical antenna mast extending from tower apex
        ctx.beginPath();
        ctx.moveTo(x, y - 5);
        ctx.lineTo(x, y - 12);
        ctx.stroke();

        // 4. Top RF radiator emitter pip
        ctx.beginPath();
        ctx.arc(x, y - 12, 2.2, 0, Math.PI * 2);
        ctx.fillStyle = mastColor;
        ctx.fill();

        // Radiating subtle RF wave arcs above the mast
        ctx.strokeStyle = isSelected ? 'rgba(214, 93, 14, 0.6)' : 'rgba(7, 102, 120, 0.45)';
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.arc(x, y - 12, 6, -Math.PI * 0.75, -Math.PI * 0.25);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(x, y - 12, 10, -Math.PI * 0.7, -Math.PI * 0.3);
        ctx.stroke();

        // Station name label below tower
        ctx.fillStyle = mastColor;
        ctx.font = 'bold 9px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(stop.name.toUpperCase(), x, y + 20);
      }

      // 8.5 Render Traced Doggo Waypoint Route (Gruvbox Olive)
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
          ctx.strokeStyle = 'rgba(121, 116, 14, 0.65)';
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
          ctx.strokeStyle = '#79740e';
          ctx.lineWidth = 1.5;
          ctx.setLineDash([4, 4]);
          ctx.stroke();
          ctx.setLineDash([]);
        }

        // Waypoint Turn Badges
        for (let i = 0; i < dogPath.length; i++) {
          const wp = dogPath[i];
          const isCurrentTarget = tags[0]?.currentWaypointIndex === i;

          if (isCurrentTarget) {
            ctx.beginPath();
            ctx.arc(wp.x, wp.y, 16 + pulse * 2, 0, Math.PI * 2);
            ctx.strokeStyle = '#79740e';
            ctx.lineWidth = 1.2;
            ctx.stroke();
          }

          // Node Box
          ctx.fillStyle = isCurrentTarget ? '#eaedd0' : '#fbf1c7';
          ctx.fillRect(wp.x - 12, wp.y - 8, 24, 16);
          ctx.strokeStyle = isCurrentTarget ? '#79740e' : '#b8bb26';
          ctx.lineWidth = isCurrentTarget ? 2 : 1;
          ctx.strokeRect(wp.x - 12, wp.y - 8, 24, 16);

          ctx.fillStyle = '#79740e';
          ctx.font = 'bold 9px monospace';
          ctx.textAlign = 'center';
          const wpIndexStr = i < 9 ? `0${i + 1}` : `${i + 1}`;
          ctx.fillText(`[${wpIndexStr}]`, wp.x, wp.y + 3);

          ctx.fillStyle = isCurrentTarget ? '#79740e' : '#7c6f64';
          ctx.font = '8px monospace';
          ctx.fillText(isCurrentTarget ? `NEXT WP` : `WP #${wpIndexStr}`, wp.x, wp.y - 12);
        }
      }

      // Cursor hint for Trace Path tool
      if (activeTool === 'TRACE_PATH') {
        ctx.fillStyle = '#fbf1c7';
        ctx.fillRect(mousePos.x + 12, mousePos.y + 12, 160, 20);
        ctx.strokeStyle = '#79740e';
        ctx.lineWidth = 1;
        ctx.strokeRect(mousePos.x + 12, mousePos.y + 12, 160, 20);
        ctx.fillStyle = '#79740e';
        ctx.font = 'bold 9px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(`+ ADD WAYPOINT #${dogPath.length + 1}`, mousePos.x + 16, mousePos.y + 26);
      }

      // 9. Render Ground-Truth Target Track & Calculated Localization
      for (const tag of tags) {
        const isSelected = selectedEntityId === tag.id;
        const res = results.find(r => r.tagId === tag.id);

        // A. True Doggo Position
        if (config.showTrueDoggos) {
          const heading = tag.headingDeg ?? 90;
          const headingRad = ((heading - 90) * Math.PI) / 180;

          // Forward Sensor Cone (60° FOV)
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
            coneGrad.addColorStop(0, 'rgba(69, 133, 136, 0.22)');
            coneGrad.addColorStop(0.7, 'rgba(69, 133, 136, 0.06)');
            coneGrad.addColorStop(1, 'rgba(69, 133, 136, 0.0)');
            ctx.fillStyle = coneGrad;
            ctx.fill();

            // Arc perimeter dashed rim
            ctx.beginPath();
            ctx.arc(tag.x, tag.y, coneDist, headingRad - halfFov, headingRad + halfFov);
            ctx.strokeStyle = 'rgba(69, 133, 136, 0.45)';
            ctx.lineWidth = 1.2;
            ctx.setLineDash([3, 3]);
            ctx.stroke();
            ctx.setLineDash([]);
            ctx.restore();
          }

          // Active Dog Track: Friendly Emoji Doggo Symbol 🐶
          ctx.save();
          // Circular paper badge
          ctx.beginPath();
          ctx.arc(tag.x, tag.y, 13, 0, Math.PI * 2);
          ctx.fillStyle = '#fbf1c7';
          ctx.fill();
          ctx.strokeStyle = isSelected ? '#d65d0e' : '#076678';
          ctx.lineWidth = isSelected ? 2.2 : 1.6;
          ctx.stroke();

          // Render Doggo Emoji 🐶
          ctx.font = '16px "Apple Color Emoji", "Segoe UI Emoji", "Noto Color Emoji", sans-serif';
          ctx.textAlign = 'center';
          ctx.textBaseline = 'middle';
          ctx.fillText('🐶', tag.x, tag.y + 0.5);
          ctx.restore();

          // Directional Heading Indicator Pip on the perimeter
          const needleDist = 13;
          const tipX = tag.x + Math.cos(headingRad) * needleDist;
          const tipY = tag.y + Math.sin(headingRad) * needleDist;
          ctx.beginPath();
          ctx.arc(tipX, tipY, 2.5, 0, Math.PI * 2);
          ctx.fillStyle = isSelected ? '#d65d0e' : '#076678';
          ctx.fill();

          // Forward Vector Needle pointing forward
          const extNeedleLen = 22;
          const extTipX = tag.x + Math.cos(headingRad) * extNeedleLen;
          const extTipY = tag.y + Math.sin(headingRad) * extNeedleLen;
          ctx.beginPath();
          ctx.moveTo(tipX, tipY);
          ctx.lineTo(extTipX, extTipY);
          ctx.strokeStyle = isSelected ? '#d65d0e' : '#076678';
          ctx.lineWidth = 2;
          ctx.stroke();

          // Vector needle arrowhead
          const arrowHead = 5;
          const leftHeadAngle = headingRad + Math.PI * 0.82;
          const rightHeadAngle = headingRad - Math.PI * 0.82;
          ctx.beginPath();
          ctx.moveTo(extTipX, extTipY);
          ctx.lineTo(extTipX + Math.cos(leftHeadAngle) * arrowHead, extTipY + Math.sin(leftHeadAngle) * arrowHead);
          ctx.lineTo(extTipX + Math.cos(rightHeadAngle) * arrowHead, extTipY + Math.sin(rightHeadAngle) * arrowHead);
          ctx.closePath();
          ctx.fillStyle = isSelected ? '#d65d0e' : '#076678';
          ctx.fill();

          // Dorsal e-Compass Telemetry Block (Clean name without TRK-01)
          const cardinal = tag.compassData?.cardinal ?? getCardinalDirection(heading);
          const cleanName = tag.name.replace(/^TRK-\d+\s*\((.+)\)$/, '$1').replace(/^TRK-\d+\s*/, '');
          const k9Label = `${cleanName} • ${Math.round(heading)}° ${cardinal}${tag.imuResting ? ' (Rest)' : ''}`;
          ctx.font = 'bold 8.5px monospace';
          const textW = ctx.measureText(k9Label).width;
          const badgeW = Math.max(76, textW + 14);
          ctx.fillStyle = '#fbf1c7';
          ctx.fillRect(tag.x - badgeW / 2, tag.y + 17, badgeW, 16);
          ctx.strokeStyle = '#d5c4a1';
          ctx.lineWidth = 1;
          ctx.strokeRect(tag.x - badgeW / 2, tag.y + 17, badgeW, 16);

          ctx.fillStyle = '#282828';
          ctx.textAlign = 'center';
          ctx.textBaseline = 'alphabetic';
          ctx.fillText(k9Label, tag.x, tag.y + 28);
        }

        // B. Server Calculated Localization Geometric Manifolds
        if (config.showEstimatedLocations && res && res.lockType !== 'BLIND_ZONE') {
          const currentAnchorCount = res.contributingAnchorIds.length;
          let transitionInfo = anchorTransitionMapRef.current.get(tag.id);

          let displayPos = unifiedDisplayCoordRef.current.get(tag.id);
          if (!displayPos) {
            displayPos = { x: tag.x, y: tag.y };
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

          let targetX = res.estimatedX;
          let targetY = res.estimatedY;

          if (transitionInfo && transitionInfo.transitionStartTime > 0) {
            const elapsed = Date.now() - transitionInfo.transitionStartTime;
            if (elapsed < 140) {
              const t = elapsed / 140;
              const smoothT = t * t * (3 - 2 * t);
              targetX = transitionInfo.prevCoord.x * (1 - smoothT) + res.estimatedX * smoothT;
              targetY = transitionInfo.prevCoord.y * (1 - smoothT) + res.estimatedY * smoothT;
            } else {
              transitionInfo.transitionStartTime = 0;
            }
          }

          // Continuous Exponential Moving Average (EMA) low-pass filter
          // Eradicates discrete 12.5Hz packet jitter while maintaining instantaneous agility (<30ms lag)
          const tau = 0.065; // 65ms relaxation time constant
          const alpha = 1.0 - Math.exp(-frameDt / tau);
          displayPos.x += (targetX - displayPos.x) * alpha;
          displayPos.y += (targetY - displayPos.y) * alpha;

          // B0. HISTORICAL PATROL BREADCRUMBS (Gruvbox Olive)
          if (config.showHistoricalBreadcrumbs && res.historyTrail && res.historyTrail.length >= 2) {
            const trail = res.historyTrail;
            ctx.save();

            ctx.beginPath();
            ctx.moveTo(trail[0].x, trail[0].y);
            for (let i = 1; i < trail.length; i++) {
              ctx.lineTo(trail[i].x, trail[i].y);
            }
            ctx.strokeStyle = 'rgba(121, 116, 14, 0.4)';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([3, 4]);
            ctx.stroke();
            ctx.setLineDash([]);

            // Decaying breadcrumbs
            for (let i = 0; i < trail.length; i++) {
              const fix = trail[i];
              const progress = (i + 1) / trail.length;
              const dotRadius = 1.8 + progress * 2.2;
              const alpha = 0.2 + progress * 0.7;

              ctx.beginPath();
              ctx.arc(fix.x, fix.y, dotRadius, 0, Math.PI * 2);
              ctx.fillStyle = `rgba(121, 116, 14, ${alpha})`;
              ctx.fill();
            }
            ctx.restore();
          }

          // B1. 1-ANCHOR RADIAL UNCERTAINTY RING (Gruvbox Amber)
          if (res.lockType === 'RADIAL_RING' && res.radialRing) {
            const { center, radiusPixels } = res.radialRing;

            ctx.beginPath();
            ctx.arc(center.x, center.y, radiusPixels, 0, Math.PI * 2);
            ctx.strokeStyle = `rgba(215, 153, 33, ${0.45 + pulse * 0.15})`;
            ctx.lineWidth = 1.8;
            ctx.setLineDash([5, 5]);
            ctx.stroke();
            ctx.setLineDash([]);
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
            const isPinpoint = res.lockType === 'DUAL_PINPOINT_LOCK';

            // Dashed baseline chord (Only rendered when showGhostCandidates is TRUE)
            if (config.showGhostCandidates) {
              ctx.beginPath();
              ctx.moveTo(p1.x, p1.y);
              ctx.lineTo(p2.x, p2.y);
              ctx.strokeStyle = isPinpoint ? 'rgba(7, 102, 120, 0.45)' : 'rgba(214, 93, 14, 0.45)';
              ctx.lineWidth = 1.5;
              ctx.setLineDash([4, 4]);
              ctx.stroke();
              ctx.setLineDash([]);
            }

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

              if (isCandidateOutcast && config.showGhostCandidates) {
                // Outcast Mirror Ghost: Struck-through echo [GHOST-VOID] in Alert Red
                ctx.beginPath();
                ctx.arc(cand.x, cand.y, 8, 0, Math.PI * 2);
                ctx.fillStyle = 'rgba(250, 230, 228, 0.6)';
                ctx.fill();
                ctx.strokeStyle = '#cc241d';
                ctx.lineWidth = 1.5;
                ctx.setLineDash([3, 3]);
                ctx.stroke();
                ctx.setLineDash([]);

                // Strikethrough X in Alert Red
                ctx.beginPath();
                ctx.moveTo(cand.x - 5, cand.y - 5); ctx.lineTo(cand.x + 5, cand.y + 5);
                ctx.moveTo(cand.x + 5, cand.y - 5); ctx.lineTo(cand.x - 5, cand.y + 5);
                ctx.strokeStyle = '#cc241d';
                ctx.lineWidth = 1.8;
                ctx.stroke();
              } else if (isCandidateValidated) {
                // Validated Candidate Reticle in Vibrant Gruvbox Orange
                const vx = displayPos.x;
                const vy = displayPos.y;
                const reticleColor = '#d65d0e'; // GRUVBOX ORANGE

                // Dashed error line
                ctx.beginPath();
                ctx.moveTo(tag.x, tag.y);
                ctx.lineTo(vx, vy);
                ctx.strokeStyle = 'rgba(214, 93, 14, 0.7)';
                ctx.lineWidth = 1.5;
                ctx.setLineDash([3, 3]);
                ctx.stroke();
                ctx.setLineDash([]);

                // Tracking Reticle Gate: 4 Corner Brackets
                const gateR = 10;
                ctx.strokeStyle = reticleColor;
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(vx - gateR, vy - gateR / 2); ctx.lineTo(vx - gateR, vy - gateR); ctx.lineTo(vx - gateR / 2, vy - gateR);
                ctx.moveTo(vx + gateR / 2, vy - gateR); ctx.lineTo(vx + gateR, vy - gateR); ctx.lineTo(vx + gateR, vy - gateR / 2);
                ctx.moveTo(vx - gateR, vy + gateR / 2); ctx.lineTo(vx - gateR, vy + gateR); ctx.lineTo(vx - gateR / 2, vy + gateR);
                ctx.moveTo(vx + gateR / 2, vy + gateR); ctx.lineTo(vx + gateR, vy + gateR); ctx.lineTo(vx + gateR, vy + gateR / 2);
                ctx.stroke();

                // Center pip
                ctx.fillStyle = reticleColor;
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
                  ctx.strokeStyle = reticleColor;
                  ctx.lineWidth = 2;
                  ctx.stroke();

                  const headLen = 5;
                  ctx.beginPath();
                  ctx.moveTo(tipX, tipY);
                  ctx.lineTo(tipX - headLen * (dirX + dirY * 0.5), tipY - headLen * (dirY - dirX * 0.5));
                  ctx.lineTo(tipX - headLen * (dirX - dirY * 0.5), tipY - headLen * (dirY + dirX * 0.5));
                  ctx.closePath();
                  ctx.fillStyle = reticleColor;
                  ctx.fill();
                }
              } else if (config.showGhostCandidates) {
                // Ambiguous Candidate Pip (Orange) only shown when ghost candidates are enabled
                ctx.beginPath();
                ctx.arc(cand.x, cand.y, 6, 0, Math.PI * 2);
                ctx.strokeStyle = '#d65d0e';
                ctx.lineWidth = 1.5;
                ctx.stroke();
              }
            });
          }

          // B3. FULL 3D MULTILATERATION LOCK [ ⌖ ] in Vibrant Gruvbox Orange
          else if (res.lockType === 'FULL_3D_LOCK') {
            const estX = displayPos.x;
            const estY = displayPos.y;
            const reticleColor = '#d65d0e'; // GRUVBOX ORANGE

            // Connect true to estimated with dashed error line in Orange
            ctx.beginPath();
            ctx.moveTo(tag.x, tag.y);
            ctx.lineTo(estX, estY);
            ctx.strokeStyle = 'rgba(214, 93, 14, 0.7)';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([3, 3]);
            ctx.stroke();
            ctx.setLineDash([]);

            // Tracking Reticle Gate: [ ⌖ ] in Gruvbox Orange
            const gateR = 12;
            ctx.strokeStyle = reticleColor;
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
            ctx.fillStyle = reticleColor;
            ctx.fillRect(estX - 2, estY - 2, 4, 4);
          }
        } else if (config.showEstimatedLocations && res && res.lockType === 'BLIND_ZONE' && res.borderExit) {
          // B4. BLIND ZONE: 0-ANCHOR BORDER EXIT WITH LAST KNOWN POSITION (LKP) in Gruvbox Red
          const lkp = res.borderExit.lastKnownPos;
          const exitSpeedMps = res.borderExit.exitSpeedMps;
          const elapsedSec = res.borderExit.elapsedSeconds;
          const headingDeg = res.borderExit.exitHeadingDeg;

          // Expanding dashed horizon ring in Gruvbox Red
          const uncertRadius = Math.max(16, Math.min(180, elapsedSec * exitSpeedMps * config.pixelsPerMeter));
          ctx.beginPath();
          ctx.arc(lkp.x, lkp.y, uncertRadius, 0, Math.PI * 2);
          ctx.strokeStyle = `rgba(204, 36, 29, ${Math.max(0.15, 0.45 - elapsedSec * 0.03)})`;
          ctx.lineWidth = 1.5;
          ctx.setLineDash([4, 4]);
          ctx.stroke();
          ctx.setLineDash([]);

          // Pulsing LKP Border Beacon in Gruvbox Red
          const beaconPulse = 0.5 + Math.sin(pulsePhaseRef.current * 1.5) * 0.5;
          ctx.beginPath();
          ctx.arc(lkp.x, lkp.y, 8 + beaconPulse * 4, 0, Math.PI * 2);
          ctx.strokeStyle = 'rgba(204, 36, 29, 0.5)';
          ctx.lineWidth = 1.2;
          ctx.stroke();

          ctx.beginPath();
          ctx.arc(lkp.x, lkp.y, 4, 0, Math.PI * 2);
          ctx.fillStyle = '#cc241d';
          ctx.fill();

          // Directional Velocity Vector Arrow at Border Exit in Red
          const velLength = Math.max(28, Math.min(85, exitSpeedMps * 18));
          const exitRad = ((headingDeg - 90) * Math.PI) / 180;
          const tipX = lkp.x + Math.cos(exitRad) * velLength;
          const tipY = lkp.y + Math.sin(exitRad) * velLength;

          ctx.beginPath();
          ctx.moveTo(lkp.x, lkp.y);
          ctx.lineTo(tipX, tipY);
          ctx.strokeStyle = '#cc241d';
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
          ctx.fillStyle = '#cc241d';
          ctx.fill();

          // LKP Badge in Gruvbox Red
          const lkpWidth = 145;
          ctx.fillStyle = '#fbf1c7';
          ctx.fillRect(lkp.x - lkpWidth / 2, lkp.y - 28, lkpWidth, 18);
          ctx.strokeStyle = '#cc241d';
          ctx.lineWidth = 1.2;
          ctx.strokeRect(lkp.x - lkpWidth / 2, lkp.y - 28, lkpWidth, 18);

          ctx.fillStyle = '#cc241d';
          ctx.font = 'bold 8.5px monospace';
          ctx.textAlign = 'center';
          ctx.fillText(`SIGNAL LOST (T+${elapsedSec}s) • ${exitSpeedMps} m/s`, lkp.x, lkp.y - 16);
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

    // 2. TOOL: ADD ANCHOR
    if (activeTool === 'ADD_TIER1_STOP' || (activeTool as any) === 'ADD_TIER2_STOP') {
      const newStop: GeoStop = {
        id: `anchor-${Date.now()}`,
        name: `Anchor #${stops.length + 1}`,
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

    // 4. TOOL: ADD GEOCENTRE (STRICTLY 1 INSTANCE)
    if (activeTool === 'ADD_GEOCENTRE') {
      const newCentre: GeoCentre = {
        id: 'geocentre-core',
        name: 'Central Gateway (Admin Block)',
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
