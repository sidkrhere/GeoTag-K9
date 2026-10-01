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

        // If a traced waypoint path exists, navigate along waypoints!
        if (dogPath.length >= 2) {
          const wpIdx = tag.currentWaypointIndex ?? 0;
          const targetWp = dogPath[wpIdx % dogPath.length];
          const dx = targetWp.x - tag.x;
          const dy = targetWp.y - tag.y;
          const dist = Math.sqrt(dx * dx + dy * dy);

          const stepSpeed = config.autoWalkSpeed * 1.6;

          if (dist < 10) {
            // Reached waypoint! Advance to next node in loop and continue fluid motion
            const nextIdx = (wpIdx + 1) % dogPath.length;
            const nextTarget = dogPath[nextIdx];
            const ndx = nextTarget.x - tag.x;
            const ndy = nextTarget.y - tag.y;
            const ndist = Math.sqrt(ndx * ndx + ndy * ndy) || 1;
            newVx = (ndx / ndist) * stepSpeed;
            newVy = (ndy / ndist) * stepSpeed;
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
            newVx = (dx / dist) * stepSpeed;
            newVy = (dy / dist) * stepSpeed;
          }
          newX = tag.x + newVx;
          newY = tag.y + newVy;
        } else {
          // Fallback: gentle boundary bounce if no path drawn yet
          newVx = tag.vx + (Math.random() - 0.5) * 0.3;
          newVy = tag.vy + (Math.random() - 0.5) * 0.3;
          const speed = Math.sqrt(newVx * newVx + newVy * newVy);
          const maxSpd = config.autoWalkSpeed;
          if (speed > maxSpd) {
            newVx = (newVx / speed) * maxSpd;
            newVy = (newVy / speed) * maxSpd;
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
          turnRate = turn / 0.033;
        }

        const cardinal = getCardinalDirection(newHeading);
        const dorsalRad = (newHeading * Math.PI) / 180;
        const nowMs = Date.now();
        const stepDistPix = Math.hypot(newVx, newVy);
        const actualSpeedMps = Math.round((stepDistPix / 0.033 / config.pixelsPerMeter) * 10) / 10;

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

      // 1. Clear & Background Grid
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Grid dots
      ctx.fillStyle = 'rgba(255, 255, 255, 0.03)';
      const gridSize = 30;
      for (let x = 0; x < canvas.width; x += gridSize) {
        for (let y = 0; y < canvas.height; y += gridSize) {
          ctx.beginPath();
          ctx.arc(x, y, 1, 0, Math.PI * 2);
          ctx.fill();
        }
      }

      // Discrete Packet Clock: Run multilateration solver at ~12.5 Hz (~80ms) for ultra-responsive tracking
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

      // 2. Render Existing Buildings
      for (const b of buildings) {
        if (b.points.length < 2) continue;

        ctx.beginPath();
        ctx.moveTo(b.points[0].x, b.points[0].y);
        for (let i = 1; i < b.points.length; i++) {
          ctx.lineTo(b.points[i].x, b.points[i].y);
        }

        if (b.closed) {
          ctx.closePath();
          ctx.fillStyle = b.color || 'rgba(59, 130, 246, 0.12)';
          ctx.fill();
          ctx.strokeStyle = 'rgba(59, 130, 246, 0.5)';
          ctx.lineWidth = 2;
          ctx.stroke();

          // Building label at centroid
          let cx = 0, cy = 0;
          for (const p of b.points) { cx += p.x; cy += p.y; }
          cx /= b.points.length;
          cy /= b.points.length;

          ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
          ctx.font = '600 11px Inter, sans-serif';
          ctx.textAlign = 'center';
          ctx.fillText(b.name, cx, cy);

          ctx.fillStyle = 'rgba(239, 68, 68, 0.6)';
          ctx.font = '500 9px monospace';
          ctx.fillText(`-${b.attenuationDb}dB Wall`, cx, cy + 14);
        } else {
          ctx.strokeStyle = 'rgba(0, 242, 254, 0.6)';
          ctx.setLineDash([4, 4]);
          ctx.lineWidth = 2;
          ctx.stroke();
          ctx.setLineDash([]);
        }

        // Draw vertex control nodes
        for (let i = 0; i < b.points.length; i++) {
          const pt = b.points[i];
          ctx.beginPath();
          ctx.arc(pt.x, pt.y, 4, 0, Math.PI * 2);
          ctx.fillStyle = 'rgba(255, 255, 255, 0.8)';
          ctx.fill();
          ctx.strokeStyle = '#00f2fe';
          ctx.lineWidth = 1.5;
          ctx.stroke();
        }
      }

      // 3. Render Active Building Drawing (In-Progress Lasso)
      if (activeTool === 'DRAW_BUILDING' && currentBuildingVertices.length > 0) {
        ctx.beginPath();
        ctx.moveTo(currentBuildingVertices[0].x, currentBuildingVertices[0].y);
        for (let i = 1; i < currentBuildingVertices.length; i++) {
          ctx.lineTo(currentBuildingVertices[i].x, currentBuildingVertices[i].y);
        }
        // Rubberband line to mouse
        ctx.lineTo(mousePos.x, mousePos.y);

        ctx.strokeStyle = 'rgba(0, 242, 254, 0.8)';
        ctx.setLineDash([6, 4]);
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.setLineDash([]);

        // Vertices
        for (let i = 0; i < currentBuildingVertices.length; i++) {
          const p = currentBuildingVertices[i];
          ctx.beginPath();
          ctx.arc(p.x, p.y, i === 0 ? 7 : 5, 0, Math.PI * 2);
          ctx.fillStyle = i === 0 ? '#00f2fe' : '#ffffff';
          ctx.fill();
          ctx.strokeStyle = '#0a0d14';
          ctx.lineWidth = 2;
          ctx.stroke();

          if (i === 0) {
            ctx.fillStyle = '#00f2fe';
            ctx.font = '600 10px Inter';
            ctx.fillText('Click to close', p.x, p.y - 12);
          }
        }
      }

      // 4. Render GeoStop Coverage Halos
      if (config.showCoverageHalos) {
        for (const stop of stops) {
          const radiusPixels = (stop.tier === 1 ? config.tier1RadiusMeters : config.tier2RadiusMeters) * config.pixelsPerMeter;
          const isTier1 = stop.tier === 1;

          // Outer pulsating ring
          ctx.beginPath();
          ctx.arc(stop.x, stop.y, radiusPixels, 0, Math.PI * 2);
          ctx.fillStyle = isTier1 ? 'rgba(0, 242, 254, 0.04)' : 'rgba(245, 158, 11, 0.04)';
          ctx.fill();

          ctx.strokeStyle = isTier1 
            ? `rgba(0, 242, 254, ${0.25 + pulse * 0.08})` 
            : `rgba(245, 158, 11, ${0.25 + pulse * 0.08})`;
          ctx.lineWidth = 1.5;
          ctx.setLineDash([6, 6]);
          ctx.stroke();
          ctx.setLineDash([]);
        }
      }

      // 5. Render Active RF Signals (Dog -> In-Range GeoStops)
      if (config.showRFRipples) {
        for (const sig of allSignals) {
          const tag = tags.find(t => t.id === sig.tagId);
          const stop = stops.find(s => s.id === sig.stopId);
          if (!tag || !stop) continue;

          ctx.beginPath();
          ctx.moveTo(tag.x, tag.y);
          ctx.lineTo(stop.x, stop.y);

          if (sig.isNlosZeroWeighted) {
            ctx.strokeStyle = 'rgba(239, 68, 68, 0.9)'; // Red dashed slash line for zero-weighted 15" wall anchor!
            ctx.lineWidth = 2.5;
            ctx.setLineDash([4, 6]);
          } else if (sig.isOutlier) {
            ctx.strokeStyle = 'rgba(245, 158, 11, 0.85)'; // Yellow dashed line for outlier anchor!
            ctx.lineWidth = 2;
            ctx.setLineDash([3, 5]);
          } else if (sig.obstructed) {
            ctx.strokeStyle = 'rgba(239, 68, 68, 0.6)'; // Red warning if wall attenuated!
            ctx.lineWidth = 1.5;
            ctx.setLineDash([4, 4]);
          } else {
            ctx.strokeStyle = stop.tier === 1 ? 'rgba(0, 242, 254, 0.5)' : 'rgba(245, 158, 11, 0.5)';
            ctx.lineWidth = 2;
            ctx.setLineDash([6, 6]);
            ctx.lineDashOffset = -flowOffset;
          }
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.lineDashOffset = 0;

          // Mid-beam RSSI badge
          const midX = (tag.x + stop.x) / 2;
          const midY = (tag.y + stop.y) / 2;
          
          let badgeText = `${sig.rssiDbm}dBm`;
          let badgeWidth = 46;
          let badgeColor = '#94a3b8';
          let borderColor = 'rgba(255, 255, 255, 0.2)';

          if (sig.isNlosZeroWeighted) {
            badgeText = `${sig.rssiDbm}dBm [15" NLOS - WEIGHT 0]`;
            badgeWidth = 152;
            badgeColor = '#ef4444';
            borderColor = '#ef4444';
          } else if (sig.isOutlier) {
            badgeText = `${sig.rssiDbm}dBm [OUTLIER]`;
            badgeWidth = 88;
            badgeColor = '#fbbf24';
            borderColor = '#f59e0b';
          } else if (sig.obstructed) {
            badgeColor = '#f87171';
            borderColor = '#ef4444';
          }

          ctx.fillStyle = 'rgba(10, 13, 20, 0.88)';
          ctx.fillRect(midX - badgeWidth / 2, midY - 9, badgeWidth, 16);
          ctx.strokeStyle = borderColor;
          ctx.strokeRect(midX - badgeWidth / 2, midY - 9, badgeWidth, 16);

          ctx.fillStyle = badgeColor;
          ctx.font = '500 9px monospace';
          ctx.textAlign = 'center';
          ctx.fillText(badgeText, midX, midY + 3);
        }
      }

      // 6. Render LoRa / Wi-Fi Backhaul Beams (Active GeoStops -> GeoCentre)
      if (config.showBackhaulBeams && geoCentre) {
        // Collect stops that are currently relaying dog signals
        const activeStopIds = new Set(allSignals.map(s => s.stopId));

        for (const stopId of activeStopIds) {
          const stop = stops.find(s => s.id === stopId);
          if (!stop) continue;

          ctx.beginPath();
          ctx.moveTo(stop.x, stop.y);
          ctx.lineTo(geoCentre.x, geoCentre.y);

          // LoRa beam (purple laser)
          ctx.strokeStyle = 'rgba(168, 85, 247, 0.7)';
          ctx.lineWidth = 2;
          ctx.setLineDash([4, 8]);
          ctx.lineDashOffset = -flowOffset * 1.5;
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.lineDashOffset = 0;

          // Moving packet particle
          const t = (pulsePhaseRef.current * 0.8) % 1;
          const px = stop.x + (geoCentre.x - stop.x) * t;
          const py = stop.y + (geoCentre.y - stop.y) * t;

          ctx.beginPath();
          ctx.arc(px, py, 3.5, 0, Math.PI * 2);
          ctx.fillStyle = '#c084fc';
          ctx.shadowColor = '#c084fc';
          ctx.shadowBlur = 8;
          ctx.fill();
          ctx.shadowBlur = 0;
        }
      }

      // 7. Render GeoCentre (Central Hub)
      if (geoCentre) {
        const isSelected = selectedEntityId === geoCentre.id;

        // Pulse ring
        ctx.beginPath();
        ctx.arc(geoCentre.x, geoCentre.y, 22 + pulse * 3, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(168, 85, 247, 0.4)';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Hub Icon circle
        ctx.beginPath();
        ctx.arc(geoCentre.x, geoCentre.y, 14, 0, Math.PI * 2);
        ctx.fillStyle = '#7e22ce';
        ctx.fill();
        ctx.strokeStyle = isSelected ? '#00f2fe' : '#c084fc';
        ctx.lineWidth = isSelected ? 3 : 2;
        ctx.stroke();

        // Server glyph symbol
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 12px Inter';
        ctx.textAlign = 'center';
        ctx.fillText('🏢', geoCentre.x, geoCentre.y + 4);

        // Name
        ctx.fillStyle = '#e9d5ff';
        ctx.font = '600 11px Inter';
        ctx.fillText(geoCentre.name, geoCentre.x, geoCentre.y - 18);
      }

      // 8. Render GeoStops
      for (const stop of stops) {
        const isSelected = selectedEntityId === stop.id;
        const isTier1 = stop.tier === 1;

        ctx.beginPath();
        ctx.arc(stop.x, stop.y, 11, 0, Math.PI * 2);
        ctx.fillStyle = isTier1 ? '#0891b2' : '#d97706';
        ctx.fill();
        ctx.strokeStyle = isSelected ? '#ffffff' : (isTier1 ? '#67e8f9' : '#fde68a');
        ctx.lineWidth = isSelected ? 3 : 2;
        ctx.stroke();

        // Icon representation
        ctx.font = '11px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(isTier1 ? '📡' : '☕', stop.x, stop.y + 4);

        // Name & Tag
        ctx.fillStyle = isTier1 ? '#a5f3fc' : '#fde68a';
        ctx.font = '500 10px Inter';
        ctx.fillText(stop.name, stop.x, stop.y - 15);
      }

      // 8.5 Render Traced Doggo Waypoint Route & Turn Badges
      if (dogPath.length > 0) {
        // Draw route path line
        if (dogPath.length >= 2) {
          ctx.beginPath();
          ctx.moveTo(dogPath[0].x, dogPath[0].y);
          for (let i = 1; i < dogPath.length; i++) {
            ctx.lineTo(dogPath[i].x, dogPath[i].y);
          }
          if (dogPath.length >= 3) {
            ctx.closePath();
          }
          ctx.strokeStyle = 'rgba(52, 211, 153, 0.55)';
          ctx.lineWidth = 2.5;
          ctx.setLineDash([8, 6]);
          ctx.lineDashOffset = -flowOffset * 0.7;
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.lineDashOffset = 0;
        }

        // Draw rubberband line if currently tracing path
        if (activeTool === 'TRACE_PATH') {
          const lastWp = dogPath[dogPath.length - 1];
          ctx.beginPath();
          ctx.moveTo(lastWp.x, lastWp.y);
          ctx.lineTo(mousePos.x, mousePos.y);
          ctx.strokeStyle = 'rgba(52, 211, 153, 0.85)';
          ctx.lineWidth = 2;
          ctx.setLineDash([4, 4]);
          ctx.stroke();
          ctx.setLineDash([]);
        }

        // Render Waypoint Turn Badges ①, ②, ③...
        const turnGlyphs = ['①', '②', '③', '④', '⑤', '⑥', '⑦', '⑧', '⑨', '⑩', '⑪', '⑫', '⑬', '⑭', '⑮', '⑯'];
        for (let i = 0; i < dogPath.length; i++) {
          const wp = dogPath[i];
          const isCurrentTarget = tags[0]?.currentWaypointIndex === i;

          // Pulse ring for the next immediate waypoint
          if (isCurrentTarget) {
            ctx.beginPath();
            ctx.arc(wp.x, wp.y, 16 + pulse * 3, 0, Math.PI * 2);
            ctx.strokeStyle = 'rgba(52, 211, 153, 0.6)';
            ctx.lineWidth = 1.5;
            ctx.stroke();
          }

          // Node Circle
          ctx.beginPath();
          ctx.arc(wp.x, wp.y, 11, 0, Math.PI * 2);
          ctx.fillStyle = isCurrentTarget ? '#065f46' : '#064e3b';
          ctx.fill();
          ctx.strokeStyle = isCurrentTarget ? '#34d399' : '#10b981';
          ctx.lineWidth = isCurrentTarget ? 2.5 : 1.5;
          ctx.stroke();

          // Number badge
          ctx.fillStyle = '#ffffff';
          ctx.font = 'bold 11px Inter, sans-serif';
          ctx.textAlign = 'center';
          const glyph = i < turnGlyphs.length ? turnGlyphs[i] : `${i + 1}`;
          ctx.fillText(glyph, wp.x, wp.y + 4);

          // Sub-label
          ctx.fillStyle = isCurrentTarget ? '#34d399' : 'rgba(167, 243, 208, 0.7)';
          ctx.font = '500 9px monospace';
          ctx.fillText(isCurrentTarget ? `Next Turn` : `Turn #${i + 1}`, wp.x, wp.y - 15);
        }
      }

      // Cursor hint for Trace Path tool
      if (activeTool === 'TRACE_PATH') {
        ctx.fillStyle = 'rgba(10, 13, 20, 0.85)';
        ctx.fillRect(mousePos.x + 12, mousePos.y + 12, 160, 22);
        ctx.strokeStyle = '#10b981';
        ctx.lineWidth = 1;
        ctx.strokeRect(mousePos.x + 12, mousePos.y + 12, 160, 22);
        ctx.fillStyle = '#34d399';
        ctx.font = '500 10px Inter, sans-serif';
        ctx.textAlign = 'left';
        ctx.fillText(`Click to add Turn #${dogPath.length + 1}`, mousePos.x + 18, mousePos.y + 27);
      }

      // 9. Render Ground-Truth GeoTag Doggos & Calculated Positions
      for (const tag of tags) {
        const isSelected = selectedEntityId === tag.id;
        const res = results.find(r => r.tagId === tag.id);

        // A. True Doggo Position
        if (config.showTrueDoggos) {
          const heading = tag.headingDeg ?? 90;
          // Canvas polar coordinates: 0° heading (North) corresponds to -PI/2 in radians
          const headingRad = ((heading - 90) * Math.PI) / 180;

          // A0. Canine Dorsal Collar Forward Vision/Radar Cone (60° FOV)
          if (config.showHeadingVisionCone !== false) {
            const coneFovRad = (60 * Math.PI) / 180;
            const coneDist = 48; // ~1.7 meters forward radar reach
            const halfFov = coneFovRad / 2;

            ctx.save();
            ctx.beginPath();
            ctx.moveTo(tag.x, tag.y);
            ctx.arc(tag.x, tag.y, coneDist, headingRad - halfFov, headingRad + halfFov);
            ctx.closePath();

            const coneGrad = ctx.createRadialGradient(tag.x, tag.y, 4, tag.x, tag.y, coneDist);
            coneGrad.addColorStop(0, 'rgba(0, 242, 254, 0.32)');
            coneGrad.addColorStop(0.6, 'rgba(0, 242, 254, 0.10)');
            coneGrad.addColorStop(1, 'rgba(0, 242, 254, 0.0)');
            ctx.fillStyle = coneGrad;
            ctx.fill();

            // Arc perimeter dashed rim
            ctx.beginPath();
            ctx.arc(tag.x, tag.y, coneDist, headingRad - halfFov, headingRad + halfFov);
            ctx.strokeStyle = 'rgba(0, 242, 254, 0.55)';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([3, 3]);
            ctx.stroke();
            ctx.setLineDash([]);

            // Radial boundary rays
            ctx.beginPath();
            ctx.moveTo(tag.x, tag.y);
            ctx.lineTo(tag.x + Math.cos(headingRad - halfFov) * coneDist, tag.y + Math.sin(headingRad - halfFov) * coneDist);
            ctx.moveTo(tag.x, tag.y);
            ctx.lineTo(tag.x + Math.cos(headingRad + halfFov) * coneDist, tag.y + Math.sin(headingRad + halfFov) * coneDist);
            ctx.strokeStyle = 'rgba(0, 242, 254, 0.22)';
            ctx.lineWidth = 1;
            ctx.stroke();
            ctx.restore();
          }

          // A1. Dog Body Circle
          ctx.beginPath();
          ctx.arc(tag.x, tag.y, 12, 0, Math.PI * 2);
          ctx.fillStyle = tag.color;
          ctx.fill();
          ctx.strokeStyle = isSelected ? '#ffffff' : '#0a0d14';
          ctx.lineWidth = 2.5;
          ctx.stroke();

          // A2. Directional Snout Needle / Collar Axis Pointer
          const needleLen = 19;
          const tipX = tag.x + Math.cos(headingRad) * needleLen;
          const tipY = tag.y + Math.sin(headingRad) * needleLen;

          ctx.beginPath();
          ctx.moveTo(tag.x, tag.y);
          ctx.lineTo(tipX, tipY);
          ctx.strokeStyle = '#00f2fe';
          ctx.lineWidth = 2.5;
          ctx.stroke();

          // Arrowhead at snout
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

          // Cute Dog Emoji
          ctx.font = '13px sans-serif';
          ctx.textAlign = 'center';
          ctx.fillText(tag.imuResting ? '💤' : '🐕', tag.x, tag.y + 5);

          // Name and Battery
          ctx.fillStyle = '#ffffff';
          ctx.font = '600 11px Inter';
          ctx.fillText(tag.name, tag.x, tag.y + 24);

          // Dorsal e-Compass Badge
          const cardinal = tag.compassData?.cardinal ?? getCardinalDirection(heading);
          ctx.fillStyle = 'rgba(10, 13, 20, 0.85)';
          ctx.fillRect(tag.x - 40, tag.y + 28, 80, 15);
          ctx.strokeStyle = 'rgba(0, 242, 254, 0.4)';
          ctx.lineWidth = 1;
          ctx.strokeRect(tag.x - 40, tag.y + 28, 80, 15);

          ctx.font = 'bold 8.5px monospace';
          ctx.fillStyle = '#00f2fe';
          ctx.fillText(`🧭 ${Math.round(heading)}° ${cardinal}`, tag.x, tag.y + 39);
        }

        // B. Server Calculated Localization Geometric Manifolds
        if (config.showEstimatedLocations && res && res.lockType !== 'BLIND_ZONE') {
          // Unified Continuous Display Coordinate Filter across ALL modes:
          // Guarantees zero coordinate jump during 2 -> 3 anchor handoffs and eliminates mirror latch-up snapping!
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

          // Check if an anchor count transition just occurred (e.g. 2 -> 3 or 3 -> 2)
          if (transitionInfo && transitionInfo.prevAnchorCount !== currentAnchorCount) {
            transitionInfo.prevAnchorCount = currentAnchorCount;
            transitionInfo.prevCoord = { ...displayPos };
            transitionInfo.transitionStartTime = Date.now();
          }

          // 1. Continuous 60 FPS Kinematic Dead-Reckoning (Predictive Forward Projection):
          // Eliminates the spatial lag where the crosshair trailed behind the dog!
          const nowMs = Date.now();
          const elapsedSec = Math.max(0, Math.min(0.20, (nowMs - (res.timestamp || nowMs)) / 1000));
          const vx = res.velocity?.x || 0;
          const vy = res.velocity?.y || 0;
          const deadReckonedX = res.estimatedX + vx * elapsedSec;
          const deadReckonedY = res.estimatedY + vy * elapsedSec;

          // 2. Compute target position with progressive 160ms transition blending
          let targetX = deadReckonedX;
          let targetY = deadReckonedY;

          if (transitionInfo && transitionInfo.transitionStartTime > 0) {
            const elapsed = nowMs - transitionInfo.transitionStartTime;
            if (elapsed < 160) {
              const t = elapsed / 160;
              // Smoothstep blend: 3t^2 - 2t^3
              const smoothT = t * t * (3 - 2 * t);
              targetX = transitionInfo.prevCoord.x * (1 - smoothT) + deadReckonedX * smoothT;
              targetY = transitionInfo.prevCoord.y * (1 - smoothT) + deadReckonedY * smoothT;
            } else {
              transitionInfo.transitionStartTime = 0;
            }
          }

          // 3. Fast-Settling Adaptive Display Filter & Slew Limiter:
          // Settles in ~2 frames (35ms) instead of 160ms, tracking fast turns with zero lag!
          const dx = targetX - displayPos.x;
          const dy = targetY - displayPos.y;
          const dist = Math.sqrt(dx * dx + dy * dy);

          if (dist > 0.05) {
            const normalStep = dist * 0.55;
            const maxStepPerFrame = Math.max(16.0, (config.autoWalkSpeed ?? 1.5) * 8.0);
            const actualStep = dist > 20.0 ? Math.min(dist * 0.75, maxStepPerFrame * 2.5) : Math.min(normalStep, maxStepPerFrame);
            displayPos.x += (dx / dist) * actualStep;
            displayPos.y += (dy / dist) * actualStep;
          }

          // B0. HISTORICAL TRAJECTORY BREADCRUMB TAIL
          if (config.showHistoricalBreadcrumbs && res.historyTrail && res.historyTrail.length >= 2) {
            const trail = res.historyTrail;
            ctx.save();

            // 1. Connect fixes with fading trajectory line
            ctx.beginPath();
            ctx.moveTo(trail[0].x, trail[0].y);
            for (let i = 1; i < trail.length; i++) {
              ctx.lineTo(trail[i].x, trail[i].y);
            }
            ctx.strokeStyle = 'rgba(0, 242, 254, 0.25)';
            ctx.lineWidth = 2;
            ctx.setLineDash([3, 4]);
            ctx.stroke();
            ctx.setLineDash([]);

            // 2. Render progressive glowing breadcrumb nodes
            for (let i = 0; i < trail.length; i++) {
              const fix = trail[i];
              const progress = (i + 1) / trail.length;
              const dotRadius = 1.8 + progress * 2.5; // 1.8px (oldest) to 4.3px (latest)
              const alpha = 0.15 + progress * 0.75;  // 0.15 (oldest) to 0.90 (latest)

              let color = `rgba(16, 185, 129, ${alpha})`; // Emerald for 3D lock
              if (fix.lockType === 'DUAL_PINPOINT_LOCK') {
                color = `rgba(0, 242, 254, ${alpha})`; // Cyan for Pinpoint
              } else if (fix.lockType === 'DUAL_KINEMATIC_LOCK' || fix.lockType === 'DUAL_AMBIGUOUS_CHORD' || fix.lockType === 'RADIAL_RING') {
                color = `rgba(245, 158, 11, ${alpha})`; // Amber for 2-anchor kinematic / ring
              } else if (fix.lockType === 'BLIND_ZONE') {
                color = `rgba(244, 63, 94, ${alpha})`; // Rose for Blind zone
              }

              // Halo glow
              ctx.beginPath();
              ctx.arc(fix.x, fix.y, dotRadius + 2.0, 0, Math.PI * 2);
              ctx.fillStyle = color.replace(/[\d.]+\)$/, `${alpha * 0.3})`);
              ctx.fill();

              // Solid center dot
              ctx.beginPath();
              ctx.arc(fix.x, fix.y, dotRadius, 0, Math.PI * 2);
              ctx.fillStyle = color;
              ctx.fill();
            }

            ctx.restore();
          }

          // B1. EXACT 1-ANCHOR RADIAL UNCERTAINTY RING
          if (res.lockType === 'RADIAL_RING' && res.radialRing) {
            const { center, radiusPixels, radiusMeters } = res.radialRing;

            // Render pulsating distance circle
            ctx.beginPath();
            ctx.arc(center.x, center.y, radiusPixels, 0, Math.PI * 2);
            ctx.strokeStyle = `rgba(245, 158, 11, ${0.45 + pulse * 0.15})`;
            ctx.lineWidth = 2;
            ctx.setLineDash([5, 5]);
            ctx.stroke();
            ctx.setLineDash([]);

            // Radial label along ring
            const angle = -Math.PI / 4;
            const lx = center.x + Math.cos(angle) * radiusPixels;
            const ly = center.y + Math.sin(angle) * radiusPixels;

            ctx.fillStyle = 'rgba(10, 13, 20, 0.85)';
            ctx.fillRect(lx - 60, ly - 9, 120, 18);
            ctx.strokeStyle = '#f59e0b';
            ctx.strokeRect(lx - 60, ly - 9, 120, 18);

            ctx.fillStyle = '#fbbf24';
            ctx.font = '600 9px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(`⭕ 1-Anchor Ring: ~${radiusMeters}m`, lx, ly + 3);
          }

          // B2. 2-ANCHOR DUAL CANDIDATE INTERSECTIONS & KINEMATIC MIRROR OUTCASTING
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

            // Dashed chord line between candidates
            ctx.beginPath();
            ctx.moveTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.strokeStyle = isPinpoint 
              ? 'rgba(0, 242, 254, 0.45)' 
              : res.lockType === 'DUAL_KINEMATIC_LOCK' 
              ? 'rgba(52, 211, 153, 0.35)' 
              : 'rgba(251, 191, 36, 0.4)';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([4, 4]);
            ctx.stroke();
            ctx.setLineDash([]);

            // Mid-chord separation and mirror ghost error label
            const midChordX = (p1.x + p2.x) / 2;
            const midChordY = (p1.y + p2.y) / 2;
            const separationM = dualInfo ? dualInfo.separationChordMeters : Math.round((distance(p1, p2) / config.pixelsPerMeter) * 10) / 10;
            const ghostErrM = dualInfo?.candidate2ErrorMeters ?? Math.round((distance({ x: tag.x, y: tag.y }, p2) / config.pixelsPerMeter) * 10) / 10;

            const chordText = dualInfo?.isTransitionBuffering
              ? `⏱️ Transition Buffer (120ms Vector Match) | Chord: ${separationM}m`
              : isPinpoint
              ? `🧭 Fused 2-Anchor Lock (Chord: ${separationM}m | Ghost Error: Δ${ghostErrM}m)`
              : res.lockType === 'DUAL_KINEMATIC_LOCK' 
              ? `Chord: ${separationM}m | Ghost Error: Δ${ghostErrM}m` 
              : res.lockType === 'DUAL_AMBIGUOUS_CHORD'
              ? `⚡ Baseline Crossing (${separationM}m)`
              : `Awaiting 2nd Packet (Syncing)`;

            const badgeWidth = isPinpoint ? 290 : res.lockType === 'DUAL_KINEMATIC_LOCK' ? 220 : 170;
            ctx.fillStyle = 'rgba(10, 13, 20, 0.90)';
            ctx.fillRect(midChordX - badgeWidth / 2, midChordY - 8, badgeWidth, 16);
            ctx.strokeStyle = isPinpoint ? '#00f2fe' : res.lockType === 'DUAL_KINEMATIC_LOCK' ? '#10b981' : '#f59e0b';
            ctx.strokeRect(midChordX - badgeWidth / 2, midChordY - 8, badgeWidth, 16);
            ctx.fillStyle = isPinpoint ? '#00f2fe' : res.lockType === 'DUAL_KINEMATIC_LOCK' ? '#34d399' : '#fbbf24';
            ctx.font = '500 8.5px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(chordText, midChordX, midChordY + 4);

            const valIdx = res.dualInfo?.validatedIndex ?? 0;
            const outIdx = res.dualInfo?.outcastIndex ?? 1;

            // Render Candidate #1 (Validated) & Candidate #2 (Outcast Mirror Ghost)
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
                // 1. OUTCAST MIRROR GHOST (STRIKE-THROUGH & OUTCAST BADGE)
                const thisGhostErrM = Math.round((distance({ x: tag.x, y: tag.y }, cand) / config.pixelsPerMeter) * 10) / 10;
                ctx.beginPath();
                ctx.arc(cand.x, cand.y, 9, 0, Math.PI * 2);
                ctx.fillStyle = 'rgba(239, 68, 68, 0.12)';
                ctx.fill();
                ctx.strokeStyle = 'rgba(239, 68, 68, 0.5)';
                ctx.lineWidth = 1.5;
                ctx.setLineDash([3, 3]);
                ctx.stroke();
                ctx.setLineDash([]);

                // Strike-through X
                ctx.beginPath();
                ctx.moveTo(cand.x - 6, cand.y - 6); ctx.lineTo(cand.x + 6, cand.y + 6);
                ctx.moveTo(cand.x + 6, cand.y - 6); ctx.lineTo(cand.x - 6, cand.y + 6);
                ctx.strokeStyle = '#ef4444';
                ctx.lineWidth = 2;
                ctx.stroke();

                // Outcast Label above with calculated error and vector alignment!
                const ghostAlign = idx === 0 ? dualInfo?.candidate1VectorAlignment : dualInfo?.candidate2VectorAlignment;
                const ghostAlignText = ghostAlign !== undefined ? ` | Match: ${(ghostAlign >= 0 ? '+' : '') + ghostAlign.toFixed(2)}` : '';
                ctx.fillStyle = 'rgba(10, 13, 20, 0.9)';
                const leapLabel = `❌ Mirror Ghost [OUTCAST: Δ${thisGhostErrM}m${ghostAlignText}]`;
                const outcastBadgeW = 210;
                ctx.fillRect(cand.x - outcastBadgeW / 2, cand.y - 24, outcastBadgeW, 16);
                ctx.strokeStyle = '#ef4444';
                ctx.strokeRect(cand.x - outcastBadgeW / 2, cand.y - 24, outcastBadgeW, 16);

                ctx.fillStyle = '#f87171';
                ctx.font = 'bold 8.5px monospace';
                ctx.textAlign = 'center';
                ctx.fillText(leapLabel, cand.x, cand.y - 13);
              } else if (isCandidateValidated) {
                // 2. VALIDATED DORSAL PINPOINT / KINEMATIC TARGET LOCK
                const vx = displayPos.x;
                const vy = displayPos.y;

                // Connect true position to validated point with dashed error line
                ctx.beginPath();
                ctx.moveTo(tag.x, tag.y);
                ctx.lineTo(vx, vy);
                ctx.strokeStyle = res.errorMeters < 3 ? 'rgba(52, 211, 153, 0.8)' : 'rgba(251, 191, 36, 0.8)';
                ctx.lineWidth = 1.5;
                ctx.setLineDash([3, 3]);
                ctx.stroke();
                ctx.setLineDash([]);

                // Mid-line error badge
                const midX = (tag.x + vx) / 2;
                const midY = (tag.y + vy) / 2;
                ctx.fillStyle = 'rgba(10, 13, 20, 0.9)';
                ctx.fillRect(midX - 28, midY - 8, 56, 15);
                ctx.strokeStyle = res.errorMeters < 3 ? '#10b981' : '#f59e0b';
                ctx.strokeRect(midX - 28, midY - 8, 56, 15);

                ctx.fillStyle = res.errorMeters < 3 ? '#34d399' : '#fbbf24';
                ctx.font = '600 9px monospace';
                ctx.textAlign = 'center';
                ctx.fillText(`Δ ${res.errorMeters}m`, midX, midY + 3);

                // Target Reticle
                ctx.beginPath();
                ctx.arc(vx, vy, 11, 0, Math.PI * 2);
                ctx.fillStyle = isPinpoint ? 'rgba(0, 242, 254, 0.22)' : 'rgba(16, 185, 129, 0.2)';
                ctx.fill();
                ctx.strokeStyle = isPinpoint ? '#00f2fe' : '#10b981';
                ctx.lineWidth = 2.5;
                ctx.stroke();

                // Crosshairs
                ctx.beginPath();
                ctx.moveTo(vx - 15, vy); ctx.lineTo(vx + 15, vy);
                ctx.moveTo(vx, vy - 15); ctx.lineTo(vx, vy + 15);
                ctx.strokeStyle = isPinpoint ? '#00f2fe' : '#10b981';
                ctx.lineWidth = 1.5;
                ctx.stroke();

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
                  ctx.strokeStyle = isPinpoint ? '#00f2fe' : '#34d399';
                  ctx.lineWidth = 2;
                  ctx.stroke();

                  // Arrowhead
                  const headLen = 6;
                  ctx.beginPath();
                  ctx.moveTo(tipX, tipY);
                  ctx.lineTo(tipX - headLen * (dirX + dirY * 0.5), tipY - headLen * (dirY - dirX * 0.5));
                  ctx.lineTo(tipX - headLen * (dirX - dirY * 0.5), tipY - headLen * (dirY + dirX * 0.5));
                  ctx.closePath();
                  ctx.fillStyle = isPinpoint ? '#00f2fe' : '#34d399';
                  ctx.fill();
                }

                // Validated Label with vector alignment score
                const valAlign = idx === 0 ? dualInfo?.candidate1VectorAlignment : dualInfo?.candidate2VectorAlignment;
                const valAlignText = valAlign !== undefined ? ` | Match: ${(valAlign >= 0 ? '+' : '') + valAlign.toFixed(2)}` : '';
                ctx.fillStyle = 'rgba(10, 13, 20, 0.9)';
                const valLabel = isPinpoint 
                  ? `🧭 Fused Lock (Δ ${res.errorMeters}m${valAlignText})`
                  : `🎯 2-Anchor Lock (Δ ${res.errorMeters}m)`;
                const valBadgeW = isPinpoint ? 210 : 160;
                ctx.fillRect(vx - valBadgeW / 2, vy - 24, valBadgeW, 16);
                ctx.strokeStyle = isPinpoint ? '#00f2fe' : '#10b981';
                ctx.strokeRect(vx - valBadgeW / 2, vy - 24, valBadgeW, 16);

                ctx.fillStyle = isPinpoint ? '#00f2fe' : '#34d399';
                ctx.font = 'bold 8.5px monospace';
                ctx.textAlign = 'center';
                ctx.fillText(valLabel, vx, vy - 13);
              } else {
                // 3. AMBIGUOUS CANDIDATE
                ctx.beginPath();
                ctx.arc(cand.x, cand.y, 8, 0, Math.PI * 2);
                ctx.fillStyle = 'rgba(245, 158, 11, 0.25)';
                ctx.fill();
                ctx.strokeStyle = '#f59e0b';
                ctx.lineWidth = 2;
                ctx.stroke();

                ctx.fillStyle = '#fbbf24';
                ctx.font = 'bold 9px monospace';
                ctx.textAlign = 'center';
                ctx.fillText(`❓ Candidate #${idx + 1}`, cand.x, cand.y - 12);
              }
            });
          }

          // B3. FULL 3D/2D MULTILATERATION LOCK (KALMAN / LOW-PASS SMOOTHED AT displayPos)
          else if (res.lockType === 'FULL_3D_LOCK') {
            const estX = displayPos.x;
            const estY = displayPos.y;

            // Connect true to estimated with dashed error line
            ctx.beginPath();
            ctx.moveTo(tag.x, tag.y);
            ctx.lineTo(estX, estY);
            ctx.strokeStyle = res.errorMeters < 3 ? 'rgba(52, 211, 153, 0.8)' : 'rgba(251, 191, 36, 0.8)';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([3, 3]);
            ctx.stroke();
            ctx.setLineDash([]);

            // Error Distance Label
            const midX = (tag.x + estX) / 2;
            const midY = (tag.y + estY) / 2;
            ctx.fillStyle = 'rgba(10, 13, 20, 0.9)';
            ctx.fillRect(midX - 28, midY - 8, 56, 15);
            ctx.strokeStyle = res.errorMeters < 3 ? '#10b981' : '#f59e0b';
            ctx.strokeRect(midX - 28, midY - 8, 56, 15);

            ctx.fillStyle = res.errorMeters < 3 ? '#34d399' : '#fbbf24';
            ctx.font = '600 9px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(`Δ ${res.errorMeters}m`, midX, midY + 3);

            // Calculated Target Crosshair 🎯
            ctx.beginPath();
            ctx.arc(estX, estY, 10, 0, Math.PI * 2);
            ctx.strokeStyle = res.hypothesisFlipDetected ? '#00f2fe' : '#f43f5e';
            ctx.lineWidth = 2.5;
            ctx.stroke();

            // Crosshair lines
            ctx.beginPath();
            ctx.moveTo(estX - 14, estY); ctx.lineTo(estX + 14, estY);
            ctx.moveTo(estX, estY - 14); ctx.lineTo(estX, estY + 14);
            ctx.strokeStyle = res.hypothesisFlipDetected ? '#00f2fe' : '#f43f5e';
            ctx.lineWidth = 1.5;
            ctx.stroke();

            ctx.fillStyle = res.hypothesisFlipDetected ? '#38bdf8' : '#f43f5e';
            ctx.font = 'bold 9px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(res.hypothesisFlipDetected ? '🔄 3D LOCK (FLIP RE-ALIGNED)' : `3D LOCK #${tag.id.slice(-2)}`, estX, estY - 14);
          }
        } else if (config.showEstimatedLocations && res && res.lockType === 'BLIND_ZONE' && res.borderExit) {
          // B4. BLIND ZONE: 0-ANCHOR BORDER EXIT WITH LAST KNOWN POSITION (LKP) & VELOCITY VECTOR
          const lkp = res.borderExit.lastKnownPos;
          const exitSpeedMps = res.borderExit.exitSpeedMps;
          const elapsedSec = res.borderExit.elapsedSeconds;
          const headingDeg = res.borderExit.exitHeadingDeg;

          // 1. Expanding dashed uncertainty horizon ring
          const uncertRadius = Math.max(16, Math.min(180, elapsedSec * exitSpeedMps * config.pixelsPerMeter));
          ctx.beginPath();
          ctx.arc(lkp.x, lkp.y, uncertRadius, 0, Math.PI * 2);
          ctx.strokeStyle = `rgba(239, 68, 68, ${Math.max(0.15, 0.45 - elapsedSec * 0.03)})`;
          ctx.lineWidth = 1.5;
          ctx.setLineDash([4, 4]);
          ctx.stroke();
          ctx.setLineDash([]);

          // 2. Pulsing Border Beacon at Last Known Position
          const beaconPulse = 0.5 + Math.sin(pulsePhaseRef.current * 1.5) * 0.5;
          ctx.beginPath();
          ctx.arc(lkp.x, lkp.y, 8 + beaconPulse * 4, 0, Math.PI * 2);
          ctx.fillStyle = `rgba(239, 68, 68, ${0.25 + beaconPulse * 0.25})`;
          ctx.fill();

          ctx.beginPath();
          ctx.arc(lkp.x, lkp.y, 5, 0, Math.PI * 2);
          ctx.fillStyle = '#ef4444';
          ctx.fill();
          ctx.strokeStyle = '#ffffff';
          ctx.lineWidth = 2;
          ctx.stroke();

          // 3. Directional Velocity Vector Arrow at Border Exit
          const velLength = Math.max(28, Math.min(85, exitSpeedMps * 18));
          const exitRad = ((headingDeg - 90) * Math.PI) / 180;
          const tipX = lkp.x + Math.cos(exitRad) * velLength;
          const tipY = lkp.y + Math.sin(exitRad) * velLength;

          ctx.beginPath();
          ctx.moveTo(lkp.x, lkp.y);
          ctx.lineTo(tipX, tipY);
          ctx.strokeStyle = '#f59e0b'; // Amber velocity vector arrow
          ctx.lineWidth = 3;
          ctx.stroke();

          // Arrowhead at velocity tip
          const arrowHead = 7;
          const leftAngle = exitRad + Math.PI * 0.82;
          const rightAngle = exitRad - Math.PI * 0.82;
          ctx.beginPath();
          ctx.moveTo(tipX, tipY);
          ctx.lineTo(tipX + Math.cos(leftAngle) * arrowHead, tipY + Math.sin(leftAngle) * arrowHead);
          ctx.lineTo(tipX + Math.cos(rightAngle) * arrowHead, tipY + Math.sin(rightAngle) * arrowHead);
          ctx.closePath();
          ctx.fillStyle = '#f59e0b';
          ctx.fill();

          // Velocity vector label
          const labelMidX = (lkp.x + tipX) / 2;
          const labelMidY = (lkp.y + tipY) / 2;
          ctx.fillStyle = 'rgba(10, 13, 20, 0.9)';
          ctx.fillRect(labelMidX - 35, labelMidY - 18, 70, 14);
          ctx.strokeStyle = '#f59e0b';
          ctx.strokeRect(labelMidX - 35, labelMidY - 18, 70, 14);

          ctx.fillStyle = '#fbbf24';
          ctx.font = '600 8px monospace';
          ctx.textAlign = 'center';
          ctx.fillText(`v_exit: ${exitSpeedMps}m/s`, labelMidX, labelMidY - 8);

          // 4. Prominent LKP Warning Badge
          ctx.fillStyle = 'rgba(10, 13, 20, 0.92)';
          ctx.fillRect(lkp.x - 70, lkp.y - 28, 140, 18);
          ctx.strokeStyle = '#ef4444';
          ctx.lineWidth = 1.5;
          ctx.strokeRect(lkp.x - 70, lkp.y - 28, 140, 18);

          ctx.fillStyle = '#f87171';
          ctx.font = 'bold 9px monospace';
          ctx.textAlign = 'center';
          ctx.fillText(`📍 LKP (T-${elapsedSec}s | θ=${Math.round(headingDeg)}°)`, lkp.x, lkp.y - 16);
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
