import React, { useState, useEffect } from 'react';
import { TopToolbar } from './components/TopToolbar';
import { ConfigPanel } from './components/ConfigPanel';
import { TelemetryHUD } from './components/TelemetryHUD';
import { TelemetryFeed } from './components/TelemetryFeed';
import { SimulationCanvas } from './components/SimulationCanvas';
import {
  Point2D,
  ActiveTool,
  SimulationConfig,
  BuildingPolygon,
  GeoStop,
  GeoCentre,
  GeoTag,
  TrilaterationResult,
  LapMetrics
} from './types/simulation';
import { getIITRoparPreset } from './engine/presetLayouts';

export const App: React.FC = () => {
  const [activeTool, setActiveTool] = useState<ActiveTool>('SELECT_MOVE');
  const [autoWalk, setAutoWalk] = useState<boolean>(true);
  const [selectedEntityId, setSelectedEntityId] = useState<string | null>(null);
  const [resetKey, setResetKey] = useState<number>(0);

  // Lap & Lifetime Error Tracking
  const [lapMetrics, setLapMetrics] = useState<LapMetrics>({
    currentLapNumber: 1,
    currentLapAvgError: 0,
    lastCompletedLapAvgError: null,
    lifetimeAvgError: 0,
    currentLapSamples: 0,
    totalSamples: 0
  });

  // Simulation parameters & sliders
  const [config, setConfig] = useState<SimulationConfig>({
    tier1RadiusMeters: 90,
    tier2RadiusMeters: 45,
    noiseStdDevDb: 3.0,
    wallAttenuationDb: 12,
    pathLossExponent: 3.0,
    pixelsPerMeter: 2.8,
    autoWalkSpeed: 1.2,
    showRFRipples: true,
    showBackhaulBeams: true,
    showCoverageHalos: true,
    showTrueDoggos: true,
    showEstimatedLocations: true,
    showHeadingVisionCone: true,
    showHistoricalBreadcrumbs: true,
    imuSensorType: 'ECOMPASS_9DOF'
  });

  // Entities
  const [buildings, setBuildings] = useState<BuildingPolygon[]>([]);
  const [stops, setStops] = useState<GeoStop[]>([]);
  const [geoCentre, setGeoCentre] = useState<GeoCentre | null>(null);
  const [tags, setTags] = useState<GeoTag[]>([]);
  const [dogPath, setDogPath] = useState<Point2D[]>([]);
  const [trilaterationResults, setTrilaterationResults] = useState<TrilaterationResult[]>([]);

  // Load IIT Ropar preset on initial load
  useEffect(() => {
    const width = window.innerWidth || 1200;
    const height = window.innerHeight || 800;
    const preset = getIITRoparPreset(width, height);
    setBuildings(preset.buildings);
    setStops(preset.stops);
    setGeoCentre(preset.centre);
    setTags(preset.tags);
    if (preset.tags[0]?.pathWaypoints) {
      setDogPath(preset.tags[0].pathWaypoints);
    }
  }, []);

  const handleResetPreset = () => {
    const width = window.innerWidth || 1200;
    const height = window.innerHeight || 800;
    const preset = getIITRoparPreset(width, height);
    setBuildings(preset.buildings);
    setStops(preset.stops);
    setGeoCentre(preset.centre);
    setTags(preset.tags);
    if (preset.tags[0]?.pathWaypoints) {
      setDogPath(preset.tags[0].pathWaypoints);
    } else {
      setDogPath([]);
    }
    setSelectedEntityId(null);
    setResetKey(k => k + 1);
    setLapMetrics({
      currentLapNumber: 1,
      currentLapAvgError: 0,
      lastCompletedLapAvgError: null,
      lifetimeAvgError: 0,
      currentLapSamples: 0,
      totalSamples: 0
    });
  };

  const handleClearAll = () => {
    setBuildings([]);
    setStops([]);
    setGeoCentre(null);
    setTags([]);
    setDogPath([]);
    setSelectedEntityId(null);
    setResetKey(k => k + 1);
    setLapMetrics({
      currentLapNumber: 1,
      currentLapAvgError: 0,
      lastCompletedLapAvgError: null,
      lifetimeAvgError: 0,
      currentLapSamples: 0,
      totalSamples: 0
    });
  };

  const handleDeleteSelected = () => {
    if (!selectedEntityId) return;
    setTags(prev => prev.filter(t => t.id !== selectedEntityId));
    setStops(prev => prev.filter(s => s.id !== selectedEntityId));
    setBuildings(prev => prev.filter(b => b.id !== selectedEntityId));
    if (geoCentre && geoCentre.id === selectedEntityId) {
      setGeoCentre(null);
    }
    setSelectedEntityId(null);
  };

  const handleToggleImuResting = (tagId: string) => {
    setTags(prev => prev.map(t =>
      t.id === tagId ? { ...t, imuResting: !t.imuResting } : t
    ));
  };

  return (
    <div className="relative w-screen h-screen overflow-hidden bg-[#0a0d14] text-slate-100 select-none">
      {/* 1. Top Control Bar */}
      <TopToolbar
        activeTool={activeTool}
        setActiveTool={setActiveTool}
        geoCentre={geoCentre}
        autoWalk={autoWalk}
        setAutoWalk={setAutoWalk}
        onResetPreset={handleResetPreset}
        onClearAll={handleClearAll}
        pathNodeCount={dogPath.length}
        onClearPath={() => setDogPath([])}
      />

      {/* 2. Top-Left Pure Real-Time Telemetry HUD */}
      <TelemetryHUD
        trilaterationResults={trilaterationResults}
        tier1Count={stops.filter(s => s.tier === 1).length}
        tier2Count={stops.filter(s => s.tier === 2).length}
        dogCount={tags.length}
        lapMetrics={lapMetrics}
      />

      {/* 3. Top-Right Sliders & Simulation Controls */}
      <ConfigPanel
        config={config}
        setConfig={setConfig}
      />

      {/* 3. Main Interactive Canvas Engine */}
      <SimulationCanvas
        activeTool={activeTool}
        setActiveTool={setActiveTool}
        config={config}
        buildings={buildings}
        setBuildings={setBuildings}
        stops={stops}
        setStops={setStops}
        geoCentre={geoCentre}
        setGeoCentre={setGeoCentre}
        tags={tags}
        setTags={setTags}
        dogPath={dogPath}
        setDogPath={setDogPath}
        autoWalk={autoWalk}
        selectedEntityId={selectedEntityId}
        setSelectedEntityId={setSelectedEntityId}
        onTrilaterationUpdate={setTrilaterationResults}
        onLapMetricsUpdate={setLapMetrics}
        resetKey={resetKey}
      />

      {/* 4. Bottom Real-Time Telemetry Logs */}
      <TelemetryFeed
        results={trilaterationResults}
        tags={tags}
        selectedTagId={selectedEntityId}
        onToggleImuResting={handleToggleImuResting}
        onDeleteSelected={handleDeleteSelected}
        hasSelection={!!selectedEntityId}
      />
    </div>
  );
};

export default App;
