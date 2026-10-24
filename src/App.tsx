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
import { resetDogTrajectory, resetAllTrackingState } from './engine/multilateration';

export const App: React.FC = () => {
  const [activeTool, setActiveTool] = useState<ActiveTool>('SELECT_MOVE');
  const [autoWalk, setAutoWalk] = useState<boolean>(true);
  const [selectedEntityId, setSelectedEntityId] = useState<string | null>(null);
  const [resetKey, setResetKey] = useState<number>(0);
  const [isConfigOpen, setIsConfigOpen] = useState<boolean>(false);

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
    anchorRadiusMeters: 90,
    tier1RadiusMeters: 90,
    tier2RadiusMeters: 90,
    noiseStdDevDb: 3.0,
    wallAttenuationDb: 12,
    pathLossExponent: 3.0,
    pixelsPerMeter: 2.8,
    autoWalkSpeed: 2.5, // Nominal K9 Patrol Trot (2.5 m/s = 9.0 km/h)
    showRFRipples: true,
    showBackhaulBeams: true,
    showCoverageHalos: false,
    showTrueDoggos: true,
    showEstimatedLocations: true,
    showHeadingVisionCone: true,
    showHistoricalBreadcrumbs: true,
    showGhostCandidates: false,
    imuSensorType: 'ECOMPASS_9DOF',
    disambiguationMode: 'QUADRATIC_MAP'
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
    resetAllTrackingState();
    preset.tags.forEach(t => resetDogTrajectory(t.id, { x: t.x, y: t.y }));
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
    resetAllTrackingState();
    preset.tags.forEach(t => resetDogTrajectory(t.id, { x: t.x, y: t.y }));
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
    resetAllTrackingState();
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
    <div className="relative w-screen h-screen overflow-hidden bg-[#fbf1c7] text-[#282828] select-none font-mono">
      {/* 1. Top Control Bar (Master Switchboard) */}
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
        isConfigOpen={isConfigOpen}
        onToggleConfig={() => setIsConfigOpen(prev => !prev)}
      />

      {/* 2. Top-Left Live Telemetry HUD */}
      <TelemetryHUD
        trilaterationResults={trilaterationResults}
        anchorCount={stops.length}
        dogCount={tags.length}
        lapMetrics={lapMetrics}
      />

      {/* 3. Top-Right Calibration Rack */}
      <ConfigPanel
        config={config}
        setConfig={setConfig}
        isOpen={isConfigOpen}
        onToggle={() => setIsConfigOpen(prev => !prev)}
      />

      {/* 4. Center Plan Position Indicator (PPI) Radar Scope Canvas */}
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

      {/* 5. Bottom Military Teleprinter / Intercept Feed */}
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
