import React, { useState } from 'react';
import { SimulationConfig } from '../types/simulation';

interface ConfigPanelProps {
  config: SimulationConfig;
  setConfig: React.Dispatch<React.SetStateAction<SimulationConfig>>;
  isOpen?: boolean;
  onToggle?: () => void;
}

export const ConfigPanel: React.FC<ConfigPanelProps> = ({
  config,
  setConfig,
  isOpen,
  onToggle
}) => {
  const [internalExpanded, setInternalExpanded] = useState(false);
  const isExpanded = isOpen !== undefined ? isOpen : internalExpanded;
  const handleToggle = onToggle || (() => setInternalExpanded(prev => !prev));

  if (!isExpanded) {
    return null; // Kept completely hidden when closed for maximum screen decluttering
  }

  return (
    <aside 
      aria-label="Configuration & Calibration Rack" 
      className="radar-panel fixed top-16 right-4 z-20 w-80 font-mono text-[#282828]"
    >
      {/* Header */}
      <div 
        onClick={handleToggle}
        className="panel-stamped-header cursor-pointer select-none"
      >
        <div className="flex items-center gap-2">
          <span className="text-xs font-bold tracking-wider text-[#282828]">
            SYSTEM CONFIGURATION & CALIBRATION
          </span>
        </div>
        <button className="text-[#7c6f64] hover:text-[#282828] text-xs font-bold px-1" aria-label="Close Panel">
          ✕
        </button>
      </div>

      <div className="p-3.5 space-y-3 max-h-[calc(100vh-140px)] overflow-y-auto text-[11px] bg-[#f2e5bc]">
        {/* Section 1: Anchor Range Sliders */}
        <div className="space-y-2">
          <div className="text-[10px] text-[#79740e] font-bold tracking-wider uppercase">
            --- Anchor Transmission Coverage ---
          </div>

          {/* Unified Anchor Coverage Radius */}
          <div className="space-y-1 p-2 bg-[#fbf1c7] rounded border border-[#d5c4a1]">
            <div className="flex justify-between text-xs">
              <span className="text-[#076678] font-bold">ANCHOR COVERAGE RADIUS:</span>
              <span className="text-[#076678] font-bold font-mono tabular-nums">{config.anchorRadiusMeters ?? 90} M</span>
            </div>
            <input
              type="range"
              className="slider-cyan"
              min="30"
              max="180"
              step="5"
              value={config.anchorRadiusMeters ?? 90}
              onChange={(e) => {
                const val = Number(e.target.value);
                setConfig(prev => ({
                  ...prev,
                  anchorRadiusMeters: val,
                  tier1RadiusMeters: val,
                  tier2RadiusMeters: val
                }));
              }}
            />
            <span className="text-[9px] text-[#7c6f64] block">UNIFIED UWB ANCHOR TRANSMISSION HORIZON (NOMINAL 90M)</span>
          </div>
        </div>

        {/* Section 2: RF Channel Physics & Noise */}
        <div className="space-y-2 pt-2 border-t border-[#d5c4a1]">
          <div className="text-[10px] text-[#79740e] font-bold tracking-wider uppercase">
            --- RF Channel Physics & Loss ---
          </div>

          {/* Noise StdDev */}
          <div className="space-y-1 p-2 bg-[#fbf1c7] rounded border border-[#d5c4a1]">
            <div className="flex justify-between text-xs">
              <span className="text-[#af3a03] font-bold">CHANNEL NOISE:</span>
              <span className="text-[#af3a03] font-bold font-mono tabular-nums">{config.noiseStdDevDb} dB</span>
            </div>
            <input
              type="range"
              className="slider-orange"
              min="0"
              max="12"
              step="0.5"
              value={config.noiseStdDevDb}
              onChange={(e) => setConfig(prev => ({ ...prev, noiseStdDevDb: Number(e.target.value) }))}
            />
            <span className="text-[9px] text-[#7c6f64] block">MULTIPATH, SHADOWING & NOISE FLUX</span>
          </div>

          {/* Building Wall Attenuation */}
          <div className="space-y-1 p-2 bg-[#fbf1c7] rounded border border-[#d5c4a1]">
            <div className="flex justify-between text-xs">
              <span className="text-[#cc241d] font-bold">WALL ATTENUATION:</span>
              <span className="text-[#cc241d] font-bold font-mono tabular-nums">{config.wallAttenuationDb} dB/WALL</span>
            </div>
            <input
              type="range"
              className="slider-red"
              min="0"
              max="25"
              step="1"
              value={config.wallAttenuationDb}
              onChange={(e) => setConfig(prev => ({ ...prev, wallAttenuationDb: Number(e.target.value) }))}
            />
            <span className="text-[9px] text-[#7c6f64] block">RF PENETRATION LOSS IN OBSTACLES</span>
          </div>
        </div>

        {/* Section 3: Target Patrol Speed */}
        <div className="space-y-2 pt-2 border-t border-[#d5c4a1]">
          <div className="text-[10px] text-[#79740e] font-bold tracking-wider uppercase">
            --- Target Dynamics & Patrol ---
          </div>

          <div className="p-2 bg-[#fbf1c7] rounded border border-[#d5c4a1] space-y-1.5">
            <div className="flex justify-between items-baseline text-xs">
              <span className="text-[#282828] font-semibold">PATROL SPEED:</span>
              <span className="text-[#79740e] font-bold font-mono tabular-nums">
                {(config.autoWalkSpeed ?? 2.5).toFixed(1)} M/S ({(((config.autoWalkSpeed ?? 2.5) * 3.6)).toFixed(1)} KM/H)
              </span>
            </div>

            <input
              type="range"
              min="0.8"
              max="12.0"
              step="0.1"
              value={config.autoWalkSpeed ?? 2.5}
              onChange={(e) => setConfig(prev => ({ ...prev, autoWalkSpeed: Number(e.target.value) }))}
            />

            {/* Stepped Selector Keys */}
            <div className="grid grid-cols-4 gap-1 pt-1">
              {[
                { label: 'WALK', spd: 1.2 },
                { label: 'TROT', spd: 2.5 },
                { label: 'CANTER', spd: 5.5 },
                { label: 'SPRINT', spd: 10.0 }
              ].map(preset => {
                const isActive = Math.abs((config.autoWalkSpeed ?? 2.5) - preset.spd) < 0.2;
                return (
                  <button
                    key={preset.label}
                    onClick={() => setConfig(prev => ({ ...prev, autoWalkSpeed: preset.spd }))}
                    className={`radar-key text-[10px] py-1 ${isActive ? 'radar-key-active' : ''}`}
                  >
                    <span>{preset.label}</span>
                  </button>
                );
              })}
            </div>
          </div>
        </div>

        {/* Section 4: Display & Visual Filters */}
        <div className="space-y-2 pt-2 border-t border-[#d5c4a1]">
          <div className="text-[10px] text-[#79740e] font-bold tracking-wider uppercase">
            --- Display & Visualization ---
          </div>

          <div className="grid grid-cols-2 gap-1.5 text-xs">
            <button
              onClick={() => setConfig(prev => ({ ...prev, showCoverageHalos: !prev.showCoverageHalos }))}
              className={`radar-key radar-key-cyan text-[10px] justify-between ${config.showCoverageHalos ? 'radar-key-active' : ''}`}
            >
              <span>ANCHOR HALOS</span>
              <span>{config.showCoverageHalos ? '[ON]' : '[OFF]'}</span>
            </button>

            <button
              onClick={() => setConfig(prev => ({ ...prev, showRFRipples: !prev.showRFRipples }))}
              className={`radar-key radar-key-orange text-[10px] justify-between ${config.showRFRipples ? 'radar-key-active' : ''}`}
            >
              <span>RF WAVEFRONTS</span>
              <span>{config.showRFRipples ? '[ON]' : '[OFF]'}</span>
            </button>

            <button
              onClick={() => setConfig(prev => ({ ...prev, showBackhaulBeams: !prev.showBackhaulBeams }))}
              className={`radar-key radar-key-cyan text-[10px] justify-between ${config.showBackhaulBeams ? 'radar-key-active' : ''}`}
            >
              <span>UPLINK BEAMS</span>
              <span>{config.showBackhaulBeams ? '[ON]' : '[OFF]'}</span>
            </button>

            <button
              onClick={() => setConfig(prev => ({ ...prev, showEstimatedLocations: !prev.showEstimatedLocations }))}
              className={`radar-key radar-key-orange text-[10px] justify-between ${config.showEstimatedLocations ? 'radar-key-active' : ''}`}
            >
              <span>TRACK RETICLE</span>
              <span>{config.showEstimatedLocations ? '[ON]' : '[OFF]'}</span>
            </button>

            {/* Ghost Candidate Visualization Toggle */}
            <button
              onClick={() => setConfig(prev => ({ ...prev, showGhostCandidates: !prev.showGhostCandidates }))}
              className={`radar-key text-[10px] justify-between col-span-2 ${config.showGhostCandidates ? 'radar-key-active' : ''}`}
            >
              <span>MIRROR HYPOTHESES (GHOST ECHOES)</span>
              <span>{config.showGhostCandidates ? '[SHOWN]' : '[HIDDEN]'}</span>
            </button>

            <button
              onClick={() => setConfig(prev => ({ ...prev, showHeadingVisionCone: !prev.showHeadingVisionCone }))}
              className={`radar-key radar-key-cyan text-[10px] justify-between col-span-2 ${config.showHeadingVisionCone ? 'radar-key-active' : ''}`}
            >
              <span>HEADING CONE (60° FOV)</span>
              <span>{config.showHeadingVisionCone ? '[ON]' : '[OFF]'}</span>
            </button>

            <button
              onClick={() => setConfig(prev => ({ ...prev, showHistoricalBreadcrumbs: !prev.showHistoricalBreadcrumbs }))}
              className={`radar-key radar-key-green text-[10px] justify-between col-span-2 ${config.showHistoricalBreadcrumbs ? 'radar-key-active' : ''}`}
            >
              <span>PATROL BREADCRUMBS</span>
              <span>{config.showHistoricalBreadcrumbs ? '[ON]' : '[OFF]'}</span>
            </button>
          </div>
        </div>

        {/* Section 5: 2-Anchor Disambiguation Mode */}
        <div className="space-y-2 pt-2 border-t border-[#d5c4a1]">
          <div className="text-[10px] text-[#79740e] font-bold tracking-wider uppercase">
            --- 2-Anchor Disambiguation ---
          </div>
          <div className="grid grid-cols-2 gap-1 text-[10px]">
            <button
              onClick={() => setConfig(prev => ({ ...prev, disambiguationMode: 'QUADRATIC_MAP' }))}
              className={`radar-key radar-key-cyan text-[10px] py-1 ${(!config.disambiguationMode || config.disambiguationMode === 'QUADRATIC_MAP') ? 'radar-key-active' : ''}`}
            >
              QUADRATIC MAP
            </button>
            <button
              onClick={() => setConfig(prev => ({ ...prev, disambiguationMode: 'HIERARCHICAL_GATE' }))}
              className={`radar-key radar-key-cyan text-[10px] py-1 ${config.disambiguationMode === 'HIERARCHICAL_GATE' ? 'radar-key-active' : ''}`}
            >
              SPATIAL GATE
            </button>
          </div>
          <p className="text-[9px] text-[#7c6f64] leading-tight">
            {(!config.disambiguationMode || config.disambiguationMode === 'QUADRATIC_MAP')
              ? 'Quadratic MAP: Continuous Bayesian cost (3d + 2d²); soft at crossings, crushes distant leaps.'
              : 'Hierarchical Gate: Closer candidate wins unconditionally if Δd > 1.5m; compass acts as tie-breaker.'}
          </p>
        </div>
      </div>
    </aside>
  );
};
