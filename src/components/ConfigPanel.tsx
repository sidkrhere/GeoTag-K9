import React, { useState } from 'react';
import { SimulationConfig } from '../types/simulation';

interface ConfigPanelProps {
  config: SimulationConfig;
  setConfig: React.Dispatch<React.SetStateAction<SimulationConfig>>;
}

export const ConfigPanel: React.FC<ConfigPanelProps> = ({
  config,
  setConfig
}) => {
  const [isExpanded, setIsExpanded] = useState(true);

  return (
    <aside 
      aria-label="Analog Calibration Rack" 
      className="radar-panel fixed top-16 right-4 z-20 w-80 font-mono text-[#00ff41]"
    >
      {/* 1950s Stamped Faceplate Header */}
      <div 
        onClick={() => setIsExpanded(!isExpanded)}
        className="panel-stamped-header cursor-pointer select-none"
      >
        <div className="flex items-center gap-2">
          <span className="text-xs font-bold tracking-widest text-[#00f2fe] glow-cyan">
            CALIBRATION RACK // AN-CAL-4
          </span>
        </div>
        <button className="text-[#00aa33] hover:text-[#00ff41] text-xs font-bold" aria-label="Toggle Panel">
          {isExpanded ? '[-]' : '[+]'}
        </button>
      </div>

      {isExpanded && (
        <div className="p-3.5 space-y-3.5 max-h-[calc(100vh-140px)] overflow-y-auto text-[11px] bg-[#020b04]">
          {/* Section 1: Beacon Range Potentiometers (Cyan for T1, Orange for T2) */}
          <div className="space-y-2.5">
            <div className="text-[10px] text-[#00aa33] font-bold tracking-wider">
              --- BEACON TRANSMISSION RADII ---
            </div>

            {/* Tier 1 Mast Radius - Electric Cyan */}
            <div className="space-y-1">
              <div className="flex justify-between text-xs">
                <span className="text-[#00f2fe] font-semibold">T-1 SOLAR MAST:</span>
                <span className="text-[#00f2fe] font-bold glow-cyan">{config.tier1RadiusMeters} M</span>
              </div>
              <input
                type="range"
                className="slider-cyan"
                min="30"
                max="180"
                step="5"
                value={config.tier1RadiusMeters}
                onChange={(e) => setConfig(prev => ({ ...prev, tier1RadiusMeters: Number(e.target.value) }))}
              />
              <span className="text-[9px] text-[#007780] block">HIGH MAST SOLAR POLE (LoRa BACKHAUL)</span>
            </div>

            {/* Tier 2 Aux Radius - Amber/Orange */}
            <div className="space-y-1">
              <div className="flex justify-between text-xs">
                <span className="text-[#ff9900] font-semibold">T-2 AUX RELAY:</span>
                <span className="text-[#ff9900] font-bold glow-orange">{config.tier2RadiusMeters} M</span>
              </div>
              <input
                type="range"
                className="slider-orange"
                min="15"
                max="90"
                step="5"
                value={config.tier2RadiusMeters}
                onChange={(e) => setConfig(prev => ({ ...prev, tier2RadiusMeters: Number(e.target.value) }))}
              />
              <span className="text-[9px] text-[#804c00] block">GROUND LEVEL AUXILIARY BEACON</span>
            </div>
          </div>

          {/* Section 2: RF Channel Physics & Noise (Orange for Noise, Red for Wall) */}
          <div className="space-y-2.5 pt-2 border-t border-[#003310]">
            <div className="text-[10px] text-[#00aa33] font-bold tracking-wider">
              --- RF CHANNEL ATTENUATION & NOISE ---
            </div>

            {/* Noise StdDev - Orange */}
            <div className="space-y-1">
              <div className="flex justify-between text-xs">
                <span className="text-[#ff9900]">CHANNEL NOISE POT:</span>
                <span className="text-[#ff9900] font-bold">{config.noiseStdDevDb} dB</span>
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
              <span className="text-[9px] text-[#804c00] block">MULTIPATH, SHADOWING & NOISE FLUX</span>
            </div>

            {/* Building Wall Attenuation - Alert Red */}
            <div className="space-y-1">
              <div className="flex justify-between text-xs">
                <span className="text-[#ff3b30] font-semibold">WALL ATTENUATION POT:</span>
                <span className="text-[#ff3b30] font-bold glow-red">{config.wallAttenuationDb} dB/WALL</span>
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
              <span className="text-[9px] text-[#801d18] block">RF PENETRATION LOSS IN DRAWN STRUCTURES</span>
            </div>
          </div>

          {/* Section 3: Target Patrol Speed & Stepped Gears (Phosphor Green) */}
          <div className="space-y-2 pt-2 border-t border-[#003310]">
            <div className="text-[10px] text-[#00aa33] font-bold tracking-wider">
              --- TARGET PATROL DYNAMICS ---
            </div>

            <div className="flex justify-between items-baseline text-xs">
              <span className="text-[#00ff41]">PATROL VELOCITY:</span>
              <span className="text-[#00ff41] font-bold glow-green">
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

          {/* Section 4: CRT Scope Visual Filter Toggles */}
          <div className="space-y-2 pt-2 border-t border-[#003310]">
            <div className="text-[10px] text-[#00aa33] font-bold tracking-wider">
              --- SCOPE DISPLAY FILTERS ---
            </div>

            <div className="grid grid-cols-2 gap-1.5 text-xs">
              <button
                onClick={() => setConfig(prev => ({ ...prev, showCoverageHalos: !prev.showCoverageHalos }))}
                className={`radar-key radar-key-cyan text-[10px] justify-between ${config.showCoverageHalos ? 'radar-key-active' : ''}`}
              >
                <span>BEACON HALOS</span>
                <span>{config.showCoverageHalos ? '[ON]' : '[OFF]'}</span>
              </button>

              <button
                onClick={() => setConfig(prev => ({ ...prev, showRFRipples: !prev.showRFRipples }))}
                className={`radar-key radar-key-orange text-[10px] justify-between ${config.showRFRipples ? 'radar-key-active' : ''}`}
              >
                <span>RF STROBES</span>
                <span>{config.showRFRipples ? '[ON]' : '[OFF]'}</span>
              </button>

              <button
                onClick={() => setConfig(prev => ({ ...prev, showBackhaulBeams: !prev.showBackhaulBeams }))}
                className={`radar-key radar-key-cyan text-[10px] justify-between ${config.showBackhaulBeams ? 'radar-key-active' : ''}`}
              >
                <span>LORA BEAMS</span>
                <span>{config.showBackhaulBeams ? '[ON]' : '[OFF]'}</span>
              </button>

              <button
                onClick={() => setConfig(prev => ({ ...prev, showEstimatedLocations: !prev.showEstimatedLocations }))}
                className={`radar-key radar-key-green text-[10px] justify-between ${config.showEstimatedLocations ? 'radar-key-active' : ''}`}
              >
                <span>TRACK RETICLE</span>
                <span>{config.showEstimatedLocations ? '[ON]' : '[OFF]'}</span>
              </button>

              <button
                onClick={() => setConfig(prev => ({ ...prev, showHeadingVisionCone: !prev.showHeadingVisionCone }))}
                className={`radar-key radar-key-cyan text-[10px] justify-between col-span-2 ${config.showHeadingVisionCone ? 'radar-key-active' : ''}`}
              >
                <span>FORWARD SENSOR CONE (60° FOV)</span>
                <span>{config.showHeadingVisionCone ? '[ON]' : '[OFF]'}</span>
              </button>

              <button
                onClick={() => setConfig(prev => ({ ...prev, showHistoricalBreadcrumbs: !prev.showHistoricalBreadcrumbs }))}
                className={`radar-key radar-key-green text-[10px] justify-between col-span-2 ${config.showHistoricalBreadcrumbs ? 'radar-key-active' : ''}`}
              >
                <span>PHOSPHOR TRAIL (15 FIXES)</span>
                <span>{config.showHistoricalBreadcrumbs ? '[ON]' : '[OFF]'}</span>
              </button>
            </div>
          </div>

          {/* Section 5: Dorsal Collar Heading Sensor */}
          <div className="space-y-2 pt-2 border-t border-[#003310]">
            <div className="text-[10px] text-[#00aa33] font-bold tracking-wider">
              --- DORSAL SENSOR CONFIG ---
            </div>
            <div className="grid grid-cols-3 gap-1 text-[10px]">
              <button
                onClick={() => setConfig(prev => ({ ...prev, imuSensorType: 'ECOMPASS_9DOF' }))}
                className={`radar-key radar-key-cyan text-[10px] py-1 ${config.imuSensorType === 'ECOMPASS_9DOF' ? 'radar-key-active' : ''}`}
              >
                E-COMPASS
              </button>
              <button
                onClick={() => setConfig(prev => ({ ...prev, imuSensorType: 'IMU_6DOF' }))}
                className={`radar-key radar-key-cyan text-[10px] py-1 ${config.imuSensorType === 'IMU_6DOF' ? 'radar-key-active' : ''}`}
              >
                6-DOF IMU
              </button>
              <button
                onClick={() => setConfig(prev => ({ ...prev, imuSensorType: 'GPS_RTK_BENCHMARK' }))}
                className={`radar-key radar-key-cyan text-[10px] py-1 ${config.imuSensorType === 'GPS_RTK_BENCHMARK' ? 'radar-key-active' : ''}`}
              >
                RTK BENCH
              </button>
            </div>
            <p className="text-[9px] text-[#007780] leading-tight">
              Dorsal spine heading locks forward vector to resolve 2-anchor baseline mirror ambiguities.
            </p>
          </div>
        </div>
      )}
    </aside>
  );
};
