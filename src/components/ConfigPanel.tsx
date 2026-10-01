import React, { useState } from 'react';
import { 
  Sliders, 
  Layers, 
  Activity, 
  ChevronDown, 
  ChevronUp, 
  Eye, 
  EyeOff, 
  Zap,
  ShieldAlert,
  Compass,
  Footprints
} from 'lucide-react';
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
      aria-label="Simulation Controls & Sliders" 
      className="glass-panel fixed top-20 right-4 z-20 w-80 rounded-2xl border border-white/10 shadow-2xl transition-all duration-300 overflow-hidden"
    >
      {/* Header Bar */}
      <div 
        onClick={() => setIsExpanded(!isExpanded)}
        className="flex items-center justify-between px-4 py-3 bg-white/5 border-b border-white/10 cursor-pointer hover:bg-white/10 transition-colors"
      >
        <div className="flex items-center gap-2 text-white">
          <Sliders size={16} className="text-cyan-400" />
          <h2 className="font-display font-semibold text-xs tracking-wider uppercase">
            Simulation Controls
          </h2>
        </div>
        <button className="text-slate-400 hover:text-white" aria-label="Toggle Controls Panel">
          {isExpanded ? <ChevronUp size={16} /> : <ChevronDown size={16} />}
        </button>
      </div>

      {isExpanded && (
        <div className="p-4 space-y-4 max-h-[calc(100vh-140px)] overflow-y-auto">
          {/* Section 1: Radii Sliders */}
          <div className="space-y-3">
            <h3 className="text-[11px] font-semibold text-slate-300 uppercase tracking-wider flex items-center gap-1.5">
              <Activity size={13} className="text-cyan-400" />
              GeoStop Ping Radii
            </h3>

            {/* Tier 1 Radius Slider */}
            <div className="space-y-1">
              <div className="flex justify-between text-xs">
                <span className="text-cyan-300 font-medium flex items-center gap-1">
                  <span className="w-2 h-2 rounded-full bg-cyan-400 inline-block" />
                  Tier 1 Pole Radius:
                </span>
                <span className="font-mono text-cyan-300 font-bold">{config.tier1RadiusMeters} m</span>
              </div>
              <input
                type="range"
                min="30"
                max="180"
                step="5"
                value={config.tier1RadiusMeters}
                onChange={(e) => setConfig(prev => ({ ...prev, tier1RadiusMeters: Number(e.target.value) }))}
              />
              <span className="text-[10px] text-slate-500 block">High pole mast with solar & LoRa backhaul</span>
            </div>

            {/* Tier 2 Radius Slider */}
            <div className="space-y-1">
              <div className="flex justify-between text-xs">
                <span className="text-amber-300 font-medium flex items-center gap-1">
                  <span className="w-2 h-2 rounded-full bg-amber-400 inline-block" />
                  Tier 2 Eatery Radius:
                </span>
                <span className="font-mono text-amber-300 font-bold">{config.tier2RadiusMeters} m</span>
              </div>
              <input
                type="range"
                min="15"
                max="90"
                step="5"
                value={config.tier2RadiusMeters}
                onChange={(e) => setConfig(prev => ({ ...prev, tier2RadiusMeters: Number(e.target.value) }))}
              />
              <span className="text-[10px] text-slate-500 block">Eateries, canteens, and hostel entrances</span>
            </div>
          </div>

          {/* Section 2: RF Channel Physics & Noise */}
          <div className="space-y-3 pt-2 border-t border-white/5">
            <h3 className="text-[11px] font-semibold text-slate-300 uppercase tracking-wider flex items-center gap-1.5">
              <Zap size={13} className="text-amber-400" />
              RF Channel & Physics
            </h3>

            {/* Gaussian Noise Slider */}
            <div className="space-y-1">
              <div className="flex justify-between text-xs">
                <span className="text-slate-300">Channel Noise (StdDev):</span>
                <span className="font-mono text-white font-bold">{config.noiseStdDevDb} dB</span>
              </div>
              <input
                type="range"
                min="0"
                max="12"
                step="0.5"
                value={config.noiseStdDevDb}
                onChange={(e) => setConfig(prev => ({ ...prev, noiseStdDevDb: Number(e.target.value) }))}
              />
              <span className="text-[10px] text-slate-500 block">Simulates cars, multipath, and dog body shadowing</span>
            </div>

            {/* Building Wall Attenuation */}
            <div className="space-y-1">
              <div className="flex justify-between text-xs">
                <span className="text-slate-300 flex items-center gap-1">
                  <ShieldAlert size={12} className="text-rose-400" />
                  Wall Attenuation:
                </span>
                <span className="font-mono text-rose-300 font-bold">{config.wallAttenuationDb} dB/wall</span>
              </div>
              <input
                type="range"
                min="0"
                max="25"
                step="1"
                value={config.wallAttenuationDb}
                onChange={(e) => setConfig(prev => ({ ...prev, wallAttenuationDb: Number(e.target.value) }))}
              />
              <span className="text-[10px] text-slate-500 block">Signal drop when beam penetrates drawn buildings</span>
            </div>
          </div>

          {/* Section 3: Canine Motion Dynamics */}
          <div className="space-y-3 pt-2 border-t border-white/5">
            <h3 className="text-[11px] font-semibold text-slate-300 uppercase tracking-wider flex items-center gap-1.5">
              <Footprints size={13} className="text-emerald-400" />
              Canine Wander Speed
            </h3>

            <div className="space-y-1">
              <div className="flex justify-between text-xs">
                <span className="text-slate-300">Simulation Walk Speed:</span>
                <span className="font-mono text-emerald-400 font-bold">{config.autoWalkSpeed}x</span>
              </div>
              <input
                type="range"
                min="0.5"
                max="4.0"
                step="0.1"
                value={config.autoWalkSpeed}
                onChange={(e) => setConfig(prev => ({ ...prev, autoWalkSpeed: Number(e.target.value) }))}
              />
              <span className="text-[10px] text-slate-500 block">DVB-KF velocity cap scales automatically with speed</span>
            </div>
          </div>

          {/* Section 4: Visualization Layers */}
          <div className="space-y-2 pt-2 border-t border-white/5">
            <h3 className="text-[11px] font-semibold text-slate-300 uppercase tracking-wider flex items-center gap-1.5">
              <Layers size={13} className="text-purple-400" />
              Display Layers
            </h3>

            <div className="grid grid-cols-2 gap-2 text-xs">
              <button
                onClick={() => setConfig(prev => ({ ...prev, showCoverageHalos: !prev.showCoverageHalos }))}
                className={`flex items-center justify-between p-2 rounded-xl border transition-colors ${
                  config.showCoverageHalos 
                    ? 'bg-cyan-500/10 border-cyan-500/30 text-cyan-300' 
                    : 'bg-white/5 border-white/5 text-slate-400'
                }`}
              >
                <span>Coverage Halos</span>
                {config.showCoverageHalos ? <Eye size={14} /> : <EyeOff size={14} />}
              </button>

              <button
                onClick={() => setConfig(prev => ({ ...prev, showRFRipples: !prev.showRFRipples }))}
                className={`flex items-center justify-between p-2 rounded-xl border transition-colors ${
                  config.showRFRipples 
                    ? 'bg-amber-500/10 border-amber-500/30 text-amber-300' 
                    : 'bg-white/5 border-white/5 text-slate-400'
                }`}
              >
                <span>RF Signals</span>
                {config.showRFRipples ? <Eye size={14} /> : <EyeOff size={14} />}
              </button>

              <button
                onClick={() => setConfig(prev => ({ ...prev, showBackhaulBeams: !prev.showBackhaulBeams }))}
                className={`flex items-center justify-between p-2 rounded-xl border transition-colors ${
                  config.showBackhaulBeams 
                    ? 'bg-purple-500/10 border-purple-500/30 text-purple-300' 
                    : 'bg-white/5 border-white/5 text-slate-400'
                }`}
              >
                <span>LoRa Backhaul</span>
                {config.showBackhaulBeams ? <Eye size={14} /> : <EyeOff size={14} />}
              </button>

              <button
                onClick={() => setConfig(prev => ({ ...prev, showEstimatedLocations: !prev.showEstimatedLocations }))}
                className={`flex items-center justify-between p-2 rounded-xl border transition-colors ${
                  config.showEstimatedLocations 
                    ? 'bg-emerald-500/10 border-emerald-500/30 text-emerald-300' 
                    : 'bg-white/5 border-white/5 text-slate-400'
                }`}
              >
                <span>Calculated 🎯</span>
                {config.showEstimatedLocations ? <Eye size={14} /> : <EyeOff size={14} />}
              </button>

              <button
                onClick={() => setConfig(prev => ({ ...prev, showHeadingVisionCone: !prev.showHeadingVisionCone }))}
                className={`flex items-center justify-between p-2 rounded-xl border transition-colors col-span-2 ${
                  config.showHeadingVisionCone 
                    ? 'bg-cyan-500/10 border-cyan-500/30 text-cyan-300' 
                    : 'bg-white/5 border-white/5 text-slate-400'
                }`}
              >
                <span>Canine Vision Cone 🧭 (60° FOV)</span>
                {config.showHeadingVisionCone ? <Eye size={14} /> : <EyeOff size={14} />}
              </button>

              <button
                onClick={() => setConfig(prev => ({ ...prev, showHistoricalBreadcrumbs: !prev.showHistoricalBreadcrumbs }))}
                className={`flex items-center justify-between p-2 rounded-xl border transition-colors col-span-2 ${
                  config.showHistoricalBreadcrumbs 
                    ? 'bg-emerald-500/10 border-emerald-500/30 text-emerald-300' 
                    : 'bg-white/5 border-white/5 text-slate-400'
                }`}
              >
                <span>History Breadcrumbs Trail (15 Fixes)</span>
                {config.showHistoricalBreadcrumbs ? <Eye size={14} /> : <EyeOff size={14} />}
              </button>
            </div>
          </div>

          {/* Section 5: Collar Sensor Fusion Profile */}
          <div className="space-y-2 pt-2 border-t border-white/5">
            <h3 className="text-[11px] font-semibold text-slate-300 uppercase tracking-wider flex items-center justify-between">
              <span className="flex items-center gap-1.5">
                <Compass size={13} className="text-cyan-400" />
                Dorsal Sensor Profile
              </span>
              <span className="text-[10px] text-cyan-300 font-mono">9-DoF Active</span>
            </h3>
            <div className="grid grid-cols-3 gap-1 text-[10px] font-mono">
              <button
                onClick={() => setConfig(prev => ({ ...prev, imuSensorType: 'ECOMPASS_9DOF' }))}
                className={`py-1.5 px-2 rounded-lg border text-center transition-all ${
                  config.imuSensorType === 'ECOMPASS_9DOF'
                    ? 'bg-cyan-500/20 border-cyan-500/40 text-cyan-300 font-bold'
                    : 'bg-white/5 border-white/5 text-slate-400 hover:text-white'
                }`}
              >
                e-Compass
              </button>
              <button
                onClick={() => setConfig(prev => ({ ...prev, imuSensorType: 'IMU_6DOF' }))}
                className={`py-1.5 px-2 rounded-lg border text-center transition-all ${
                  config.imuSensorType === 'IMU_6DOF'
                    ? 'bg-cyan-500/20 border-cyan-500/40 text-cyan-300 font-bold'
                    : 'bg-white/5 border-white/5 text-slate-400 hover:text-white'
                }`}
              >
                6-DoF IMU
              </button>
              <button
                onClick={() => setConfig(prev => ({ ...prev, imuSensorType: 'GPS_RTK_BENCHMARK' }))}
                className={`py-1.5 px-2 rounded-lg border text-center transition-all ${
                  config.imuSensorType === 'GPS_RTK_BENCHMARK'
                    ? 'bg-cyan-500/20 border-cyan-500/40 text-cyan-300 font-bold'
                    : 'bg-white/5 border-white/5 text-slate-400 hover:text-white'
                }`}
              >
                RTK Ref
              </button>
            </div>
            <p className="text-[10px] text-slate-400 leading-tight">
              Collar dorsal mount locks to canine spine forward vector, disambiguating 2-anchor baseline mirrors.
            </p>
          </div>
        </div>
      )}
    </aside>
  );
};
