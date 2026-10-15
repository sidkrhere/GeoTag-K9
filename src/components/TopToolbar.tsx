import React from 'react';
import { ActiveTool, GeoCentre } from '../types/simulation';

interface TopToolbarProps {
  activeTool: ActiveTool;
  setActiveTool: (tool: ActiveTool) => void;
  geoCentre: GeoCentre | null;
  autoWalk: boolean;
  setAutoWalk: (autoWalk: boolean | ((prev: boolean) => boolean)) => void;
  onResetPreset: () => void;
  onClearAll: () => void;
  pathNodeCount: number;
  onClearPath: () => void;
}

export const TopToolbar: React.FC<TopToolbarProps> = ({
  activeTool,
  setActiveTool,
  geoCentre,
  autoWalk,
  setAutoWalk,
  onResetPreset,
  onClearAll,
  pathNodeCount,
  onClearPath
}) => {
  const tools: {
    id: ActiveTool;
    code: string;
    label: string;
    glyph: string;
    badge?: string;
    colorClass: string;
    desc: string;
  }[] = [
    {
      id: 'SELECT_MOVE',
      code: 'F1',
      glyph: '⌖',
      label: 'SELECT',
      colorClass: 'radar-key-cyan',
      desc: 'Inspect target tracks, beacons, or obstacle vertices'
    },
    {
      id: 'DRAW_BUILDING',
      code: 'F2',
      glyph: '▨',
      label: 'PLOT WALL',
      badge: 'RF ATTEN',
      colorClass: 'radar-key-red',
      desc: 'Plot RF obstacle boundaries; click origin to close'
    },
    {
      id: 'TRACE_PATH',
      code: 'F3',
      glyph: '☡',
      label: 'TRACE RT',
      badge: pathNodeCount > 0 ? `${pathNodeCount}N` : undefined,
      colorClass: 'radar-key-green',
      desc: 'Plot dog patrol turn waypoints for trajectory follow'
    },
    {
      id: 'ADD_TIER1_STOP',
      code: 'F4',
      glyph: '▲',
      label: 'T-1 MAST',
      colorClass: 'radar-key-cyan',
      desc: 'Deploy High-Power Solar Radar Mast (LoRa backhaul)'
    },
    {
      id: 'ADD_TIER2_STOP',
      code: 'F5',
      glyph: '◈',
      label: 'T-2 AUX',
      colorClass: 'radar-key-orange',
      desc: 'Deploy Ground Auxiliary Relay Beacon'
    },
    {
      id: 'ADD_GEOCENTRE',
      code: 'F6',
      glyph: '⊕',
      label: 'HQ BASE',
      badge: geoCentre ? '1/1' : '0/1',
      colorClass: 'radar-key-cyan',
      desc: 'Central Command Bunker Localization Hub'
    }
  ];

  return (
    <header className="radar-panel fixed top-3 left-4 right-4 z-30 flex items-center justify-between px-3 py-2 font-mono">
      {/* 1950s Stamped Identification Plate */}
      <div className="flex items-center gap-3 pr-4 border-r border-[#00551a]">
        <div className="w-8 h-8 border border-[#00f2fe] bg-[#001418] flex items-center justify-center text-[#00f2fe] font-bold text-sm glow-cyan">
          K9
        </div>
        <div>
          <div className="flex items-center gap-2">
            <span className="text-xs font-bold tracking-widest text-[#00f2fe] glow-cyan">
              RADAR PLOTTER MK-52
            </span>
            <span className="text-[9px] px-1.5 py-0.2 bg-[#001f22] border border-[#007780] text-[#00f2fe]">
              AN/FPS-52
            </span>
          </div>
          <p className="text-[10px] text-[#00cc3a] hidden sm:block">
            IIT-ROPAR SECTOR COMMAND // 2.4GHz + LoRa
          </p>
        </div>
      </div>

      {/* Main Mode / Tool Switchboard */}
      <nav aria-label="Radar Mode Selector" className="flex items-center gap-1.5 overflow-x-auto py-0.5 px-1">
        {tools.map((t) => {
          const isActive = activeTool === t.id;
          return (
            <button
              key={t.id}
              onClick={() => setActiveTool(t.id)}
              title={t.desc}
              className={`radar-key ${t.colorClass} ${isActive ? 'radar-key-active' : ''}`}
            >
              <span className="opacity-80 text-[10px]">{t.glyph}</span>
              <span>{t.label}</span>
              {t.badge && (
                <span className={`text-[9px] px-1 border ${
                  isActive 
                    ? 'border-[#010602] bg-[#010602] text-current' 
                    : 'border-current bg-black/40'
                }`}>
                  {t.badge}
                </span>
              )}
            </button>
          );
        })}
      </nav>

      {/* Rotary Action Switches & System Commands */}
      <div className="flex items-center gap-1.5 pl-3 border-l border-[#00551a]">
        {/* Sweep / Auto-Walk Toggle */}
        <button
          onClick={() => setAutoWalk((prev: boolean) => !prev)}
          title={autoWalk ? 'Halt autonomous dog patrol' : 'Engage autonomous dog patrol'}
          className={`radar-key radar-key-green ${autoWalk ? 'radar-key-active' : ''}`}
        >
          <span>{autoWalk ? '▶ PATROL: RUN' : '⏸ PATROL: HALT'}</span>
        </button>

        {/* Clear Route Button */}
        {pathNodeCount > 0 && (
          <button
            onClick={onClearPath}
            title="Purge plotted patrol waypoints"
            className="radar-key radar-key-red text-[10px]"
          >
            <span>PURGE RT</span>
          </button>
        )}

        {/* IIT Ropar Preset Reset */}
        <button
          onClick={onResetPreset}
          title="Re-zero to IIT Ropar Standard Grid Layout"
          className="radar-key radar-key-orange text-[10px]"
        >
          <span>PRESET: ROPAR</span>
        </button>

        {/* Zero Scope / Clear All */}
        <button
          onClick={onClearAll}
          title="Emergency Master Clear / Zero Scope"
          className="radar-key radar-key-red text-[10px]"
        >
          <span>ZERO SCOPE</span>
        </button>
      </div>
    </header>
  );
};
