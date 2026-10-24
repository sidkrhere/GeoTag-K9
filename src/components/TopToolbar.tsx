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
  isConfigOpen: boolean;
  onToggleConfig: () => void;
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
  onClearPath,
  isConfigOpen,
  onToggleConfig
}) => {
  const tools: {
    id: ActiveTool;
    label: string;
    glyph: string;
    badge?: string;
    colorClass: string;
    desc: string;
  }[] = [
    {
      id: 'SELECT_MOVE',
      glyph: '↖',
      label: 'SELECT',
      colorClass: 'radar-key-cyan',
      desc: 'Inspect or move dog, beacons, or building vertices'
    },
    {
      id: 'DRAW_BUILDING',
      glyph: '▱',
      label: 'WALL',
      badge: '12dB',
      colorClass: 'radar-key-red',
      desc: 'Draw RF attenuation walls; click start point to close'
    },
    {
      id: 'TRACE_PATH',
      glyph: '∿',
      label: 'ROUTE',
      badge: pathNodeCount > 0 ? `${pathNodeCount}N` : undefined,
      colorClass: 'radar-key-green',
      desc: 'Plot dog patrol waypoints'
    },
    {
      id: 'ADD_TIER1_STOP',
      glyph: '▲',
      label: 'ANCHOR',
      colorClass: 'radar-key-cyan',
      desc: 'Deploy UWB Tracking Anchor Node'
    },
    {
      id: 'ADD_GEOCENTRE',
      glyph: '⊕',
      label: 'GATEWAY',
      badge: geoCentre ? '1/1' : '0/1',
      colorClass: 'radar-key-cyan',
      desc: 'Central Base Station Hub (Admin Block)'
    }
  ];

  return (
    <header className="radar-panel fixed top-3 left-4 right-4 z-30 flex items-center justify-between px-3 py-1.5 font-mono">
      {/* Brand Identification */}
      <div className="flex items-center gap-2.5 pr-3 border-r border-[#d5c4a1]">
        <div className="w-7 h-7 border border-[#458588] bg-[#ebf2f2] flex items-center justify-center text-[#076678] font-bold text-xs rounded">
          K9
        </div>
        <div>
          <div className="flex items-center gap-1.5">
            <span className="text-xs font-bold tracking-wider text-[#282828]">
              GeoTag-K9
            </span>
            <span className="text-[9px] px-1 py-0.2 bg-[#ebdbb2] text-[#7c6f64] border border-[#d5c4a1] rounded font-semibold">
              IIT ROPAR
            </span>
          </div>
          <p className="text-[9px] text-[#7c6f64] hidden sm:block">
            UWB Multilateration + 9DoF E-Compass Tracking
          </p>
        </div>
      </div>

      {/* Main Mode / Tool Switchboard */}
      <nav aria-label="Tool Selector" className="flex items-center gap-1 overflow-x-auto py-0.5 px-1">
        {tools.map((t) => {
          const isActive = activeTool === t.id;
          return (
            <button
              key={t.id}
              onClick={() => setActiveTool(t.id)}
              title={t.desc}
              className={`radar-key ${t.colorClass} ${isActive ? 'radar-key-active' : ''}`}
            >
              <span className="font-bold text-[11px]">{t.glyph}</span>
              <span>{t.label}</span>
              {t.badge && (
                <span className={`text-[8px] px-1 py-0.2 border rounded ${
                  isActive 
                    ? 'border-[#fbf1c7] bg-[#fbf1c7] text-[#282828]' 
                    : 'border-[#bdae93] bg-[#ebdbb2] text-[#504945]'
                }`}>
                  {t.badge}
                </span>
              )}
            </button>
          );
        })}
      </nav>

      {/* Control Action Buttons */}
      <div className="flex items-center gap-1.5 pl-3 border-l border-[#d5c4a1]">
        {/* Patrol Toggle */}
        <button
          onClick={() => setAutoWalk((prev: boolean) => !prev)}
          title={autoWalk ? 'Pause canine patrol movement' : 'Run autonomous canine patrol'}
          className={`radar-key radar-key-green ${autoWalk ? 'radar-key-active' : ''}`}
        >
          <span>{autoWalk ? '▶ PATROL' : '⏸ PAUSED'}</span>
        </button>

        {/* Clear Route */}
        {pathNodeCount > 0 && (
          <button
            onClick={onClearPath}
            title="Clear plotted patrol route"
            className="radar-key radar-key-red text-[10px]"
          >
            <span>CLEAR ROUTE</span>
          </button>
        )}

        {/* Reset Preset */}
        <button
          onClick={onResetPreset}
          title="Reset to default IIT Ropar Layout"
          className="radar-key text-[10px]"
        >
          <span>RESET</span>
        </button>

        {/* Clear All */}
        <button
          onClick={onClearAll}
          title="Clear all entities"
          className="radar-key radar-key-red text-[10px]"
        >
          <span>CLEAR ALL</span>
        </button>

        {/* Config Panel Toggle */}
        <button
          onClick={onToggleConfig}
          title="Toggle Simulation Config & Calibration Panel"
          className={`radar-key radar-key-orange text-[10px] ${isConfigOpen ? 'radar-key-active' : ''}`}
        >
          <span>⚙ CONFIG</span>
        </button>
      </div>
    </header>
  );
};
