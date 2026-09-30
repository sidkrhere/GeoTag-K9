import React from 'react';
import { 
  Pointer, 
  SquareDashedBottomCode, 
  Radio, 
  Coffee, 
  Server, 
  Dog, 
  RotateCcw, 
  Play, 
  Pause, 
  School,
  Route,
  MapPinOff
} from 'lucide-react';
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
  const tools: { id: ActiveTool; label: string; icon: React.ReactNode; badge?: string; desc: string }[] = [
    {
      id: 'SELECT_MOVE',
      label: 'Select / Drag',
      icon: <Pointer size={18} />,
      desc: 'Move and inspect nodes, dogs, or building vertices'
    },
    {
      id: 'DRAW_BUILDING',
      label: 'Draw Building',
      icon: <SquareDashedBottomCode size={18} />,
      badge: 'Lasso',
      desc: 'Click canvas to drop wall vertices; click start node to close'
    },
    {
      id: 'TRACE_PATH',
      label: 'Trace Dog Path',
      icon: <Route size={18} className="text-emerald-400" />,
      badge: pathNodeCount > 0 ? `${pathNodeCount} Nodes` : 'New',
      desc: 'Click on canvas to drop turn nodes for Bruno to follow'
    },
    {
      id: 'ADD_TIER1_STOP',
      label: 'Tier 1 Pole',
      icon: <Radio size={18} className="text-cyan-400" />,
      desc: 'High mast pole anchor (Solar + LoRa backhaul)'
    },
    {
      id: 'ADD_TIER2_STOP',
      label: 'Tier 2 Eatery',
      icon: <Coffee size={18} className="text-amber-400" />,
      desc: 'Ground-level canteen/cafe anchor (Mains + Wi-Fi)'
    },
    {
      id: 'ADD_GEOCENTRE',
      label: 'GeoCentre',
      icon: <Server size={18} className="text-purple-400" />,
      badge: geoCentre ? '1/1 Placed' : '0/1',
      desc: 'Central localization server (Strictly max 1 instance)'
    }
  ];

  return (
    <header className="glass-panel fixed top-3 left-4 right-4 z-30 flex items-center justify-between px-4 py-2.5 rounded-2xl border border-white/10 shadow-2xl">
      {/* Brand & Project Info */}
      <div className="flex items-center gap-3 pr-4 border-r border-white/10">
        <div className="w-10 h-10 rounded-xl bg-gradient-to-tr from-cyan-500 to-blue-600 flex items-center justify-center shadow-lg shadow-cyan-500/20">
          <Dog size={22} className="text-white" />
        </div>
        <div>
          <div className="flex items-center gap-2">
            <h1 className="font-display font-bold text-base text-white tracking-wide">
              GeoTag <span className="text-cyan-400">K9</span>
            </h1>
            <span className="text-[10px] font-mono uppercase px-2 py-0.5 rounded-full bg-cyan-500/10 text-cyan-400 border border-cyan-500/20">
              IIT Ropar
            </span>
          </div>
          <p className="text-[11px] text-slate-400 font-sans hidden sm:block">
            Hybrid Localization & Network Simulator
          </p>
        </div>
      </div>

      {/* Main Mode / Tool Switcher */}
      <nav aria-label="Tool Switcher" className="flex items-center gap-1.5 overflow-x-auto py-1 px-2">
        {tools.map((t) => {
          const isActive = activeTool === t.id;
          return (
            <button
              key={t.id}
              onClick={() => setActiveTool(t.id)}
              title={t.desc}
              className={`flex items-center gap-2 px-3 py-1.5 rounded-xl text-xs font-medium transition-all duration-200 relative group cursor-pointer ${
                isActive
                  ? 'bg-cyan-500/20 text-cyan-300 border border-cyan-500/40 shadow-sm shadow-cyan-500/30'
                  : 'text-slate-300 hover:text-white hover:bg-white/5 border border-transparent'
              }`}
            >
              {t.icon}
              <span className="hidden md:inline">{t.label}</span>
              {t.badge && (
                <span className={`text-[10px] px-1.5 py-0.2 rounded font-mono ${
                  isActive ? 'bg-cyan-400/20 text-cyan-200' : 'bg-white/10 text-slate-400'
                }`}>
                  {t.badge}
                </span>
              )}
            </button>
          );
        })}
      </nav>

      {/* Action Controls & Presets */}
      <div className="flex items-center gap-2 pl-4 border-l border-white/10">
        <button
          onClick={() => setAutoWalk((prev: boolean) => !prev)}
          title={autoWalk ? 'Pause dog wandering' : 'Resume autonomous dog wandering'}
          className={`flex items-center gap-1.5 px-3 py-1.5 rounded-xl text-xs font-medium transition-colors cursor-pointer ${
            autoWalk 
              ? 'bg-emerald-500/20 text-emerald-300 border border-emerald-500/30'
              : 'bg-white/5 text-slate-400 hover:text-white hover:bg-white/10 border border-white/10'
          }`}
        >
          {autoWalk ? <Pause size={15} /> : <Play size={15} />}
          <span className="hidden lg:inline">{autoWalk ? 'Pause Walk' : 'Auto Walk'}</span>
        </button>

        {pathNodeCount > 0 && (
          <button
            onClick={onClearPath}
            title="Clear traced dog route"
            className="flex items-center gap-1 px-2.5 py-1.5 rounded-xl text-xs font-medium text-amber-300 bg-amber-500/10 hover:bg-amber-500/20 border border-amber-500/20 transition-colors cursor-pointer"
          >
            <MapPinOff size={14} />
            <span className="hidden xl:inline">Clear Path</span>
          </button>
        )}

        <button
          onClick={onResetPreset}
          title="Reset to official IIT Ropar Campus Layout Preset"
          className="flex items-center gap-1.5 px-3 py-1.5 rounded-xl text-xs font-medium text-slate-300 hover:text-white bg-white/5 hover:bg-white/10 border border-white/10 transition-colors cursor-pointer"
        >
          <School size={15} className="text-amber-400" />
          <span className="hidden xl:inline">IIT Ropar Preset</span>
        </button>

        <button
          onClick={onClearAll}
          title="Clear canvas"
          className="p-1.5 rounded-xl text-slate-400 hover:text-rose-400 hover:bg-rose-500/10 border border-transparent hover:border-rose-500/20 transition-colors cursor-pointer"
        >
          <RotateCcw size={16} />
        </button>
      </div>
    </header>
  );
};
