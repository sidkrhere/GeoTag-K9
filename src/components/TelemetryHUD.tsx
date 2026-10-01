import React, { useState } from 'react';
import { Activity, ChevronDown, ChevronUp, RotateCw, Sigma } from 'lucide-react';
import { TrilaterationResult, LapMetrics } from '../types/simulation';

interface TelemetryHUDProps {
  trilaterationResults: TrilaterationResult[];
  tier1Count: number;
  tier2Count: number;
  dogCount: number;
  lapMetrics?: LapMetrics;
}

export const TelemetryHUD: React.FC<TelemetryHUDProps> = ({
  trilaterationResults,
  tier1Count,
  tier2Count,
  dogCount,
  lapMetrics
}) => {
  const [isExpanded, setIsExpanded] = useState(true);

  // Compute live average error across locked dogs (supporting 3D Full Lock, 2-Anchor Pinpoint, and Kinematic Locks)
  const lockedResults = trilaterationResults.filter(r => 
    r.lockType === 'FULL_3D_LOCK' ||
    r.lockType === 'DUAL_PINPOINT_LOCK' ||
    r.lockType === 'DUAL_KINEMATIC_LOCK'
  );

  const avgError = lockedResults.length > 0 
    ? Math.round((lockedResults.reduce((acc, r) => acc + r.errorMeters, 0) / lockedResults.length) * 10) / 10
    : 0;

  const avgGdop = lockedResults.length > 0
    ? Math.round((lockedResults.reduce((acc, r) => acc + r.gdop, 0) / lockedResults.length) * 10) / 10
    : 0;

  const is2AnchorPinpoint = lockedResults.some(r => r.lockType === 'DUAL_PINPOINT_LOCK');
  const is2AnchorKinematic = lockedResults.some(r => r.lockType === 'DUAL_KINEMATIC_LOCK');

  const actualSpeedVal = lockedResults.length > 0 ? (lockedResults[0].actualSpeedMps ?? 0) : 0;
  const predSpeedVal = lockedResults.length > 0 ? (lockedResults[0].speedMps ?? 0) : 0;

  return (
    <aside 
      aria-label="Pure Real-Time Telemetry HUD" 
      className="glass-panel fixed top-20 left-4 z-20 w-[420px] rounded-2xl border border-white/10 shadow-2xl transition-all duration-300 overflow-hidden"
    >
      {/* Header Bar */}
      <div 
        onClick={() => setIsExpanded(!isExpanded)}
        className="flex items-center justify-between px-4 py-3 bg-white/5 border-b border-white/10 cursor-pointer hover:bg-white/10 transition-colors"
      >
        <div className="flex items-center gap-2 text-white">
          <Activity size={17} className="text-cyan-400" />
          <h2 className="font-display font-semibold text-xs tracking-wider uppercase">
            Live Telemetry HUD
          </h2>
        </div>
        <div className="flex items-center gap-2">
          {lockedResults.length > 0 && (
            <span className={`flex items-center gap-1.5 text-[11px] font-mono px-2.5 py-0.5 rounded-full border ${
              is2AnchorPinpoint 
                ? 'text-cyan-300 bg-cyan-500/10 border-cyan-500/30' 
                : is2AnchorKinematic
                ? 'text-emerald-300 bg-emerald-500/10 border-emerald-500/25'
                : 'text-emerald-400 bg-emerald-500/10 border-emerald-500/20'
            }`}>
              <span className={`w-1.5 h-1.5 rounded-full animate-ping ${is2AnchorPinpoint ? 'bg-cyan-400' : 'bg-emerald-400'}`} />
              {lockedResults.length} {is2AnchorPinpoint ? 'Fused 2-Anchor Lock' : is2AnchorKinematic ? 'Kinematic Lock' : '3D Lock'}
            </span>
          )}
          <button className="text-slate-400 hover:text-white" aria-label="Toggle Telemetry Panel">
            {isExpanded ? <ChevronUp size={16} /> : <ChevronDown size={16} />}
          </button>
        </div>
      </div>

      {isExpanded && (
        <div className="p-4 space-y-3.5 font-mono text-[11px]">
          {/* Row 1: Avg Error & Constellation GDOP */}
          <div className="grid grid-cols-2 gap-3">
            {/* Live Error Tile */}
            <div className="p-3 rounded-xl bg-white/5 border border-white/10 flex flex-col justify-between h-[86px]">
              <div className="flex items-center justify-between gap-1">
                <span className="text-slate-400 text-[10px] font-bold uppercase tracking-wider whitespace-nowrap">
                  Live Error
                </span>
                {is2AnchorPinpoint ? (
                  <span className="text-[10px] px-2 py-0.5 rounded-full bg-cyan-500/20 text-cyan-300 border border-cyan-500/30 font-sans font-bold flex items-center gap-1 whitespace-nowrap">
                    <span className="w-1.5 h-1.5 rounded-full bg-cyan-400 animate-pulse" />
                    USED COS + KIN
                  </span>
                ) : is2AnchorKinematic ? (
                  <span className="text-[10px] px-2 py-0.5 rounded-full bg-emerald-500/20 text-emerald-300 border border-emerald-500/30 font-sans font-bold whitespace-nowrap">
                    Kinematic
                  </span>
                ) : lockedResults.length > 0 ? (
                  <span className="text-[10px] px-2 py-0.5 rounded-full bg-emerald-500/20 text-emerald-400 border border-emerald-500/30 font-sans font-bold whitespace-nowrap">
                    3D Lock
                  </span>
                ) : (
                  <span className="text-[10px] px-2 py-0.5 rounded-full bg-slate-500/20 text-slate-400 font-sans whitespace-nowrap">
                    No Lock
                  </span>
                )}
              </div>
              <div className="flex items-baseline gap-1.5">
                <span className={`text-2xl font-black tabular-nums tracking-tight ${avgError < 3 ? 'text-emerald-400' : avgError < 7 ? 'text-amber-400' : 'text-rose-400'}`}>
                  {lockedResults.length > 0 ? avgError.toFixed(1) : '—'}
                </span>
                {lockedResults.length > 0 && <span className="text-xs text-slate-400 font-sans font-medium">m</span>}
              </div>
            </div>

            {/* Constellation GDOP Tile */}
            <div className="p-3 rounded-xl bg-white/5 border border-white/10 flex flex-col justify-between h-[86px]">
              <div className="flex items-center justify-between gap-1">
                <span className="text-slate-400 text-[10px] font-bold uppercase tracking-wider whitespace-nowrap">
                  Constellation
                </span>
                <span className="text-[10px] px-2 py-0.5 rounded-full bg-cyan-500/10 text-cyan-300 border border-cyan-500/20 font-sans font-medium whitespace-nowrap">
                  GDOP
                </span>
              </div>
              <div className="flex items-baseline gap-1.5">
                <span className={`text-2xl font-black tabular-nums tracking-tight ${avgGdop < 2.5 ? 'text-emerald-400' : 'text-cyan-400'}`}>
                  {lockedResults.length > 0 ? avgGdop.toFixed(2) : '—'}
                </span>
                {lockedResults.length > 0 && (
                  <span className="text-[11px] text-slate-400 font-sans">
                    {avgGdop < 2.5 ? '(Optimal)' : avgGdop < 5 ? '(Good)' : '(Moderate)'}
                  </span>
                )}
              </div>
            </div>
          </div>

          {/* Row 1.5: Lap & Lifetime Accuracy Bar (Real-Time Lap Error & Error Till Now) */}
          <div className="px-3 py-2 rounded-xl bg-gradient-to-r from-cyan-950/40 via-slate-900/50 to-emerald-950/40 border border-white/10 flex items-center justify-between text-[11px] font-mono shadow-inner shadow-cyan-500/5">
            {/* Lap Error */}
            <div className="flex items-center gap-2">
              <div className="w-5 h-5 rounded-md bg-cyan-500/10 border border-cyan-500/25 flex items-center justify-center shrink-0">
                <RotateCw size={11} className="text-cyan-400" />
              </div>
              <div className="flex flex-col">
                <span className="text-[9px] uppercase tracking-wider text-slate-400 font-bold">
                  Lap #{lapMetrics?.currentLapNumber ?? 1} Avg
                </span>
                <span className="text-xs font-black text-cyan-300 tabular-nums">
                  {lapMetrics && lapMetrics.currentLapSamples > 0 
                    ? `${lapMetrics.currentLapAvgError.toFixed(2)} m` 
                    : '—'}
                </span>
              </div>
            </div>

            <div className="h-6 w-[1px] bg-white/10" />

            {/* Lifetime Error Till Now */}
            <div className="flex items-center gap-2">
              <div className="w-5 h-5 rounded-md bg-emerald-500/10 border border-emerald-500/25 flex items-center justify-center shrink-0">
                <Sigma size={11} className="text-emerald-400" />
              </div>
              <div className="flex flex-col">
                <span className="text-[9px] uppercase tracking-wider text-slate-400 font-bold">
                  Avg Till Now
                </span>
                <span className="text-xs font-black text-emerald-300 tabular-nums">
                  {lapMetrics && lapMetrics.totalSamples > 0 
                    ? `${lapMetrics.lifetimeAvgError.toFixed(2)} m` 
                    : '—'}
                </span>
              </div>
            </div>

            {/* Last Completed Lap Benchmark */}
            {lapMetrics?.lastCompletedLapAvgError != null && (
              <>
                <div className="h-6 w-[1px] bg-white/10" />
                <div className="flex flex-col items-end">
                  <span className="text-[9px] uppercase tracking-wider text-slate-400">Prev Lap</span>
                  <span className="text-xs font-mono font-bold text-slate-300 tabular-nums">
                    {lapMetrics.lastCompletedLapAvgError.toFixed(2)} m
                  </span>
                </div>
              </>
            )}
          </div>

          {/* Row 2: Dedicated Spacious Speed Readouts (No wrapping, rock-solid boxes) */}
          <div className="grid grid-cols-2 gap-3">
            {/* Actual Dog Speed */}
            <div className="p-3 rounded-xl bg-cyan-500/[0.08] border border-cyan-500/25 flex flex-col justify-between h-[92px] overflow-hidden">
              <div className="flex items-center justify-between">
                <span className="text-cyan-300 text-[10px] font-bold uppercase tracking-wider whitespace-nowrap">
                  Actual Speed
                </span>
                <span className="flex items-center gap-1 text-[9px] text-cyan-400/90 font-mono">
                  <span className="w-1.5 h-1.5 rounded-full bg-cyan-400 animate-pulse" />
                  Live Dog
                </span>
              </div>
              <div className="flex items-baseline gap-1.5">
                <span className="text-3xl font-black text-cyan-300 tabular-nums tracking-tight">
                  {actualSpeedVal.toFixed(1)}
                </span>
                <span className="text-xs font-bold text-cyan-400/80 uppercase font-sans">m/s</span>
              </div>
            </div>

            {/* Predicted Speed - BIGGER, SPACIOUS ROCK-SOLID BOX */}
            <div className="p-3 rounded-xl bg-emerald-500/[0.08] border border-emerald-500/35 flex flex-col justify-between h-[92px] overflow-hidden shadow-inner shadow-emerald-500/10">
              <div className="flex items-center justify-between">
                <span className="text-emerald-300 text-[10px] font-bold uppercase tracking-wider whitespace-nowrap">
                  Pred Speed
                </span>
                <span className="text-[10px] px-2 py-0.5 rounded-md bg-emerald-500/25 border border-emerald-500/40 text-emerald-300 font-mono font-bold tracking-wide whitespace-nowrap">
                  DVB-KF
                </span>
              </div>
              <div className="flex items-baseline gap-1.5">
                <span className="text-3xl font-black text-emerald-400 tabular-nums tracking-tight">
                  {predSpeedVal.toFixed(1)}
                </span>
                <span className="text-xs font-bold text-emerald-400/80 uppercase font-sans">m/s</span>
              </div>
            </div>
          </div>

          {/* Row 3: Active Node Counts Footer (Spacious, full width, no cut-offs) */}
          <div className="pt-2.5 border-t border-white/10 flex items-center justify-between px-1 text-[11px] text-slate-400">
            <span>T1 Poles: <b className="text-cyan-300 font-bold ml-1">{tier1Count}</b></span>
            <span>T2 Eateries: <b className="text-amber-300 font-bold ml-1">{tier2Count}</b></span>
            <span>Doggos: <b className="text-emerald-300 font-bold ml-1">{dogCount}</b></span>
          </div>
        </div>
      )}
    </aside>
  );
};
