import React, { useState } from 'react';
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

  // Compute live average error across locked targets
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
      aria-label="Tactical Display Unit TDU-5"
      className="radar-panel fixed top-16 left-4 z-20 w-[420px] font-mono text-[#00ff41]"
    >
      {/* 1950s Metal Instrument Faceplate Header */}
      <div
        onClick={() => setIsExpanded(!isExpanded)}
        className="panel-stamped-header cursor-pointer select-none"
      >
        <div className="flex items-center gap-2">
          <span className="text-xs font-bold tracking-widest text-[#00f2fe] glow-cyan">
            TACTICAL DISPLAY UNIT // TDU-5
          </span>
        </div>
        <div className="flex items-center gap-2">
          {lockedResults.length > 0 && (
            <span className={`text-[10px] px-2 py-0.5 border font-bold tracking-wider ${
              is2AnchorPinpoint 
                ? 'border-[#00f2fe] bg-[#001f22] text-[#00f2fe]'
                : is2AnchorKinematic
                  ? 'border-[#ff9900] bg-[#1a0f00] text-[#ff9900]'
                  : 'border-[#00ff41] bg-[#002b0c] text-[#00ff41]'
            }`}>
              {is2AnchorPinpoint ? '2-BCN PINPOINT' : is2AnchorKinematic ? 'KINEMATIC' : '3D LOCK'}
            </span>
          )}
          <button className="text-[#00aa33] hover:text-[#00ff41] text-xs font-bold" aria-label="Toggle Panel">
            {isExpanded ? '[-]' : '[+]'}
          </button>
        </div>
      </div>

      {isExpanded && (
        <div className="p-3.5 space-y-3 text-[11px] bg-[#020b04]">
          {/* Row 1: Primary Localization Error & GDOP Gauges */}
          <div className="grid grid-cols-2 gap-2.5">
            {/* Live Error Gauge - Color-coded by error severity */}
            <div className={`p-2.5 border bg-[#010903] flex flex-col justify-between h-[84px] ${
              avgError < 3 ? 'border-[#00551a]' : avgError < 7 ? 'border-[#804c00]' : 'border-[#801d18]'
            }`}>
              <div className="flex items-center justify-between text-[10px] font-bold uppercase tracking-wider">
                <span className="text-[#00aa33]">EST SPATIAL ERROR</span>
                <span className={`text-[9px] ${avgError < 3 ? 'text-[#00ff41]' : avgError < 7 ? 'text-[#ff9900]' : 'text-[#ff3b30]'}`}>
                  {lockedResults.length > 0 ? '[LOCKED]' : '[UNSOLVED]'}
                </span>
              </div>
              <div className="flex items-baseline gap-1.5 my-auto">
                <span className={`text-2xl font-bold tracking-tight ${
                  avgError < 3 ? 'text-[#00ff41] glow-green' : avgError < 7 ? 'text-[#ff9900] glow-orange' : 'text-[#ff3b30] glow-red'
                }`}>
                  {lockedResults.length > 0 ? avgError.toFixed(1) : '—'}
                </span>
                {lockedResults.length > 0 && <span className="text-xs text-[#008826]">METERS</span>}
              </div>
              <div className="text-[9px] text-[#00551a]">
                RADIAL CONFIDENCE: ±{(avgError * 0.4).toFixed(1)}M
              </div>
            </div>

            {/* Constellation GDOP Gauge */}
            <div className="p-2.5 border border-[#00551a] bg-[#010903] flex flex-col justify-between h-[84px]">
              <div className="flex items-center justify-between text-[10px] text-[#00aa33] font-bold uppercase tracking-wider">
                <span>GEOM DILUTION (GDOP)</span>
                <span className="text-[9px] text-[#00f2fe]">TIE-LINE</span>
              </div>
              <div className="flex items-baseline gap-1.5 my-auto">
                <span className={`text-2xl font-bold tracking-tight ${
                  avgGdop < 2.5 ? 'text-[#00ff41] glow-green' : avgGdop < 5 ? 'text-[#00f2fe] glow-cyan' : 'text-[#ff9900] glow-orange'
                }`}>
                  {lockedResults.length > 0 ? avgGdop.toFixed(2) : '—'}
                </span>
                {lockedResults.length > 0 && (
                  <span className={`text-[10px] ${avgGdop < 2.5 ? 'text-[#00ff41]' : avgGdop < 5 ? 'text-[#00f2fe]' : 'text-[#ff9900]'}`}>
                    {avgGdop < 2.5 ? '[OPTIMAL]' : avgGdop < 5 ? '[GOOD]' : '[MODERATE]'}
                  </span>
                )}
              </div>
              <div className="text-[9px] text-[#00551a]">
                CONSTELLATION BASELINE RATIO
              </div>
            </div>
          </div>

          {/* Row 1.5: Patrol Sector Rounds (Laps & Lifetime Benchmark) */}
          <div className="p-2 border border-[#00551a] bg-[#010903] flex items-center justify-between text-[10px]">
            <div>
              <span className="text-[#00aa33] block">PATROL ROUND #{lapMetrics?.currentLapNumber ?? 1}</span>
              <span className="text-xs font-bold text-[#00f2fe]">
                {lapMetrics && lapMetrics.currentLapSamples > 0
                  ? `${lapMetrics.currentLapAvgError.toFixed(2)} M`
                  : 'SYNCHRONIZING'}
              </span>
            </div>

            <div className="h-6 w-[1px] bg-[#00551a]" />

            <div>
              <span className="text-[#00aa33] block">CUMULATIVE ERROR</span>
              <span className="text-xs font-bold text-[#00ff41]">
                {lapMetrics && lapMetrics.totalSamples > 0
                  ? `${lapMetrics.lifetimeAvgError.toFixed(2)} M`
                  : '—'}
              </span>
            </div>

            {lapMetrics?.lastCompletedLapAvgError != null && (
              <>
                <div className="h-6 w-[1px] bg-[#00551a]" />
                <div className="text-right">
                  <span className="text-[#008826] block">PREV ROUND</span>
                  <span className="text-xs font-bold text-[#ff9900]">
                    {lapMetrics.lastCompletedLapAvgError.toFixed(2)} M
                  </span>
                </div>
              </>
            )}
          </div>

          {/* Row 2: Target Velocity & Kalman Filter Estimate (Cyan for Dog, Green for KF) */}
          <div className="grid grid-cols-2 gap-2.5">
            <div className="p-2 border border-[#007780] bg-[#001014]">
              <div className="flex justify-between text-[10px] text-[#00f2fe] font-bold">
                <span>TARGET SPEED (LIVE)</span>
                <span>CH-A</span>
              </div>
              <div className="flex items-baseline gap-1 my-1">
                <span className="text-xl font-bold text-[#00f2fe] glow-cyan">
                  {actualSpeedVal.toFixed(1)}
                </span>
                <span className="text-[10px] text-[#007780]">M/S</span>
                <span className="text-[9px] text-[#007780] ml-auto">
                  ({(actualSpeedVal * 3.6).toFixed(0)} KM/H)
                </span>
              </div>
            </div>

            <div className="p-2 border border-[#00551a] bg-[#010903]">
              <div className="flex justify-between text-[10px] text-[#00aa33] font-bold">
                <span>PRED VELOCITY (KF)</span>
                <span>DVB-KF</span>
              </div>
              <div className="flex items-baseline gap-1 my-1">
                <span className="text-xl font-bold text-[#00ff41] glow-green">
                  {predSpeedVal.toFixed(1)}
                </span>
                <span className="text-[10px] text-[#008826]">M/S</span>
                <span className="text-[9px] text-[#00551a] ml-auto">
                  ({(predSpeedVal * 3.6).toFixed(0)} KM/H)
                </span>
              </div>
            </div>
          </div>

          {/* Row 3: Active Station Counts with Distinct Colors */}
          <div className="pt-2 border-t border-[#003310] flex items-center justify-between text-[10px]">
            <span>T1 MASTS: <b className="text-[#00f2fe] font-bold">{tier1Count}</b></span>
            <span>T2 RELAYS: <b className="text-[#ff9900] font-bold">{tier2Count}</b></span>
            <span>TARGETS: <b className="text-[#00f2fe] font-bold">{dogCount}</b></span>
          </div>
        </div>
      )}
    </aside>
  );
};
