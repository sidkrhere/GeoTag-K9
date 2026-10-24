import React, { useState } from 'react';
import { TrilaterationResult, LapMetrics } from '../types/simulation';

interface TelemetryHUDProps {
  trilaterationResults: TrilaterationResult[];
  anchorCount?: number;
  tier1Count?: number;
  tier2Count?: number;
  dogCount: number;
  lapMetrics?: LapMetrics;
}

export const TelemetryHUD: React.FC<TelemetryHUDProps> = ({
  trilaterationResults,
  anchorCount,
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
      aria-label="Live Telemetry Console"
      className="radar-panel fixed top-16 left-4 z-20 w-[290px] min-w-[290px] max-w-[290px] box-border font-mono text-[#282828] select-none"
    >
      {/* Header */}
      <div
        onClick={() => setIsExpanded(!isExpanded)}
        className="panel-stamped-header cursor-pointer select-none h-[34px] min-h-[34px] max-h-[34px] px-2.5 flex items-center justify-between box-border"
      >
        <div className="flex items-center gap-2">
          <span className="text-xs font-bold tracking-wider text-[#282828]">
            TELEMETRY
          </span>
        </div>
        <div className="flex items-center gap-2">
          <span className={`w-[58px] min-w-[58px] max-w-[58px] text-center inline-block text-[8.5px] px-1 py-0.5 border font-bold rounded truncate tabular-nums ${
            lockedResults.length > 0
              ? (is2AnchorPinpoint
                  ? 'border-[#83a598] bg-[#ebf2f2] text-[#076678]'
                  : is2AnchorKinematic
                    ? 'border-[#fe8019] bg-[#fdeddc] text-[#d65d0e]'
                    : 'border-[#b8bb26] bg-[#f4f6e4] text-[#79740e]')
              : 'border-[#d5c4a1] bg-[#fbf1c7] text-[#7c6f64]'
          }`}>
            {lockedResults.length > 0 ? (is2AnchorPinpoint || is2AnchorKinematic ? '2-ANCHOR' : '3D MULTI') : 'SEARCH'}
          </span>
          <button className="text-[#7c6f64] hover:text-[#282828] text-xs font-bold px-1" aria-label="Toggle Panel">
            {isExpanded ? '[-]' : '[+]'}
          </button>
        </div>
      </div>

      {isExpanded && (
        <div className="p-2 space-y-1.5 text-[10px] bg-[#f2e5bc] w-full box-border">
          {/* SECTION 1: ERROR & ACCURACY */}
          <div className="space-y-1">
            <div className="flex items-center justify-between text-[9px] font-bold uppercase tracking-wider text-[#7c6f64] px-0.5 h-[16px] min-h-[16px] max-h-[16px] leading-none">
              <span className="flex items-center gap-1.5">
                <span className="w-1.5 h-1.5 bg-[#af3a03] rounded-sm" />
                ERROR & ACCURACY
              </span>
              <span className={`w-[44px] min-w-[44px] text-center inline-block text-[8.5px] px-1 py-0.2 border rounded font-semibold truncate ${
                avgError < 3
                  ? 'border-[#b8bb26] bg-[#f4f6e4] text-[#79740e]'
                  : avgError < 7
                    ? 'border-[#fe8019] bg-[#fdeddc] text-[#d65d0e]'
                    : 'border-[#fb4934] bg-[#fae6e4] text-[#cc241d]'
              }`}>
                {lockedResults.length > 0 ? (avgError < 3 ? 'LOCK' : 'FAIR') : 'SEARCH'}
              </span>
            </div>

            <div className="grid grid-cols-2 gap-1.5">
              {/* Est Error Card */}
              <div className={`p-1.5 border rounded bg-[#fbf1c7] flex flex-col justify-between h-[50px] min-h-[50px] max-h-[50px] box-border overflow-hidden ${
                avgError < 3 ? 'border-[#b8bb26]' : avgError < 7 ? 'border-[#d5c4a1]' : 'border-[#fb4934]'
              }`}>
                <span className="text-[9px] font-bold text-[#7c6f64] uppercase truncate">EST ERR</span>
                <div className="flex items-baseline justify-between overflow-hidden whitespace-nowrap">
                  <span className={`text-base font-bold font-mono tabular-nums leading-none ${
                    avgError < 3 ? 'text-[#79740e]' : avgError < 7 ? 'text-[#d65d0e]' : 'text-[#cc241d]'
                  }`}>
                    {lockedResults.length > 0 ? avgError.toFixed(2) : '---'}
                  </span>
                  <span className="text-[9.5px] text-[#7c6f64] font-medium ml-1">m</span>
                </div>
                <span className="text-[8.5px] text-[#928374] tabular-nums leading-none truncate whitespace-nowrap">
                  DEV ±{(avgError * 0.4).toFixed(1)}m
                </span>
              </div>

              {/* GDOP Card */}
              <div className="p-1.5 border border-[#d5c4a1] rounded bg-[#fbf1c7] flex flex-col justify-between h-[50px] min-h-[50px] max-h-[50px] box-border overflow-hidden">
                <span className="text-[9px] font-bold text-[#7c6f64] uppercase truncate">GDOP</span>
                <div className="flex items-baseline justify-between overflow-hidden whitespace-nowrap">
                  <span className={`text-base font-bold font-mono tabular-nums leading-none ${
                    avgGdop < 2.5 ? 'text-[#79740e]' : avgGdop < 5 ? 'text-[#076678]' : 'text-[#d65d0e]'
                  }`}>
                    {lockedResults.length > 0 ? avgGdop.toFixed(2) : '---'}
                  </span>
                  <span className="text-[9.5px] text-[#7c6f64] font-medium ml-1">DIL</span>
                </div>
                <span className="text-[8.5px] text-[#928374] leading-none truncate whitespace-nowrap">
                  {lockedResults.length > 0 ? (avgGdop < 2.5 ? 'OPTIMAL' : avgGdop < 5 ? 'GOOD' : 'FAIR') : 'SEARCH'}
                </span>
              </div>
            </div>
          </div>

          {/* SECTION 2: PATROL BENCHMARK */}
          <div className="space-y-1">
            <div className="flex items-center gap-1.5 text-[9px] font-bold uppercase tracking-wider text-[#7c6f64] px-0.5 h-[16px] min-h-[16px] max-h-[16px] leading-none">
              <span className="w-1.5 h-1.5 bg-[#076678] rounded-sm" />
              PATROL BENCHMARK
            </div>

            <div className="p-1.5 border border-[#d5c4a1] rounded bg-[#fbf1c7] grid grid-cols-3 gap-1 h-[42px] min-h-[42px] max-h-[42px] box-border items-center overflow-hidden">
              <div className="overflow-hidden whitespace-nowrap">
                <span className="text-[#7c6f64] block font-semibold truncate text-[8px] leading-tight">
                  CURR (L{lapMetrics?.currentLapNumber ?? 1})
                </span>
                <span className="text-[11px] font-bold text-[#076678] block font-mono tabular-nums truncate leading-none mt-0.5">
                  {lapMetrics && lapMetrics.currentLapSamples > 0
                    ? `${lapMetrics.currentLapAvgError.toFixed(2)}m`
                    : '---'}
                </span>
              </div>

              <div className="border-l border-[#ebdbb2] pl-1.5 overflow-hidden whitespace-nowrap">
                <span className="text-[#7c6f64] block font-semibold truncate text-[8px] leading-tight">
                  MEAN ERR
                </span>
                <span className="text-[11px] font-bold text-[#79740e] block font-mono tabular-nums truncate leading-none mt-0.5">
                  {lapMetrics && lapMetrics.totalSamples > 0
                    ? `${lapMetrics.lifetimeAvgError.toFixed(2)}m`
                    : '---'}
                </span>
              </div>

              <div className="border-l border-[#ebdbb2] pl-1.5 overflow-hidden whitespace-nowrap">
                <span className="text-[#7c6f64] block font-semibold truncate text-[8px] leading-tight">
                  PREV LAP
                </span>
                <span className="text-[11px] font-bold text-[#d65d0e] block font-mono tabular-nums truncate leading-none mt-0.5">
                  {lapMetrics?.lastCompletedLapAvgError != null
                    ? `${lapMetrics.lastCompletedLapAvgError.toFixed(2)}m`
                    : '---'}
                </span>
              </div>
            </div>
          </div>

          {/* SECTION 3: TARGET KINEMATICS */}
          <div className="space-y-1">
            <div className="flex items-center gap-1.5 text-[9px] font-bold uppercase tracking-wider text-[#7c6f64] px-0.5 h-[16px] min-h-[16px] max-h-[16px] leading-none">
              <span className="w-1.5 h-1.5 bg-[#79740e] rounded-sm" />
              TARGET KINEMATICS
            </div>

            <div className="grid grid-cols-2 gap-1.5">
              {/* Actual Speed Card */}
              <div className="p-1.5 border border-[#83a598] rounded bg-[#ebf2f2] flex flex-col justify-between h-[50px] min-h-[50px] max-h-[50px] box-border overflow-hidden">
                <span className="text-[9px] text-[#076678] font-bold uppercase truncate">
                  ACTUAL VEL
                </span>
                <div className="flex items-baseline justify-between overflow-hidden whitespace-nowrap">
                  <span className="text-base font-bold font-mono tabular-nums text-[#076678] leading-none">
                    {actualSpeedVal.toFixed(1)}
                  </span>
                  <span className="text-[9.5px] text-[#458588] font-medium ml-1">m/s</span>
                </div>
              </div>

              {/* Estimated Speed Card */}
              <div className="p-1.5 border border-[#b8bb26] rounded bg-[#f4f6e4] flex flex-col justify-between h-[50px] min-h-[50px] max-h-[50px] box-border overflow-hidden">
                <span className="text-[9px] text-[#79740e] font-bold uppercase truncate">
                  EST VEL
                </span>
                <div className="flex items-baseline justify-between overflow-hidden whitespace-nowrap">
                  <span className="text-base font-bold font-mono tabular-nums text-[#79740e] leading-none">
                    {predSpeedVal.toFixed(1)}
                  </span>
                  <span className="text-[9.5px] text-[#98971a] font-medium ml-1">m/s</span>
                </div>
              </div>
            </div>
          </div>

          {/* SECTION 4: NETWORK NODES */}
          <div className="pt-1.5 border-t border-[#d5c4a1] flex items-center justify-between text-[9px] text-[#7c6f64] uppercase font-semibold h-[24px] min-h-[24px] max-h-[24px] leading-none box-border whitespace-nowrap overflow-hidden">
            <span>ANCHORS: <b className="text-[#076678] font-mono tabular-nums">{anchorCount ?? (tier1Count ?? 0) + (tier2Count ?? 0)}</b></span>
            <span>TAGS: <b className="text-[#282828] font-mono tabular-nums">{dogCount}</b></span>
          </div>
        </div>
      )}
    </aside>
  );
};
