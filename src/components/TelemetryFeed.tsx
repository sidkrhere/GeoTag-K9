import React, { useState } from 'react';
import { Terminal, ChevronUp, ChevronDown, CheckCircle2, AlertTriangle, XCircle, Trash2, Moon, Sun, Compass } from 'lucide-react';
import { TrilaterationResult, GeoTag } from '../types/simulation';

interface TelemetryFeedProps {
  results: TrilaterationResult[];
  tags: GeoTag[];
  selectedTagId: string | null;
  onToggleImuResting: (tagId: string) => void;
  onDeleteSelected: () => void;
  hasSelection: boolean;
}

export const TelemetryFeed: React.FC<TelemetryFeedProps> = ({
  results,
  tags,
  selectedTagId,
  onToggleImuResting,
  onDeleteSelected,
  hasSelection
}) => {
  const [isOpen, setIsOpen] = useState(false);

  const selectedDog = tags.find(t => t.id === selectedTagId);
  const selectedResult = results.find(r => r.tagId === selectedTagId);

  return (
    <footer className="glass-panel fixed bottom-3 left-4 right-4 z-20 rounded-2xl border border-white/10 shadow-2xl transition-all duration-300">
      {/* Summary Bar */}
      <div className="flex items-center justify-between px-4 py-2.5">
        <div className="flex items-center gap-3">
          <div className="flex items-center gap-2 text-slate-300 font-mono text-xs">
            <Terminal size={14} className="text-cyan-400" />
            <span className="font-semibold text-white">GeoCentre Core Feed</span>
            <span className="text-slate-500">|</span>
            <span>Active Nodes: <b className="text-cyan-300">{tags.length} Doggos</b></span>
          </div>

          {selectedDog && (
            <div className="flex items-center gap-2 pl-3 border-l border-white/10 font-mono text-xs">
              <span className="text-slate-400">Inspecting:</span>
              <span className="text-white font-bold">{selectedDog.name}</span>
              
              {/* Dorsal e-Compass Spine Heading Widget */}
              <div className="flex items-center gap-1.5 px-2 py-0.5 rounded bg-cyan-500/10 border border-cyan-500/20 text-cyan-300 text-[11px] font-mono">
                <Compass 
                  size={12} 
                  className="text-cyan-400 transition-transform duration-200" 
                  style={{ transform: `rotate(${selectedDog.headingDeg ?? 90}deg)` }}
                />
                <span>
                  {Math.round(selectedDog.headingDeg ?? 90)}° {selectedDog.compassData?.cardinal ?? 'E'}
                </span>
                <span className="text-[9px] text-cyan-400/70 font-sans">
                  (Dorsal Spine)
                </span>
              </div>

              {selectedResult && (
                <div className="flex items-center gap-2">
                  {selectedResult.lockType === 'BLIND_ZONE' && selectedResult.borderExit ? (
                    <span className="text-[11px] font-bold text-rose-400 bg-rose-500/10 px-2 py-0.5 rounded border border-rose-500/30 font-mono">
                      📍 BLIND ZONE (LKP Exit: {selectedResult.borderExit.exitSpeedMps}m/s @ {Math.round(selectedResult.borderExit.exitHeadingDeg)}°, T-{selectedResult.borderExit.elapsedSeconds}s ago)
                    </span>
                  ) : (
                    <span className={`text-[11px] font-bold ${
                      selectedResult.lockType === 'FULL_3D_LOCK' || selectedResult.lockType === 'DUAL_PINPOINT_LOCK' || selectedResult.lockType === 'DUAL_KINEMATIC_LOCK' 
                        ? 'text-emerald-400' 
                        : 'text-amber-400'
                    }`}>
                      (Δ {selectedResult.errorMeters}m
                      {selectedResult.lockType === 'DUAL_PINPOINT_LOCK' && selectedResult.dualInfo && (
                        <span className="text-slate-300 font-normal">
                          {' '}| Match: <b className="text-cyan-300">+{selectedResult.dualInfo.candidate1VectorAlignment}</b> | Ghost: <b className="text-rose-400 font-bold">Δ {selectedResult.dualInfo.candidate2ErrorMeters}m</b>
                        </span>
                      )}
                      {selectedResult.lockType === 'DUAL_KINEMATIC_LOCK' && selectedResult.dualInfo && (
                        <span className="text-slate-400 font-normal">
                          {' '}| Mirror Ghost: <b className="text-rose-400 font-bold">Δ {selectedResult.dualInfo.candidate2ErrorMeters}m</b>
                        </span>
                      )}
                      )
                    </span>
                  )}

                  {selectedResult && (
                    <div className="flex items-center gap-1.5 px-2 py-0.5 rounded bg-blue-500/10 border border-blue-500/20 text-[11px] font-mono">
                      <span className="text-slate-400">Speed:</span>
                      <span className="text-cyan-300">Act <b>{selectedDog.speedMps ?? selectedResult.actualSpeedMps ?? 0} m/s</b></span>
                      <span className="text-slate-500">|</span>
                      <span className="text-emerald-300">Pred <b>{selectedResult.speedMps ?? 0} m/s</b></span>
                    </div>
                  )}
                </div>
              )}
              <button
                onClick={() => onToggleImuResting(selectedDog.id)}
                title="Toggle IMU Rest State (Sleep / Motion)"
                className={`flex items-center gap-1 px-2 py-0.5 rounded text-[11px] font-medium transition-colors ${selectedDog.imuResting
                    ? 'bg-purple-500/20 text-purple-300 border border-purple-500/30'
                    : 'bg-emerald-500/20 text-emerald-300 border border-emerald-500/30'
                  }`}
              >
                {selectedDog.imuResting ? <Moon size={11} /> : <Sun size={11} />}
                {selectedDog.imuResting ? 'IMU Resting (Sleeping)' : 'IMU Active (Running)'}
              </button>
            </div>
          )}
        </div>

        <div className="flex items-center gap-3">
          {hasSelection && (
            <button
              onClick={onDeleteSelected}
              className="flex items-center gap-1 px-2.5 py-1 rounded-lg text-xs font-medium text-rose-300 bg-rose-500/10 hover:bg-rose-500/20 border border-rose-500/30 transition-colors"
            >
              <Trash2 size={13} />
              <span>Delete Node</span>
            </button>
          )}

          <button
            onClick={() => setIsOpen(!isOpen)}
            className="flex items-center gap-1 text-xs text-slate-400 hover:text-white transition-colors"
          >
            <span>{isOpen ? 'Collapse Logs' : 'View Packets'}</span>
            {isOpen ? <ChevronDown size={14} /> : <ChevronUp size={14} />}
          </button>
        </div>
      </div>

      {/* Expanded Live Telemetry Table */}
      {isOpen && (
        <div className="p-4 pt-1 border-t border-white/10 max-h-48 overflow-y-auto font-mono text-[11px]">
          <table className="w-full text-left">
            <thead>
              <tr className="text-slate-500 border-b border-white/5 text-[10px] uppercase">
                <th className="pb-1.5">Tag ID / Dog</th>
                <th className="pb-1.5">Dorsal Heading</th>
                <th className="pb-1.5">Lock Status</th>
                <th className="pb-1.5">Anchors Heard</th>
                <th className="pb-1.5">Speed (Act / Pred)</th>
                <th className="pb-1.5">GDOP</th>
                <th className="pb-1.5">Est. Deviation (Error)</th>
                <th className="pb-1.5">IMU Motion State</th>
              </tr>
            </thead>
            <tbody className="divide-y divide-white/5">
              {tags.map((tag) => {
                const res = results.find(r => r.tagId === tag.id);
                return (
                  <tr key={tag.id} className="text-slate-300 hover:bg-white/5 transition-colors">
                    <td className="py-1.5 font-bold text-white flex items-center gap-1.5">
                      <span className="w-2 h-2 rounded-full" style={{ backgroundColor: tag.color }} />
                      {tag.name}
                    </td>
                    <td className="py-1.5 text-cyan-300 font-mono text-[10px]">
                      🧭 {Math.round(tag.headingDeg ?? 90)}° {tag.compassData?.cardinal ?? 'E'}
                    </td>
                    <td className="py-1.5">
                      {res?.lockType === 'FULL_3D_LOCK' ? (
                        <span className="inline-flex items-center gap-1 text-emerald-400 bg-emerald-500/10 px-2 py-0.5 rounded border border-emerald-500/20 text-[10px]">
                          <CheckCircle2 size={11} /> 3D Lock
                          {res.hypothesisFlipDetected && (
                            <span className="text-cyan-300 ml-1 font-bold">(🔄 Mirror Re-aligned)</span>
                          )}
                          {res.outlierAnchorIds && res.outlierAnchorIds.length > 0 && (
                            <span className="text-amber-400 ml-1 font-bold">({res.outlierAnchorIds.length} Outlier Filtered)</span>
                          )}
                        </span>
                      ) : res?.lockType === 'DUAL_PINPOINT_LOCK' ? (
                        <span className="inline-flex items-center gap-1 text-cyan-300 bg-cyan-500/10 px-2 py-0.5 rounded border border-cyan-500/30 text-[10px]">
                          <CheckCircle2 size={11} className="text-cyan-400" /> Fused 2-Anchor Lock
                          {res.dualInfo && (
                            <span className="text-emerald-300 ml-1 font-mono">
                              (Match: +{res.dualInfo.candidate1VectorAlignment} | Ghost: Δ{res.dualInfo.outcastLeapMeters}m)
                            </span>
                          )}
                        </span>
                      ) : res?.lockType === 'DUAL_KINEMATIC_LOCK' ? (
                        <span className="inline-flex items-center gap-1 text-emerald-400 bg-emerald-500/10 px-2 py-0.5 rounded border border-emerald-500/20 text-[10px]">
                          <CheckCircle2 size={11} /> 2-Anchor Kinematic Lock
                          {res.dualInfo && (
                            <span className="text-cyan-300 ml-1 font-mono">
                              (Mirror Outcast: Δ{res.dualInfo.outcastLeapMeters}m)
                            </span>
                          )}
                        </span>
                      ) : res?.lockType === 'DUAL_AMBIGUOUS_CHORD' ? (
                        <span className="inline-flex items-center gap-1 text-amber-400 bg-amber-500/10 px-2 py-0.5 rounded border border-amber-500/20 text-[10px]">
                          <AlertTriangle size={11} /> 2-Anchor Chord Crossing
                        </span>
                      ) : res?.lockType === 'DUAL_COLD_BOOT' ? (
                        <span className="inline-flex items-center gap-1 text-slate-300 bg-white/10 px-2 py-0.5 rounded border border-white/20 text-[10px]">
                          <AlertTriangle size={11} /> 2-Anchor Cold Boot (Syncing)
                        </span>
                      ) : res?.lockType === 'DUAL_AMBIGUITY' ? (
                        <span className="inline-flex items-center gap-1 text-amber-400 bg-amber-500/10 px-2 py-0.5 rounded border border-amber-500/20 text-[10px]">
                          <AlertTriangle size={11} /> 2-Anchor Dual Candidates
                        </span>
                      ) : res?.lockType === 'RADIAL_RING' ? (
                        <span className="inline-flex items-center gap-1 text-cyan-400 bg-cyan-500/10 px-2 py-0.5 rounded border border-cyan-500/20 text-[10px]">
                          <AlertTriangle size={11} /> 1-Anchor Radial Ring
                        </span>
                      ) : (
                        <span className="inline-flex items-center gap-1 text-rose-400 bg-rose-500/10 px-2 py-0.5 rounded border border-rose-500/20 text-[10px]">
                          <XCircle size={11} /> Blind Zone (0 Anchors)
                        </span>
                      )}
                    </td>
                    <td className="py-1.5 font-semibold text-cyan-300">
                      {res?.contributingAnchorIds.length || 0} Anchors
                    </td>
                    <td className="py-1.5 font-mono text-[10px]">
                      <span className="text-cyan-300 font-semibold">{tag.speedMps ?? res?.actualSpeedMps ?? 0}</span>
                      <span className="text-slate-500"> / </span>
                      <span className="text-emerald-400 font-semibold">{res?.speedMps ?? 0} m/s</span>
                    </td>
                    <td className="py-1.5">
                      {res ? res.gdop : '—'}
                    </td>
                    <td className="py-1.5 font-bold">
                      {res?.lockType === 'FULL_3D_LOCK' ? (
                        <span className={res.errorMeters < 3 ? 'text-emerald-400' : 'text-amber-400'}>
                          ±{res.errorMeters} m
                        </span>
                      ) : res?.lockType === 'DUAL_PINPOINT_LOCK' ? (
                        <div className="flex flex-col">
                          <span className={res.errorMeters < 3 ? 'text-emerald-400' : 'text-cyan-300'}>
                            Pinpoint: ±{res.errorMeters} m
                          </span>
                          {res.dualInfo && (
                            <span className="text-rose-400 text-[10px] font-normal">
                              Ghost: ±{res.dualInfo.candidate2ErrorMeters} m ({res.dualInfo.candidate2VectorAlignment})
                            </span>
                          )}
                        </div>
                      ) : res?.lockType === 'DUAL_KINEMATIC_LOCK' ? (
                        <div className="flex flex-col">
                          <span className={res.errorMeters < 3 ? 'text-emerald-400' : 'text-amber-400'}>
                            Opt 1 (Lock): ±{res.errorMeters} m
                          </span>
                          {res.dualInfo && (
                            <span className="text-rose-400 text-[10px] font-normal">
                              Opt 2 (Ghost): ±{res.dualInfo.candidate2ErrorMeters} m
                            </span>
                          )}
                        </div>
                      ) : res?.lockType === 'DUAL_AMBIGUOUS_CHORD' || res?.lockType === 'DUAL_AMBIGUITY' ? (
                        <div className="flex flex-col">
                          <span className="text-amber-300">Opt 1: ±{res.dualInfo?.candidate1ErrorMeters ?? res.errorMeters} m</span>
                          {res.dualInfo && (
                            <span className="text-amber-400/80 text-[10px] font-normal">
                              Opt 2: ±{res.dualInfo.candidate2ErrorMeters} m
                            </span>
                          )}
                        </div>
                      ) : res?.lockType === 'RADIAL_RING' ? (
                        <span className="text-cyan-300">Ring r ≈ {res.radialRing?.radiusMeters} m</span>
                      ) : (
                        <span className="text-slate-600">—</span>
                      )}
                    </td>
                    <td className="py-1.5">
                      {tag.imuResting ? (
                        <span className="text-purple-400">Lying Down (Sleeping)</span>
                      ) : (
                        <span className="text-emerald-400">Active / Moving</span>
                      )}
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      )}
    </footer>
  );
};
