import React, { useState } from 'react';
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
    <footer className="radar-panel fixed bottom-3 left-4 right-4 z-20 font-mono text-[#282828]">
      {/* Telemetry Summary Bar */}
      <div className="flex items-center justify-between px-3 py-1.5 text-xs bg-[#f2e5bc]">
        <div className="flex items-center gap-2.5">
          <div className="flex items-center gap-1.5">
            <span className="text-[10px] px-1.5 py-0.5 border border-[#458588] bg-[#ebf2f2] text-[#076678] font-bold rounded">
              GATEWAY LOG
            </span>
            <span className="font-bold text-[#282828]">NETWORK ONLINE</span>
            <span className="text-[#bdae93]">|</span>
            <span className="text-[#7c6f64]">ACTIVE TAGS: <b className="text-[#076678] font-mono tabular-nums">{tags.length}</b></span>
          </div>

          {selectedDog && (
            <div className="flex items-center gap-2 pl-2.5 border-l border-[#d5c4a1] text-[11px]">
              <span className="text-[#7c6f64] font-semibold">TARGET:</span>
              <span className="text-[#076678] font-bold">{selectedDog.name.toUpperCase()}</span>

              {/* Dorsal Heading */}
              <div className="px-1.5 py-0.5 border border-[#d5c4a1] bg-[#fbf1c7] text-[#282828] text-[10px] rounded font-mono tabular-nums min-w-[70px] text-center">
                {Math.round(selectedDog.headingDeg ?? 90)}° {selectedDog.compassData?.cardinal ?? 'E'}
              </div>

              {selectedResult && (
                <div className="flex items-center gap-1.5">
                  {selectedResult.lockType === 'BLIND_ZONE' && selectedResult.borderExit ? (
                    <span className="text-[10px] border border-[#fb4934] bg-[#fae6e4] text-[#cc241d] px-1.5 py-0.5 font-bold rounded">
                      SIGNAL LOST (T+{selectedResult.borderExit.elapsedSeconds}s)
                    </span>
                  ) : (
                    <span className="text-[10px] border border-[#d5c4a1] bg-[#fbf1c7] px-1.5 py-0.5 rounded font-mono tabular-nums min-w-[76px] text-center inline-block">
                      <span className={`font-bold ${selectedResult.errorMeters < 3 ? 'text-[#79740e]' : selectedResult.errorMeters < 7 ? 'text-[#d65d0e]' : 'text-[#cc241d]'}`}>
                        Δ {selectedResult.errorMeters.toFixed(1)} m
                      </span>
                    </span>
                  )}

                  <div className="px-1.5 py-0.5 border border-[#d5c4a1] bg-[#fbf1c7] text-[10px] rounded font-mono tabular-nums min-w-[130px] text-center">
                    <span className="text-[#7c6f64]">VEL: </span>
                    <span className="text-[#076678] font-bold">{(selectedDog.speedMps ?? selectedResult.actualSpeedMps ?? 0).toFixed(1)}</span>
                    <span className="text-[#bdae93]"> / </span>
                    <span className="text-[#79740e] font-bold">{(selectedResult.speedMps ?? 0).toFixed(1)} m/s</span>
                  </div>
                </div>
              )}

              <button
                onClick={() => onToggleImuResting(selectedDog.id)}
                title="Toggle IMU Target Harness State (Standby / Active)"
                className={`radar-key ${selectedDog.imuResting ? 'radar-key-orange radar-key-active' : 'radar-key-cyan'} text-[10px] py-0.5 px-2`}
              >
                <span>{selectedDog.imuResting ? 'COLLAR: STANDBY' : 'COLLAR: ACTIVE'}</span>
              </button>
            </div>
          )}
        </div>

        <div className="flex items-center gap-1.5">
          {hasSelection && (
            <button
              onClick={onDeleteSelected}
              className="radar-key radar-key-red text-[10px] py-0.5 px-2"
            >
              <span>DELETE TAG</span>
            </button>
          )}

          <button
            onClick={() => setIsOpen(!isOpen)}
            className="radar-key text-[10px] py-0.5 px-2"
          >
            <span>{isOpen ? 'COLLAPSE LOG' : 'EXPAND LOG'}</span>
          </button>
        </div>
      </div>

      {/* Expanded Table */}
      {isOpen && (
        <div className="p-3 border-t border-[#d5c4a1] max-h-48 overflow-y-auto text-[10px] bg-[#fbf1c7]">
          <table className="w-full text-left">
            <thead>
              <tr className="text-[#7c6f64] border-b border-[#d5c4a1] uppercase font-bold">
                <th className="pb-1">TARGET TAG</th>
                <th className="pb-1">HEADING</th>
                <th className="pb-1">LOCK MODE</th>
                <th className="pb-1">ANCHORS</th>
                <th className="pb-1">SPEED (LIVE / EST)</th>
                <th className="pb-1">GDOP</th>
                <th className="pb-1">ERROR</th>
                <th className="pb-1">COLLAR</th>
              </tr>
            </thead>
            <tbody className="divide-y divide-[#ebdbb2]">
              {tags.map((tag) => {
                const res = results.find(r => r.tagId === tag.id);
                return (
                  <tr key={tag.id} className="text-[#282828] hover:bg-[#f2e5bc]">
                    <td className="py-1 font-bold text-[#076678]">
                      {tag.name.toUpperCase()}
                    </td>
                    <td className="py-1 font-mono tabular-nums text-[#504945]">
                      {Math.round(tag.headingDeg ?? 90)}° {tag.compassData?.cardinal ?? 'E'}
                    </td>
                    <td className="py-1">
                      {res?.lockType === 'FULL_3D_LOCK' ? (
                        <span className="text-[#79740e] font-bold">
                          3D Multilateration
                        </span>
                      ) : res?.lockType === 'DUAL_PINPOINT_LOCK' ? (
                        <span className="text-[#076678] font-bold">
                          2-Anchor Fusion
                        </span>
                      ) : res?.lockType === 'DUAL_KINEMATIC_LOCK' ? (
                        <span className="text-[#d65d0e] font-semibold">
                          2-Anchor Lock
                        </span>
                      ) : (
                        <span className="text-[#cc241d] font-bold">
                          Signal Lost
                        </span>
                      )}
                    </td>
                    <td className="py-1 font-mono tabular-nums text-[#076678]">
                      {res?.contributingAnchorIds.length || 0} Anchors
                    </td>
                    <td className="py-1 font-mono tabular-nums">
                      <span className="text-[#076678]">{(tag.speedMps ?? res?.actualSpeedMps ?? 0).toFixed(1)}</span>
                      <span className="text-[#bdae93]"> / </span>
                      <span className="text-[#79740e]">{(res?.speedMps ?? 0).toFixed(1)} m/s</span>
                    </td>
                    <td className="py-1 font-mono tabular-nums">
                      {res ? (
                        <span className={res.gdop < 2.5 ? 'text-[#79740e]' : res.gdop < 5 ? 'text-[#076678]' : 'text-[#d65d0e]'}>
                          {res.gdop.toFixed(2)}
                        </span>
                      ) : '—'}
                    </td>
                    <td className="py-1 font-mono tabular-nums font-bold">
                      {res ? (
                        <span className={res.errorMeters < 3 ? 'text-[#79740e]' : 'text-[#d65d0e]'}>±{res.errorMeters.toFixed(1)} m</span>
                      ) : '—'}
                    </td>
                    <td className="py-1">
                      {tag.imuResting ? <span className="text-[#d65d0e]">Standby</span> : <span className="text-[#79740e]">Active</span>}
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
