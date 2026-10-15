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
    <footer className="radar-panel fixed bottom-3 left-4 right-4 z-20 font-mono text-[#00ff41]">
      {/* 1950s Teleprinter Summary Bar */}
      <div className="flex items-center justify-between px-3 py-2 text-xs bg-[#020b04]">
        <div className="flex items-center gap-3">
          <div className="flex items-center gap-2">
            <span className="text-[10px] px-1.5 py-0.5 border border-[#00f2fe] bg-[#001f22] text-[#00f2fe] font-bold">
              SIGINT-FEED
            </span>
            <span className="font-bold text-[#00f2fe] glow-cyan">GEOCENTRE MASTER TELEPRINTER</span>
            <span className="text-[#00551a]">|</span>
            <span className="text-[#00aa33]">ACTIVE TRACKS: <b className="text-[#00f2fe]">{tags.length}</b></span>
          </div>

          {selectedDog && (
            <div className="flex items-center gap-2 pl-3 border-l border-[#00551a] text-[11px]">
              <span className="text-[#00aa33]">TARGET:</span>
              <span className="text-[#00f2fe] font-bold glow-cyan">{selectedDog.name.toUpperCase()}</span>

              {/* Dorsal Azimuth Dial */}
              <div className="px-1.5 py-0.5 border border-[#007780] bg-[#001418] text-[#00f2fe] text-[10px]">
                AZ: {Math.round(selectedDog.headingDeg ?? 90)}° {selectedDog.compassData?.cardinal ?? 'E'} (DORSAL)
              </div>

              {selectedResult && (
                <div className="flex items-center gap-2">
                  {selectedResult.lockType === 'BLIND_ZONE' && selectedResult.borderExit ? (
                    <span className="text-[10px] border border-[#ff3b30] bg-[#220705] text-[#ff3b30] px-1.5 py-0.5 font-bold glow-red">
                      [BLIND ZONE // LKP BRG {Math.round(selectedResult.borderExit.exitHeadingDeg)}° // VEL {selectedResult.borderExit.exitSpeedMps}M/S]
                    </span>
                  ) : (
                    <span className="text-[10px] border border-[#00551a] px-1.5 py-0.5">
                      <span className={selectedResult.errorMeters < 3 ? 'text-[#00ff41]' : selectedResult.errorMeters < 7 ? 'text-[#ff9900]' : 'text-[#ff3b30]'}>
                        Δ {selectedResult.errorMeters}M
                      </span>
                      {selectedResult.lockType === 'DUAL_PINPOINT_LOCK' && selectedResult.dualInfo && (
                        <span className="text-[#00f2fe]">
                          {' '}| MATCH: +{selectedResult.dualInfo.candidate1VectorAlignment} | <span className="text-[#ff3b30]">GHOST: Δ{selectedResult.dualInfo.candidate2ErrorMeters}M</span>
                        </span>
                      )}
                      {selectedResult.lockType === 'DUAL_KINEMATIC_LOCK' && selectedResult.dualInfo && (
                        <span className="text-[#ff9900]">
                          {' '}| <span className="text-[#ff3b30]">GHOST: Δ{selectedResult.dualInfo.candidate2ErrorMeters}M</span>
                        </span>
                      )}
                    </span>
                  )}

                  <div className="px-1.5 py-0.5 border border-[#00551a] bg-[#010903] text-[10px]">
                    <span className="text-[#008826]">VEL: </span>
                    <span className="text-[#00f2fe] font-bold">ACT {selectedDog.speedMps ?? selectedResult.actualSpeedMps ?? 0}</span>
                    <span className="text-[#00551a]"> / </span>
                    <span className="text-[#00ff41] font-bold">KF {selectedResult.speedMps ?? 0} M/S</span>
                  </div>
                </div>
              )}

              <button
                onClick={() => onToggleImuResting(selectedDog.id)}
                title="Toggle IMU Target Harness State (Standby / Moving)"
                className={`radar-key ${selectedDog.imuResting ? 'radar-key-orange radar-key-active' : 'radar-key-cyan'} text-[10px] py-0.5 px-2`}
              >
                <span>{selectedDog.imuResting ? 'HARNESS: DORMANT' : 'HARNESS: ENGAGED'}</span>
              </button>
            </div>
          )}
        </div>

        <div className="flex items-center gap-2">
          {hasSelection && (
            <button
              onClick={onDeleteSelected}
              className="radar-key radar-key-red text-[10px] py-1"
            >
              <span>PURGE TARGET</span>
            </button>
          )}

          <button
            onClick={() => setIsOpen(!isOpen)}
            className="radar-key radar-key-cyan text-[10px] py-1"
          >
            <span>{isOpen ? 'COLLAPSE LOG' : 'EXPAND TELEPRINTER'}</span>
          </button>
        </div>
      </div>

      {/* Expanded Military Teleprinter Log Table */}
      {isOpen && (
        <div className="p-3 border-t border-[#003310] max-h-48 overflow-y-auto text-[10px] bg-[#010903]">
          <table className="w-full text-left">
            <thead>
              <tr className="text-[#00aa33] border-b border-[#003310] uppercase">
                <th className="pb-1">CALLSIGN / TARGET</th>
                <th className="pb-1">DORSAL AZIMUTH</th>
                <th className="pb-1">CONSTELLATION STATUS</th>
                <th className="pb-1">BEACONS</th>
                <th className="pb-1">VELOCITY (ACT / KF)</th>
                <th className="pb-1">GDOP</th>
                <th className="pb-1">SPATIAL ERROR (Δ)</th>
                <th className="pb-1">HARNESS STATE</th>
              </tr>
            </thead>
            <tbody className="divide-y divide-[#00220a]">
              {tags.map((tag) => {
                const res = results.find(r => r.tagId === tag.id);
                return (
                  <tr key={tag.id} className="text-[#00cc3a] hover:bg-[#00220a]">
                    <td className="py-1 font-bold text-[#00f2fe]">
                      [TRK] {tag.name.toUpperCase()}
                    </td>
                    <td className="py-1 text-[#00f2fe]">
                      {Math.round(tag.headingDeg ?? 90)}° {tag.compassData?.cardinal ?? 'E'}
                    </td>
                    <td className="py-1">
                      {res?.lockType === 'FULL_3D_LOCK' ? (
                        <span className="text-[#00ff41] font-bold">
                          [3D-LOCK: MULTILATERAL]
                          {res.hypothesisFlipDetected && <span className="text-[#00f2fe] ml-1">(FLIP-RESOLVED)</span>}
                        </span>
                      ) : res?.lockType === 'DUAL_PINPOINT_LOCK' ? (
                        <span className="text-[#00f2fe] font-bold">
                          [2-BCN PINPOINT]
                          {res.dualInfo && ` (MATCH +${res.dualInfo.candidate1VectorAlignment})`}
                        </span>
                      ) : res?.lockType === 'DUAL_KINEMATIC_LOCK' ? (
                        <span className="text-[#ff9900]">
                          [2-BCN KINEMATIC]
                        </span>
                      ) : res?.lockType === 'DUAL_AMBIGUOUS_CHORD' ? (
                        <span className="text-[#ff9900]">
                          [2-BCN CHORD CROSSING]
                        </span>
                      ) : res?.lockType === 'RADIAL_RING' ? (
                        <span className="text-[#ff9900]">
                          [1-BCN RADIAL RING]
                        </span>
                      ) : (
                        <span className="text-[#ff3b30] font-bold">
                          [BLIND ZONE: 0 BEACONS]
                        </span>
                      )}
                    </td>
                    <td className="py-1 text-[#00f2fe]">
                      {res?.contributingAnchorIds.length || 0} BEACONS
                    </td>
                    <td className="py-1">
                      <span className="text-[#00f2fe]">{tag.speedMps ?? res?.actualSpeedMps ?? 0}</span>
                      <span className="text-[#007780]"> / </span>
                      <span className="text-[#00ff41]">{res?.speedMps ?? 0} M/S</span>
                    </td>
                    <td className="py-1">
                      {res ? (
                        <span className={res.gdop < 2.5 ? 'text-[#00ff41]' : res.gdop < 5 ? 'text-[#00f2fe]' : 'text-[#ff9900]'}>
                          {res.gdop}
                        </span>
                      ) : '—'}
                    </td>
                    <td className="py-1 font-bold">
                      {res?.lockType === 'FULL_3D_LOCK' ? (
                        <span className={res.errorMeters < 3 ? 'text-[#00ff41]' : 'text-[#ff9900]'}>±{res.errorMeters} M</span>
                      ) : res?.lockType === 'DUAL_PINPOINT_LOCK' ? (
                        <span className="text-[#00f2fe]">PINPOINT: ±{res.errorMeters} M</span>
                      ) : res?.lockType === 'DUAL_KINEMATIC_LOCK' ? (
                        <span className="text-[#ff9900]">±{res.errorMeters} M</span>
                      ) : res?.lockType === 'RADIAL_RING' ? (
                        <span className="text-[#ff9900]">R ≈ {res.radialRing?.radiusMeters} M</span>
                      ) : (
                        <span className="text-[#ff3b30]">—</span>
                      )}
                    </td>
                    <td className="py-1">
                      {tag.imuResting ? <span className="text-[#ff9900]">[DORMANT]</span> : <span className="text-[#00f2fe]">[PATROLLING]</span>}
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
