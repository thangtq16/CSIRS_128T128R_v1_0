function m = csirs_computeMetrics(fb, carrier)
%CSIRS_COMPUTEMETRICS  Derive RI, MCS, Throughput from CSI feedback struct.
%
%  MCS is computed via nr5g.internal.computeMCS (L2SM/MIESM internally),
%  giving the proper MCS index (0-based) per TS 38.214 Table 5.1.3.1-1 (qam64).
%
%  Throughput approximation (link-level, per slot):
%    TP = RI x ModOrder x CodeRate x N_RE_per_slot / T_slot
%
%  Inputs:
%    fb      - struct from csirs_feedback:
%                ri_B/C/D/E, cqi_B/C/D/E, cap_B/C/D/E
%                sinrPerRE_C, sinrPerRE_D, sinrPerRE_E  (linear [nRE x nLayers])
%    carrier - nrCarrierConfig (NSizeGrid, SubcarrierSpacing)
%
%  Output:
%    m - struct (each field is [1x4] vector, order: B C D E):
%          .ri  - RI per approach
%          .mcs - MCS index per approach (0-based, TS 38.214 Table 5.1.3.1-1)
%          .tp  - throughput in Mbps per approach

% ── Resource geometry ────────────────────────────────────────────────────
nPRB       = carrier.NSizeGrid;
nSym_PDSCH = 12;            % ~12 DMRS-free PDSCH symbols per slot
slotDur_s  = 1e-3 / (carrier.SubcarrierSpacing / 15);   % 0.5ms @ SCS 30kHz
nRE_slot   = nPRB * 12 * nSym_PDSCH;

% ── L2SM init ────────────────────────────────────────────────────────────
l2sm = nr5g.internal.L2SM.initialize(carrier);

% ── MCS via computeMCS for each approach ─────────────────────────────────
%  computeMCS(l2smSRS, carrier, pdsch, sinr_linear [nRE x nLayers], mcsTable)
%  nrPDSCHConfig is built manually (nrCSIReferenceResource not in local lib).

% Approach B: replicate per-layer wideband ZF SINR to [nRE x ri_B]
sinr_B = repmat(fb.sinr_B(:).', nPRB*12, 1);   % [nRE x ri_B], linear
pdsch_B = makePDSCH(carrier, fb.ri_B);
mcs_B = nr5g.internal.computeMCS(l2sm, carrier, pdsch_B, sinr_B, 'qam64');

% Approach C
pdsch_C = makePDSCH(carrier, fb.ri_C);
mcs_C = nr5g.internal.computeMCS(l2sm, carrier, pdsch_C, fb.sinrPerRE_C, 'qam64');

% Approach D
pdsch_D = makePDSCH(carrier, fb.ri_D);
mcs_D = nr5g.internal.computeMCS(l2sm, carrier, pdsch_D, fb.sinrPerRE_D, 'qam64');

% Approach E — ThangTQ23_128T128R_eTypeII_Rel19
pdsch_E = makePDSCH(carrier, fb.ri_E);
mcs_E = nr5g.internal.computeMCS(l2sm, carrier, pdsch_E, fb.sinrPerRE_E, 'qam64');

% ── Throughput from MCS index (TS 38.214 Table 5.1.3.1-1, qam64) ─────────
mcsTab = nrPDSCHMCSTables().QAM64Table;   % columns: MCSIndex Qm TargetCodeRate SE

tp_fn = @(ri, mcsIdx) computeTP(ri, mcsIdx, mcsTab, nRE_slot, slotDur_s);

tp_B = tp_fn(fb.ri_B, mcs_B) / 1e6;   % Mbps
tp_C = tp_fn(fb.ri_C, mcs_C) / 1e6;
tp_D = tp_fn(fb.ri_D, mcs_D) / 1e6;
tp_E = tp_fn(fb.ri_E, mcs_E) / 1e6;   % ThangTQ23_128T128R_eTypeII_Rel19

% ── Pack output ───────────────────────────────────────────────────────────
m.ri  = [fb.ri_B, fb.ri_C, fb.ri_D, fb.ri_E];
m.mcs = [mcs_B,   mcs_C,   mcs_D,   mcs_E];
m.tp  = [tp_B,    tp_C,    tp_D,    tp_E];

end

% ── Helper: build minimal nrPDSCHConfig for CSI reference resource ────────
function pdsch = makePDSCH(carrier, nLayers)
    pdsch           = nrPDSCHConfig;
    pdsch.PRBSet    = 0:carrier.NSizeGrid-1;
    pdsch.NumLayers = nLayers;
end

% ── Helper: TP from MCS index ─────────────────────────────────────────────
function tp = computeTP(ri, mcsIdx, mcsTab, nRE_slot, slotDur_s)
    row = mcsIdx + 1;   % mcsIdx is 0-based; MATLAB table is 1-based
    if row < 1 || row > height(mcsTab) || isnan(mcsTab.TargetCodeRate(row))
        tp = 0;
        return;
    end
    modOrder = mcsTab.Qm(row);
    codeRate = mcsTab.TargetCodeRate(row);   % in [0,1]
    tp = ri * modOrder * codeRate * nRE_slot / slotDur_s;
end
