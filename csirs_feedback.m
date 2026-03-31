function fb = csirs_feedback(carrier, csirs, H_est_full, nVar_all, slotAssign)
%CSIRS_FEEDBACK  CSI feedback (RI/PMI/CQI/Cap) for Approaches B, C, D.
%
%  Approach B: Full 128-port SVD upper bound (ideal reference)
%  Approach C: Rel-19 Type I Single-Panel Mode A (TS 38.214 S5.2.2.2.1a)
%  Approach D: Rel-19 Type I Single-Panel Mode B (TS 38.214 S5.2.2.2.1a)
%
%  CQI for C/D: computed via nr5g.internal.nrCQISelect (L2SM/MIESM,
%  BLER threshold 10%, per TS 38.214 S5.2.2.1) — now supports Rel-19.
%
%  Inputs:
%    carrier      - nrCarrierConfig
%    csirs        - {1x4} nrCSIRSConfig objects (csirs{1} used for C/D)
%    H_est_full   - [K x 28 x nRx x 128] estimated channel
%    nVar_all     - [1x4] noise variance per resource
%    slotAssign   - [1x4] slot index per resource
%
%  Output:
%    fb - struct:
%           .ri_B,  .cqi_B,  .cap_B,  .W_B
%           .ri_C,  .cqi_C,  .cap_C,  .W_C,  .sinrPerRE_C
%           .ri_D,  .cqi_D,  .cap_D,  .W_D,  .sinrPerRE_D
%         cqi_* is wideband scalar; cap_* is scalar (bits/s/Hz)

nRxAntennas  = size(H_est_full, 3);
nTxAntennas  = size(H_est_full, 4);
nPortsPerRes = nTxAntennas / 4;
nSymPerSlot  = carrier.SymbolsPerSlot;

% ── Wideband channel matrix H_wb [nRx x nTx] ─────────────────────────────
H_wb = zeros(nRxAntennas, nTxAntennas);
for r = 1:4
    pS = (r-1)*nPortsPerRes + 1;  pE = r*nPortsPerRes;
    sS = slotAssign(r)*nSymPerSlot + 1;
    sE = (slotAssign(r)+1)*nSymPerSlot;
    H_wb(:, pS:pE) = squeeze(mean(H_est_full(:, sS:sE, :, pS:pE), [1 2]));
end

nVar_wb = mean(nVar_all);

% Broadcast H_wb to [K x L x nRx x nTx] for nrPMIReport / nrCQISelect
carrier.NSlot = 0;
nSC = carrier.NSizeGrid * 12;
H_4d = repmat(reshape(H_wb, 1, 1, nRxAntennas, nTxAntennas), ...
              [nSC, carrier.SymbolsPerSlot, 1, 1]);

% ── Subband size per TS 38.214 Table 5.2.1.4-2 (smallest valid value) ────
nSizeBWP = carrier.NSizeGrid;
if nSizeBWP >= 145
    subbandSize = 16;
elseif nSizeBWP >= 73
    subbandSize = 8;
elseif nSizeBWP >= 24
    subbandSize = 4;
else
    subbandSize = [];   % BWP < 24 PRBs: no subbands
end

% ── Shared struct for nrCQISelect (Rel-19, now supported) ────────────────
cqiCfg.NSizeBWP        = carrier.NSizeGrid;
cqiCfg.NStartBWP       = 0;
cqiCfg.CodebookType    = 'typeI-SinglePanel-r19';
cqiCfg.PanelDimensions = [1, 16, 4];
cqiCfg.PMIMode         = 'Subband';
cqiCfg.SubbandSize     = subbandSize;
cqiCfg.CQIMode         = 'Wideband';
cqiCfg.CQITable        = 'table1';

% =========================================================================
%  APPROACH B: Full 128-Port SVD Upper Bound
%  CQI: ZF SINR per layer → threshold table (wideband SVD, no codebook grid)
% =========================================================================
[~, S_B, V_B]    = svd(H_wb, 'econ');
singVals_B       = diag(S_B);
snr_sv_B         = singVals_B.^2 / nVar_wb;
riThresh_B       = 0.1 * singVals_B(1);
ri_B             = min(sum(singVals_B > riThresh_B), nRxAntennas);
W_B              = V_B(:, 1:ri_B);

H_eff_B = H_wb * W_B;
W_zf_B  = pinv(H_eff_B);
sinr_B  = zeros(ri_B, 1);
for lyr = 1:ri_B
    sig  = abs(W_zf_B(lyr,:) * H_eff_B(:,lyr))^2;
    intf = sum(abs(W_zf_B(lyr,:) * H_eff_B).^2) - sig;
    nse  = nVar_wb * norm(W_zf_B(lyr,:))^2;
    sinr_B(lyr) = sig / (intf + nse);
end
cqi_tbl = [-6.7,-4.7,-2.3,0.2,2.4,4.7,6.9,9.3,10.7,12.2,14.1,15.6,18.0,20.3,22.7];
sinr_B_dB = 10*log10(sinr_B);
cqi_B     = max(sum(mean(sinr_B_dB) >= cqi_tbl), 1);   % wideband scalar
cap_B     = sum(log2(1 + snr_sv_B(1:ri_B) / ri_B));

fb.ri_B   = ri_B;
fb.cqi_B  = cqi_B;
fb.cap_B  = cap_B;
fb.W_B    = W_B;
fb.sinr_B = sinr_B;   % [ri_B x 1] per-layer wideband ZF SINR (linear)

% =========================================================================
%  APPROACH C: Rel-19 Mode A
%  RI: nrRISelect with typeI-SinglePanel-r19 (capacity maximisation)
%  CQI: nrCQISelect (L2SM/MIESM) — now supports typeI-SinglePanel-r19
% =========================================================================
riCfg_C.NSizeBWP        = carrier.NSizeGrid;
riCfg_C.NStartBWP       = 0;
riCfg_C.CodebookType    = 'typeI-SinglePanel-r19';
riCfg_C.PanelDimensions = [1, 16, 4];
riCfg_C.CodebookMode    = 1;
riCfg_C.PMIMode         = 'Subband';
riCfg_C.SubbandSize     = subbandSize;
[ri_C, ~, ~] = nr5g.internal.nrRISelect(carrier, csirs{1}, riCfg_C, H_4d, nVar_wb);

cqiCfg_C             = cqiCfg;
cqiCfg_C.CodebookMode = 1;
[cqi_C_vec, ~, ~, pmiInfo_C] = nr5g.internal.nrCQISelect( ...
    carrier, csirs{1}, cqiCfg_C, ri_C, H_4d, nVar_wb);
cqi_C = cqi_C_vec(1);   % wideband CQI (row 1 = wideband)
W_C   = pmiInfo_C.W(:,:,1);   % first subband precoder for wideband capacity approx
cap_C = real(log2(det(eye(nRxAntennas) + ...
    (1/nVar_wb) * (H_wb*W_C*(H_wb*W_C)'))));

fb.ri_C        = ri_C;
fb.cqi_C       = cqi_C;
fb.cap_C       = cap_C;
fb.W_C         = W_C;
fb.sinrPerRE_C = pmiInfo_C.SINRPerREPMI;

% =========================================================================
%  APPROACH D: Rel-19 Mode B
%  RI: nrRISelect with typeI-SinglePanel-r19 (capacity maximisation)
%  CQI: nrCQISelect (L2SM/MIESM) — typeI-SinglePanel-r19
% =========================================================================
riCfg_D.NSizeBWP        = carrier.NSizeGrid;
riCfg_D.NStartBWP       = 0;
riCfg_D.CodebookType    = 'typeI-SinglePanel-r19';
riCfg_D.PanelDimensions = [1, 16, 4];
riCfg_D.CodebookMode    = 2;
riCfg_D.PMIMode         = 'Subband';
riCfg_D.SubbandSize     = subbandSize;
[ri_D, ~, ~] = nr5g.internal.nrRISelect(carrier, csirs{1}, riCfg_D, H_4d, nVar_wb);

cqiCfg_D             = cqiCfg;
cqiCfg_D.CodebookMode = 2;
[cqi_D_vec, ~, ~, pmiInfo_D] = nr5g.internal.nrCQISelect( ...
    carrier, csirs{1}, cqiCfg_D, ri_D, H_4d, nVar_wb);
cqi_D = cqi_D_vec(1);
W_D   = pmiInfo_D.W;
cap_D = real(log2(det(eye(nRxAntennas) + ...
    (1/nVar_wb) * (H_wb*W_D*(H_wb*W_D)'))));

fb.ri_D        = ri_D;
fb.cqi_D       = cqi_D;
fb.cap_D       = cap_D;
fb.W_D         = W_D;
fb.sinrPerRE_D = pmiInfo_D.SINRPerREPMI;

end
