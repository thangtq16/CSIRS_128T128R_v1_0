function fb = csirs_feedback(carrier, csirs, H_est_full, nVar_all, slotAssign, pmiMode, cqiMode, panelDimensions, paramCombE)
%CSIRS_FEEDBACK  CSI feedback (RI/PMI/CQI/Cap) for Approaches B, C, D, E.
%
%  Approach B: Full 128-port SVD upper bound (ideal reference)
%  Approach C: Rel-19 Type I Single-Panel Mode A (TS 38.214 S5.2.2.2.1a)
%  Approach D: Rel-19 Type I Single-Panel Mode B (TS 38.214 S5.2.2.2.1a)
%  Approach E: Rel-19 Refined eTypeII 128-port (TS 38.214 S5.2.2.2.5a)
%              ThangTQ23_128T128R_eTypeII_Rel19
%
%  CQI for C/D/E: computed via nr5g.internal.nrCQISelect (L2SM/MIESM,
%  BLER threshold 10%, per TS 38.214 S5.2.2.1) — now supports Rel-19.
%
%  Inputs:
%    carrier      - nrCarrierConfig
%    csirs        - {1x4} nrCSIRSConfig objects (csirs{1} used for C/D/E)
%    H_est_full   - [K x 28 x nRx x 128] estimated channel
%    nVar_all     - [1x4] noise variance per resource
%    slotAssign   - [1x4] slot index per resource
%    pmiMode      - (optional) 'Subband' (default) or 'Wideband'
%    cqiMode      - (optional) 'Wideband' (default) or 'Subband'
%                   When 'Subband': fb.cqi_C/D/E are vectors [1+numSB x 1]
%                   Row 1 = wideband CQI, rows 2..end = per-subband CQI
%    panelDimensions - (optional) [Ng N1 N2], default [1 16 4]
%    paramCombE   - (optional) ParameterCombination for Approach E, default 1
%                   Valid: 1-8 per TS 38.214 Table 5.2.2.2.5-1
%                   PC 7/8: L=6, maxRank capped at 2 (pv for rank 3/4 = "-")
%
%  Output:
%    fb - struct:
%           .ri_B,  .cqi_B,  .cap_B,  .W_B
%           .ri_C,  .cqi_C,  .cap_C,  .W_C,  .sinrPerRE_C
%           .ri_D,  .cqi_D,  .cap_D,  .W_D,  .sinrPerRE_D
%           .ri_E,  .cqi_E,  .cap_E,  .W_E,  .sinrPerRE_E
%         cqi_C/D/E: wideband scalar (CQIMode='Wideband')
%                    or [1+numSB x 1] vector (CQIMode='Subband'),
%                    row 1 = wideband, rows 2..end = per-subband
%                    Note: Mode D beam is always wideband (spec constraint),
%                    but per-subband CQI with that beam is still valid.

if nargin < 6 || isempty(pmiMode)
    pmiMode = 'Subband';
end
pmiMode = validatestring(pmiMode, {'Subband','Wideband'});

if nargin < 7 || isempty(cqiMode)
    cqiMode = 'Wideband';
end
cqiMode = validatestring(cqiMode, {'Wideband','Subband'});

nRxAntennas  = size(H_est_full, 3);
nTxAntennas  = size(H_est_full, 4);
nResources   = length(csirs);
nPortsPerRes = nTxAntennas / nResources;

if nargin < 8 || isempty(panelDimensions)
    panelDimensions = [1, 16, 4];   % fallback: [Ng, N1=Nh, N2=Nv]
end

% ThangTQ23_128T128R_eTypeII_Rel19: ParameterCombination for Approach E
if nargin < 9 || isempty(paramCombE)
    paramCombE = 5;   % L=4, pv(v=1..4)=1/4, β=1/4 — supports rank 1~4
end
nSymPerSlot  = carrier.SymbolsPerSlot;

% ── Wideband channel matrix H_wb [nRx x nTx] ─────────────────────────────
H_wb = zeros(nRxAntennas, nTxAntennas);
for r = 1:nResources
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
cqiCfg.PanelDimensions = panelDimensions;
cqiCfg.PMIMode         = pmiMode;
cqiCfg.SubbandSize     = subbandSize;
cqiCfg.CQIMode         = cqiMode;
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
riCfg_C.PanelDimensions = panelDimensions;
riCfg_C.CodebookMode    = 1;
riCfg_C.PMIMode         = pmiMode;
riCfg_C.SubbandSize     = subbandSize;
[ri_C, ~, ~] = nr5g.internal.nrRISelect(carrier, csirs{1}, riCfg_C, H_4d, nVar_wb);

cqiCfg_C             = cqiCfg;
cqiCfg_C.CodebookMode = 1;
[cqi_C_vec, ~, ~, pmiInfo_C] = nr5g.internal.nrCQISelect( ...
    carrier, csirs{1}, cqiCfg_C, ri_C, H_4d, nVar_wb);
% cqi_C_vec: row 1 = wideband, rows 2..end = per-subband (when CQIMode='Subband')
if strcmpi(cqiMode, 'Subband')
    cqi_C = cqi_C_vec;   % full vector [1+numSB x 1]
else
    cqi_C = cqi_C_vec(1);
end
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
riCfg_D.PanelDimensions = panelDimensions;
riCfg_D.CodebookMode    = 2;
riCfg_D.PMIMode         = pmiMode;
riCfg_D.SubbandSize     = subbandSize;
[ri_D, ~, ~] = nr5g.internal.nrRISelect(carrier, csirs{1}, riCfg_D, H_4d, nVar_wb);

cqiCfg_D             = cqiCfg;
cqiCfg_D.CodebookMode = 2;
% Mode B beam group (i1) is always wideband (spec constraint).
% Co-phase (i2) follows pmiMode: 'Subband' activates Phase 2 per-subband
% co-phase in nrPMIReport, giving consistent W between RI and CQI steps.
cqiCfg_D.PMIMode      = pmiMode;
[cqi_D_vec, ~, ~, pmiInfo_D] = nr5g.internal.nrCQISelect( ...
    carrier, csirs{1}, cqiCfg_D, ri_D, H_4d, nVar_wb);
% cqi_D_vec: row 1 = wideband, rows 2..end = per-subband (when CQIMode='Subband')
if strcmpi(cqiMode, 'Subband')
    cqi_D = cqi_D_vec;   % full vector [1+numSB x 1]
else
    cqi_D = cqi_D_vec(1);
end
W_D   = pmiInfo_D.W(:,:,1);   % first subband precoder for wideband capacity approx
cap_D = real(log2(det(eye(nRxAntennas) + ...
    (1/nVar_wb) * (H_wb*W_D*(H_wb*W_D)'))));

fb.ri_D        = ri_D;
fb.cqi_D       = cqi_D;
fb.cap_D       = cap_D;
fb.W_D         = W_D;
fb.sinrPerRE_D = pmiInfo_D.SINRPerREPMI;

% =========================================================================
%  APPROACH E: Refined eTypeII (TS 38.214 §5.2.2.2.5a, Rel-19, 128 ports)
%  ThangTQ23_128T128R_eTypeII_Rel19
%
%  L=2 (PC 1/2): nrRISelect + nrCQISelect via nrPMIReport — spec-accurate,
%                C(1024,2)≈31K entries, fits memory, CQI from L2SM.
%  L≥4 (PC 3-8): greedy DFT beam selection — avoids OOM of exhaustive
%                search (C(1024,4)≈46×10^9 entries × 128×4 → >5 GB).
%                CQI via wideband ZF SINR (same method as Approach B).
% =========================================================================
L_tableE = [2 2 4 4 4 4 6 6];
L_E      = L_tableE(paramCombE);
maxRankE = min(nRxAntennas, 4);
if paramCombE >= 7
    maxRankE = min(nRxAntennas, 2);   % PC 7/8: pv(v=3,4)="-" → maxRank=2
end

if L_E <= 2
    % ── L=2: full toolbox pipeline (nrPMIReport exhaustive, tractable) ───
    riCfg_E.NSizeBWP                         = carrier.NSizeGrid;
    riCfg_E.NStartBWP                        = 0;
    riCfg_E.CodebookType                     = 'eTypeII-r19';
    riCfg_E.PanelDimensions                  = panelDimensions;
    riCfg_E.ParameterCombination             = paramCombE;
    riCfg_E.NumberOfPMISubbandsPerCQISubband = 1;
    riCfg_E.PMIMode                          = pmiMode;
    riCfg_E.SubbandSize                      = subbandSize;
    [ri_E, ~, ~] = nr5g.internal.nrRISelect(carrier, csirs{1}, riCfg_E, H_4d, nVar_wb);

    cqiCfg_E.NSizeBWP                         = carrier.NSizeGrid;
    cqiCfg_E.NStartBWP                        = 0;
    cqiCfg_E.CodebookType                     = 'eTypeII-r19';
    cqiCfg_E.PanelDimensions                  = panelDimensions;
    cqiCfg_E.ParameterCombination             = paramCombE;
    cqiCfg_E.NumberOfPMISubbandsPerCQISubband = 1;
    cqiCfg_E.PMIMode                          = pmiMode;
    cqiCfg_E.SubbandSize                      = subbandSize;
    cqiCfg_E.CQIMode                          = cqiMode;
    cqiCfg_E.CQITable                         = 'table1';
    % Retry loop: wideband RI selection may pick a rank that the subband
    % eTypeII quantization (pv, L) can't represent → nrCQISelect returns NaN.
    % Decrement ri_E until a valid CQI is obtained.
    cqi_E_vec = NaN;
    while ri_E >= 1 && isnan(cqi_E_vec(1))
        [cqi_E_vec, ~, ~, pmiInfo_E] = nr5g.internal.nrCQISelect( ...
            carrier, csirs{1}, cqiCfg_E, ri_E, H_4d, nVar_wb);
        if isnan(cqi_E_vec(1))
            ri_E = ri_E - 1;
        end
    end
    if strcmpi(cqiMode, 'Subband')
        cqi_E = cqi_E_vec;
    else
        cqi_E = cqi_E_vec(1);
    end
    W_E         = pmiInfo_E.W(:,:,1);
    cap_E       = real(log2(det(eye(nRxAntennas) + ...
        (1/nVar_wb) * (H_wb*W_E*(H_wb*W_E)'))));
    sinrPerRE_E = pmiInfo_E.SINRPerREPMI;

else
    % ── L≥4: greedy DFT beam selection (OOM-safe) ────────────────────────
    % nrPMIReport pre-allocates Precoders(nTx,nLayers,C(nGrid,L),q1,q2):
    % C(1024,4)×128×4 entries > 5 GB → OOM. Greedy: O(L×nGrid) per rank.
    bestCap_E = -Inf;
    ri_E = 1;
    W_E  = zeros(nTxAntennas, 1);
    for rank_E = 1:maxRankE
        W_try  = greedyEType2R19Precoder(H_wb, panelDimensions, L_E, rank_E);
        HW_try = H_wb * W_try;
        cap_try = real(log2(det(eye(nRxAntennas) + ...
            (1/nVar_wb) * (HW_try * HW_try'))));
        if cap_try > bestCap_E
            bestCap_E = cap_try;  ri_E = rank_E;  W_E = W_try;
        end
    end
    cap_E = bestCap_E;

    % SINR per layer: ZF receiver on wideband H (same as Approach B)
    H_eff_E = H_wb * W_E;
    W_zf_E  = pinv(H_eff_E);
    sinr_E  = zeros(ri_E, 1);
    for lyr = 1:ri_E
        sig  = abs(W_zf_E(lyr,:) * H_eff_E(:,lyr))^2;
        intf = sum(abs(W_zf_E(lyr,:) * H_eff_E).^2) - sig;
        nse  = nVar_wb * norm(W_zf_E(lyr,:))^2;
        sinr_E(lyr) = sig / (intf + nse);
    end

    % CQI: wideband SINR → table lookup (same cqi_tbl as Approach B)
    sinr_E_dB = 10*log10(max(sinr_E, 1e-10));
    cqi_E     = max(sum(mean(sinr_E_dB) >= cqi_tbl), 1);

    % sinrPerRE_E: replicate wideband SINR [nRE × ri_E] for computeMetrics
    sinrPerRE_E = repmat(sinr_E.', carrier.NSizeGrid * 12, 1);
end

fb.ri_E        = ri_E;
fb.cqi_E       = cqi_E;
fb.cap_E       = cap_E;
fb.W_E         = W_E;
fb.sinrPerRE_E = sinrPerRE_E;

end

% =========================================================================
%  LOCAL HELPER: Greedy DFT beam selection for eTypeII-r19 (L≥4 path)
%  ThangTQ23_128T128R_eTypeII_Rel19
% =========================================================================
function W = greedyEType2R19Precoder(H_wb, panelDimensions, L, nLayers)
%GREEDYETYPE2R19PRECODER  eTypeII-r19 precoder via greedy DFT beam selection.
%
%  Dual-pol DFT codebook structure per TS 38.214 §5.2.2.2.5:
%    W = W1 * W2
%  W1: L DFT beams per polarization selected by greedy matching pursuit.
%  W2: combining coefficients from projected SVD.
%
%  Inputs:
%    H_wb          - [nRx × 2*N1*N2] wideband channel matrix
%    panelDimensions - [Ng, N1, N2]
%    L             - number of beams
%    nLayers       - number of spatial layers (rank)
%
%  Output:
%    W - [2*N1*N2 × nLayers] unit-Frobenius-norm precoder (||W||_F = 1)

    N1 = panelDimensions(2);
    N2 = panelDimensions(3);
    O1 = 4;  O2 = 4;          % fixed oversampling per §5.2.2.2.5a
    nPerPol = N1 * N2;

    % ── Oversampled DFT grid A [nPerPol × O1*N1*O2*N2] ──────────────────
    % u_{n1,n2} = a_{N1,O1}(n1) ⊗ a_{N2,O2}(n2)  (TS 38.214 §5.2.2.2.3)
    k1 = (0:N1-1)';  k2 = (0:N2-1)';
    nGrid = N1*O1 * N2*O2;
    A = zeros(nPerPol, nGrid);
    col = 0;
    for n1 = 0:N1*O1-1
        a1 = exp(1j*2*pi*n1*k1/(N1*O1)) / sqrt(N1);
        for n2 = 0:N2*O2-1
            col = col + 1;
            a2 = exp(1j*2*pi*n2*k2/(N2*O2)) / sqrt(N2);
            A(:, col) = kron(a1, a2);
        end
    end

    % ── Greedy matching pursuit — shared beams for both polarizations ─────
    H1 = H_wb(:, 1:nPerPol);        % pol-1  [nRx × nPerPol]
    H2 = H_wb(:, nPerPol+1:end);    % pol-2  [nRx × nPerPol]
    R1 = H1;  R2 = H2;
    B  = zeros(nPerPol, L);
    for l = 1:L
        scores     = sum(abs(R1*A).^2 + abs(R2*A).^2, 1);
        [~, best]  = max(scores);
        a_sel      = A(:, best);
        B(:, l)    = a_sel;
        R1 = R1 - (R1 * a_sel) * a_sel';   % orthogonal deflation
        R2 = R2 - (R2 * a_sel) * a_sel';
    end

    % ── W1: dual-pol block-diagonal [2*nPerPol × 2L] ─────────────────────
    W1 = [B, zeros(nPerPol,L); zeros(nPerPol,L), B];

    % ── W2: combining coefficients via projected SVD [2L × nLayers] ──────
    [~, ~, Vp] = svd(H_wb * W1, 'econ');
    W2 = Vp(:, 1:min(nLayers, size(Vp,2)));

    % ── W = W1 * W2, unit Frobenius norm (||W||_F = 1) ───────────────────
    % Per TS 38.214 §5.2.2.2.5: W2 normalized s.t. ||W||_F=1 → total TX
    % power = 1, consistent with Approach B (sum log2(1+sv²/(r*nVar)))
    % and nrPMIReport output for Approach C/D.
    W = W1 * W2;
    W_fro = norm(W, 'fro');
    if W_fro > 1e-10
        W = W / W_fro;
    end
end
