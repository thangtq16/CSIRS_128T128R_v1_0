%% 128T128R NZP-CSI-RS Channel Estimation & CSI Feedback
%  Project: CSI-RS experiment for Massive MIMO 128T128R
%
%  Configuration:
%    - 128 CSI-RS ports = 4 NZP-CSI-RS Resources x 32 ports (Row 18, CDM8)
%    - 2-slot simulation: R0+R1 in Slot 0, R2+R3 in Slot 1
%    - Channel: CDL-C (spatial, suitable for massive MIMO)
%    - Pipeline: Signal Gen -> Channel -> CE -> CSI Feedback (PMI/RI/CQI)
%
%  References:
%    - TS 38.211 S7.4.1.5 (CSI-RS signal generation)
%    - TS 38.214 S5.2.2.3 (CSI reporting)
%    - TS 38.331 (NZP-CSI-RS-ResourceSet)
%    - R1-2500098 (Huawei, RAN1#120, Feb 2025) — Rel-19 MIMO Phase 5
%
%  Author: ThangTQ23 - VSI
%  Date:   2026-03

clear; close all; clc;
%rng(100);  % Reproducible

%% NR Cell Performance with Downlink MU-MIMO
% Custom Path Library
custom_lib_path_5g = fullfile(pwd, '5g');
custom_lib_path_wn = fullfile(pwd, 'wirelessnetwork');

addpath(genpath(custom_lib_path_5g));
addpath(genpath(custom_lib_path_wn));

rehash toolboxcache;

%% ========================================================================
%  SECTION 1: SYSTEM CONFIGURATION
%  ========================================================================

% --- Carrier ---
carrier = nrCarrierConfig;
carrier.NSizeGrid           = 52;          % 10 MHz @ SCS 30 kHz
carrier.SubcarrierSpacing   = 30;          % kHz
carrier.NSlot               = 0;
carrier.NFrame              = 0;
carrier.CyclicPrefix        = 'normal';

% --- Antenna Configuration ---
nTxAntennas  = 128;                        % gNB antennas
nRxAntennas  = 4;                          % UE antennas (typical)
nPortsPerRes = 32;                         % Ports per CSI-RS resource
nResources   = nTxAntennas / nPortsPerRes; % = 4 resources

fprintf('=== 128T128R CSI-RS Project ===\n');
fprintf('Carrier: %d PRBs, SCS %d kHz\n', carrier.NSizeGrid, carrier.SubcarrierSpacing);
fprintf('Antenna: %dT%dR\n', nTxAntennas, nRxAntennas);
fprintf('CSI-RS: %d resources x %d ports = %d total ports\n', ...
    nResources, nPortsPerRes, nTxAntennas);

%% ========================================================================
%  SECTION 2: CONFIGURE 4 NZP-CSI-RS RESOURCES (Row 18, CDM8)
%  ========================================================================
%
%  Row 18: 32 ports, CDM8 (FD-CDM2 x TD-CDM4), density 0.5
%    - k' = 0,1       (2 subcarriers per CDM group, FD-CDM2)
%    - l' = 0,1,2,3   (4 symbols per CDM group, TD-CDM4)
%    - 8 CDM groups x 4 ports/group = 32 ports
%    - SymbolLocations: single scalar l0; toolbox auto-expands to l0,l0+1,l0+2,l0+3
%
%  4 resources x 4 symbols/resource = 16 symbols > 14 symbols/slot
%  -> must spread across 2 consecutive slots
%
%  2-slot layout (R1-2500098 Rel-19 MIMO Phase 5):
%    Slot 0: R0 (l0=2, ports 0-31)   -> symbols 2,3,4,5
%            R1 (l0=8, ports 32-63)  -> symbols 8,9,10,11
%    Slot 1: R2 (l0=2, ports 64-95)  -> symbols 2,3,4,5
%            R3 (l0=8, ports 96-127) -> symbols 8,9,10,11

% l0 start symbol for each resource (within its slot) and slot assignment
l0_values  = [2, 8, 2, 8];   % Single scalar per resource; CDM8 auto-expands to 4 symbols
slotAssign = [0, 0, 1, 1];   % Slot 0 or 1 per resource

% Subcarrier locations for Row 18 (TS 38.211 Table 7.4.1.5.3-1)
% FD-CDM2, density 0.5: 4 k_i values = [0, 2, 4, 6]
subcarrierLoc = [0, 2, 4, 6];

% CDM type for Row 18: CDM8 = FD-CDM2 x TD-CDM4
cdmType    = 'CDM8';
cdmLengths = [2, 4];  % [FD-CDM2, TD-CDM4]

% Create CSI-RS config for each resource
csirs = cell(1, nResources);
for resIdx = 1:nResources
    csirs{resIdx} = nrCSIRSConfig;
    csirs{resIdx}.CSIRSType           = {'nzp'};
    csirs{resIdx}.CSIRSPeriod         = 'on';       % Always on
    csirs{resIdx}.RowNumber           = 18;
    csirs{resIdx}.Density             = {'one'};      % Fix 2: full PRB coverage
    csirs{resIdx}.SymbolLocations     = {l0_values(resIdx)};  % Single l0; CDM8 auto-expands
    csirs{resIdx}.SubcarrierLocations = {subcarrierLoc};
    csirs{resIdx}.NumRB               = carrier.NSizeGrid;
    csirs{resIdx}.NID                 = carrier.NCellID;

    fprintf('Resource #%d: Row 18, Slot %d, l0=%d -> symbols [%d,%d,%d,%d], ports %d-%d\n', ...
        resIdx-1, slotAssign(resIdx), l0_values(resIdx), ...
        l0_values(resIdx), l0_values(resIdx)+1, ...
        l0_values(resIdx)+2, l0_values(resIdx)+3, ...
        (resIdx-1)*32, resIdx*32-1);
end

%% ========================================================================
%  SECTION 3: GENERATE CSI-RS SYMBOLS & MAP TO 2-SLOT GRID
%  ========================================================================

% Power scaling
powerCSIRS_dB = 0;  % dB
powerScale = db2mag(powerCSIRS_dB);

% Initialize per-slot resource grids [K x 14 x 128]
carrier.NSlot = 0;
grid_slot0 = nrResourceGrid(carrier, nTxAntennas);
carrier.NSlot = 1;
grid_slot1 = nrResourceGrid(carrier, nTxAntennas);

allCsirsInd = cell(1, nResources);
allCsirsSym = cell(1, nResources);

for resIdx = 1:nResources
    slotNum = slotAssign(resIdx);
    carrier.NSlot = slotNum;

    % Generate symbols and indices for this resource in its assigned slot
    sym = nrCSIRS(carrier, csirs{resIdx});
    ind = nrCSIRSIndices(carrier, csirs{resIdx});

    % Apply power scaling
    sym = sym * powerScale;

    % Port offset: resource #k maps to ports (k-1)*32 : k*32-1
    portOffset = (resIdx - 1) * nPortsPerRes;

    % Remap indices from [K x L x 32] to [K x L x 128] port space.
    % nrCSIRSIndices returns 1-based linear indices relative to a grid of
    % size [nSC x nSymPerSlot x nPortsPerRes]. Shift port dimension only.
    gridSizePerRes = [carrier.NSizeGrid*12, carrier.SymbolsPerSlot, nPortsPerRes];
    [subInd, symInd, portInd] = ind2sub(gridSizePerRes, ind);
    portInd = portInd + portOffset;
    gridSizeFull = [carrier.NSizeGrid*12, carrier.SymbolsPerSlot, nTxAntennas];
    fullInd = sub2ind(gridSizeFull, subInd, symInd, portInd);

    % Map into the correct per-slot grid
    if slotNum == 0
        grid_slot0(fullInd) = sym;
    else
        grid_slot1(fullInd) = sym;
    end

    allCsirsInd{resIdx} = ind;
    allCsirsSym{resIdx} = sym;

    fprintf('Resource #%d (Slot %d): %d symbols, %d indices generated\n', ...
        resIdx-1, slotNum, length(sym), length(ind));
end

% Concatenate into 2-slot grid [K x 28 x 128]
txGrid_2slots = [grid_slot0, grid_slot1];
fprintf('txGrid_2slots size: [%s]\n', num2str(size(txGrid_2slots)));

%% ========================================================================
%  SECTION 4: VISUALIZE CSI-RS RESOURCE GRID (2 SLOTS)
%  ========================================================================

nSymbols_2slots = 2 * carrier.SymbolsPerSlot;  % 28

figure('Name', 'CSI-RS Resource Allocation (2 Slots)', 'Position', [100 100 1400 500]);

% Left: resource grid map over 28 symbols
subplot(1,2,1);
gridView = zeros(carrier.NSizeGrid*12, nSymbols_2slots);
colors = [1, 2, 3, 4];
for resIdx = 1:nResources
    gridSizePerRes = [carrier.NSizeGrid*12, carrier.SymbolsPerSlot, nPortsPerRes];
    [subInd, symInd, ~] = ind2sub(gridSizePerRes, allCsirsInd{resIdx});
    % Offset symbol index into the global 28-symbol span for slot 1 resources
    symInd_global = symInd + slotAssign(resIdx) * carrier.SymbolsPerSlot;
    uniquePos = unique([subInd, symInd_global], 'rows');
    for p = 1:size(uniquePos,1)
        gridView(uniquePos(p,1), uniquePos(p,2)) = colors(resIdx);
    end
end
imagesc(0:nSymbols_2slots-1, 1:carrier.NSizeGrid*12, gridView);
axis xy;
myColormap = [1 1 1; 0.2 0.4 0.8; 0.8 0.2 0.2; 0.2 0.7 0.3; 0.9 0.6 0.1];
colormap(myColormap);
xlabel('OFDM Symbol (Slot 0: 0-13,  Slot 1: 14-27)');
ylabel('Subcarrier');
title('CSI-RS Allocation (4 Resources, 2 Slots)');
xline(13.5, '--k', 'Slot boundary', 'LabelVerticalAlignment', 'bottom');
xlim([-0.5, nSymbols_2slots - 0.5]);
xticks(0:nSymbols_2slots-1);

% Right: occupancy bar chart over 28-symbol span
subplot(1,2,2);
occupancyMatrix = zeros(nSymbols_2slots, nResources);
for resIdx = 1:nResources
    gridSizePerRes = [carrier.NSizeGrid*12, carrier.SymbolsPerSlot, nPortsPerRes];
    [subInd, symInd, ~] = ind2sub(gridSizePerRes, allCsirsInd{resIdx});
    symInd_global = symInd + slotAssign(resIdx) * carrier.SymbolsPerSlot;
    uniquePos = unique([subInd, symInd_global], 'rows');
    for sym = 1:nSymbols_2slots
        occupancyMatrix(sym, resIdx) = sum(uniquePos(:,2) == sym);
    end
end
b = bar(0:nSymbols_2slots-1, occupancyMatrix, 'stacked', 'EdgeColor', 'none');
resourceColors = myColormap(2:end, :);
for k = 1:nResources
    b(k).FaceColor = resourceColors(k, :);
end
xline(13.5, '--k');
xlabel('OFDM Symbol');
ylabel('Total Occupied Subcarriers');
title('Symbol Occupancy (28-Symbol Span)');
xlim([-0.5, nSymbols_2slots - 0.5]);
xticks(0:nSymbols_2slots-1);
grid on;

%% ========================================================================
%  SECTION 5: CDL-C CHANNEL MODEL (SPATIAL)
%  ========================================================================
%
%  CDL-C: Cluster delay line model with spatial information
%  - Supports AoA/AoD -> essential for massive MIMO beamforming evaluation
%  - Antenna array geometry directly affects channel matrix

% --- gNB antenna array (aligned with Huawei R1-2500098 Appendix A) ---
% Virtualized: 4V x 16H x 2pol = 128 ports  [(N1,N2)=(16,4) in Huawei notation]
gnbArraySize = [4, 16, 2, 1, 1];  % [Nv, Nh, Npol, Mg, Ng]

% --- UE antenna array ---
% 4 antennas: 1V x 2H x 2pol = 4
ueArraySize = [1, 2, 2, 1, 1];

% --- CDL-C Channel (ONE object, used for the full 2-slot waveform) ---
channel = nrCDLChannel;
channel.DelayProfile         = 'CDL-C';
channel.DelaySpread          = 100e-9;       % 100ns (Urban Macro)
channel.MaximumDopplerShift  = 5;            % 5 Hz (low mobility, indoor/pedestrian)
channel.CarrierFrequency     = 3.5e9;        % 3.5 GHz (n78)

% Antenna array configuration
channel.TransmitAntennaArray.Size           = gnbArraySize;
channel.TransmitAntennaArray.ElementSpacing = [0.5 0.5 1 1];  % in wavelengths
channel.ReceiveAntennaArray.Size            = ueArraySize;
channel.ReceiveAntennaArray.ElementSpacing  = [0.5 0.5 1 1];

% Set sample rate from OFDM info
ofdmInfo = nrOFDMInfo(carrier);
channel.SampleRate = ofdmInfo.SampleRate;

fprintf('\n--- Channel Configuration ---\n');
fprintf('Channel model: CDL-C\n');
fprintf('Delay spread: %.0f ns\n', channel.DelaySpread*1e9);
fprintf('Doppler: %.1f Hz\n', channel.MaximumDopplerShift);
fprintf('Carrier freq: %.1f GHz\n', channel.CarrierFrequency/1e9);
fprintf('gNB array: %s -> %d antennas\n', mat2str(gnbArraySize), prod(gnbArraySize));
fprintf('UE array:  %s -> %d antennas\n', mat2str(ueArraySize), prod(ueArraySize));

%% ========================================================================
%  SECTION 6: TRANSMIT THROUGH CHANNEL + AWGN
%  ========================================================================

% OFDM modulation of full 2-slot grid (single call)
carrier.NSlot = 0;
[txWaveform, ofdmModInfo] = nrOFDMModulate(carrier, txGrid_2slots);

% Zero-pad for channel delay
chInfo = info(channel);
maxChDelay = chInfo.MaximumChannelDelay;
txWaveform = [txWaveform; zeros(maxChDelay, size(txWaveform, 2))];

% Pass through CDL-C channel ONCE with the full 2-slot waveform.
% The channel is stateful: internal time advances continuously, giving
% physically correct temporal correlation between slots. Do NOT split into
% 2 calls or reset between slots.
[rxWaveform, pathGains, sampleTimes] = channel(txWaveform);

% SNR configuration
SNRdB = 20;  % Realistic SNR for channel estimation evaluation
SNR = 10^(SNRdB/10);
N0 = 1/sqrt(2.0 * nRxAntennas * double(ofdmInfo.Nfft) * SNR);

rng(42);  % Reproducible
noise = N0 * complex(randn(size(rxWaveform)), randn(size(rxWaveform)));
rxWaveform = rxWaveform + noise;

fprintf('\n--- Transmission ---\n');
fprintf('SNR: %d dB\n', SNRdB);
fprintf('Noise variance N0: %.2e\n', N0^2);
fprintf('txWaveform size: [%s]\n', num2str(size(txWaveform)));

%% ========================================================================
%  SECTION 7: TIMING SYNC + OFDM DEMOD
%  ========================================================================

% Timing estimation using Resource #0 (Slot 0) as reference
carrier.NSlot = 0;
refSym = allCsirsSym{1};
refInd = allCsirsInd{1};
offset = nrTimingEstimate(carrier, rxWaveform, refInd, refSym);
fprintf('\nTiming offset: %d samples\n', offset);

rxWaveform = rxWaveform(1+offset:end, :);

% OFDM demodulation of full 2-slot waveform (single call)
carrier.NSlot = 0;
rxGrid_2slots = nrOFDMDemodulate(carrier, rxWaveform);
fprintf('rxGrid_2slots size: [%s]\n', num2str(size(rxGrid_2slots)));

% Split into per-slot grids for per-resource channel estimation
nSymPerSlot  = carrier.SymbolsPerSlot;  % 14
rxGrid_slot0 = rxGrid_2slots(:, 1:nSymPerSlot, :);
rxGrid_slot1 = rxGrid_2slots(:, (nSymPerSlot+1):(2*nSymPerSlot), :);

%% ========================================================================
%  SECTION 8: CHANNEL ESTIMATION (PER RESOURCE, THEN COMBINE)
%  ========================================================================
%
%  Each resource yields H_est of size [K x 14 x nRx x 32].
%  R0, R1 estimated from rxGrid_slot0; R2, R3 from rxGrid_slot1.
%  Combined H_est_full: [K x 28 x nRx x 128].
%  CDM lengths for Row 18: [2 4] (FD-CDM2 x TD-CDM4).

nSubcarriers = carrier.NSizeGrid * 12;
H_est_full   = zeros(nSubcarriers, nSymbols_2slots, nRxAntennas, nTxAntennas);
nVar_all     = zeros(1, nResources);

for resIdx = 1:nResources
    slotNum = slotAssign(resIdx);
    carrier.NSlot = slotNum;

    if slotNum == 0
        rxGrid_slot = rxGrid_slot0;
    else
        rxGrid_slot = rxGrid_slot1;
    end

    [H_est_res, nVar_res] = nrChannelEstimate(carrier, rxGrid_slot, ...
        allCsirsInd{resIdx}, allCsirsSym{resIdx}, ...
        'CDMLengths', cdmLengths);

    % H_est_res: [K x 14 x nRx x 32]
    portStart = (resIdx - 1) * nPortsPerRes + 1;
    portEnd   = resIdx * nPortsPerRes;
    symStart  = slotNum * nSymPerSlot + 1;
    symEnd    = (slotNum + 1) * nSymPerSlot;

    H_est_full(:, symStart:symEnd, :, portStart:portEnd) = H_est_res;
    nVar_all(resIdx) = nVar_res;

    fprintf('Resource #%d (Slot %d): H_est size [%s], nVar = %.4e\n', ...
        resIdx-1, slotNum, num2str(size(H_est_res)), nVar_res);
end

nVar_avg = mean(nVar_all);
fprintf('\nH_est_full size: [%s]\n', num2str(size(H_est_full)));
fprintf('Average noise variance: %.4e\n', nVar_avg);

%% ========================================================================
%  SECTION 9: CHANNEL ESTIMATION ERROR ANALYSIS
%  ========================================================================

% Perfect channel estimates for both slots (ground truth)
pathFilters = getPathFilters(channel);

carrier.NSlot = 0;
H_actual_slot0 = nrPerfectChannelEstimate(carrier, pathGains, pathFilters, offset, sampleTimes);
carrier.NSlot = 1;
H_actual_slot1 = nrPerfectChannelEstimate(carrier, pathGains, pathFilters, offset, sampleTimes);

% Combine into 2-slot ground truth [K x 28 x nRx x nTx]
H_actual = cat(2, H_actual_slot0, H_actual_slot1);

% Compare H_est_full vs H_actual.
% H_est_full is populated per resource in its own slot only:
%   Ports 1-64  (Res 0,1 — Slot 0): sym 1-14  non-zero, sym 15-28 = 0
%   Ports 65-128 (Res 2,3 — Slot 1): sym 15-28 non-zero, sym 1-14  = 0
% Comparing against H_actual (dense, all 28 sym) would count the zeros as
% error.  Correct approach: evaluate NMSE per resource at its own slot.
H_actual_trimmed = H_actual(:, :, :, 1:size(H_est_full, 4));

mse_per_port  = zeros(nTxAntennas, 1);
ref_per_port  = zeros(nTxAntennas, 1);
for resIdx = 1:nResources
    portStart = (resIdx-1)*nPortsPerRes + 1;
    portEnd   = resIdx*nPortsPerRes;
    slotNum   = slotAssign(resIdx);
    symStart  = slotNum*nSymPerSlot + 1;
    symEnd    = (slotNum+1)*nSymPerSlot;

    H_e = H_est_full(:, symStart:symEnd, :, portStart:portEnd);     % [K×14×nRx×32]
    H_a = H_actual_trimmed(:, symStart:symEnd, :, portStart:portEnd);
    err = H_e - H_a;
    for p = 1:nPortsPerRes
        mse_per_port(portStart+p-1) = mean(abs(err(:,:,:,p)).^2, 'all');
        ref_per_port(portStart+p-1) = mean(abs(H_a(:,:,:,p)).^2, 'all');
    end
end
nmse_per_port = mse_per_port ./ ref_per_port;

fprintf('\n--- Channel Estimation Error (per-resource, per-slot) ---\n');
fprintf('Overall MSE:  %.4e\n', mean(mse_per_port));
fprintf('Overall NMSE: %.2f dB\n', 10*log10(mean(nmse_per_port)));

figure('Name', 'Channel Estimation Quality', 'Position', [100 100 1400 400]);

subplot(1,3,1);
plot(0:nTxAntennas-1, 10*log10(nmse_per_port), '-o', 'MarkerSize', 3);
xlabel('CSI-RS Port Index'); ylabel('NMSE (dB)');
title('NMSE per Port (128 ports, 2 slots)');
grid on;
xline([32 64 96]-0.5, '--r', {'Res#1','Res#2','Res#3'});

subplot(1,3,2);
imagesc(abs(H_est_full(:, :, 1, 1)));
axis xy; colorbar;
xlabel('OFDM Symbol (0-27)'); ylabel('Subcarrier');
title('|H_{est}| (Rx0, Port0, 2 slots)');

subplot(1,3,3);
imagesc(abs(H_actual_trimmed(:, :, 1, 1)));
axis xy; colorbar;
xlabel('OFDM Symbol (0-27)'); ylabel('Subcarrier');
title('|H_{actual}| (Rx0, Port0, 2 slots)');

%% ========================================================================
%  SECTION 10: CSI FEEDBACK - PMI, RI, CQI
%  ========================================================================
%
%  Using Type I codebook (single panel) for 128 ports
%  TS 38.214 S5.2.2.2.1
%
%  NOTE: nrPMISelect / nrCQISelect support up to 32 ports.
%  For 128 ports, custom implementation or sub-array approach is needed.
%  Two approaches are demonstrated here:
%    (A) Per-resource PMI (32 ports each) - simple, uses built-in functions
%    (B) Full 128-port PMI via wideband SVD - approximate, no codebook

fprintf('\n=== CSI FEEDBACK ===\n');

% --- Approach A: Per-resource PMI/RI/CQI (32 ports each) ---
%
%  Uses nr5g.internal.nrRISelect / nrCQISelect (TS 38.214 Type-I Single Panel)
%  reportConfig must be a struct (not nrCSIReportConfig object)
%  PanelDimensions = [N1, N2] for Type1SinglePanel: [8,2] → 2×8×2 = 32 ports
%
fprintf('\n--- Approach A: Per-Resource PMI/RI/CQI (32 ports each) ---\n');

csiRepCfg = struct();
csiRepCfg.NSizeBWP        = carrier.NSizeGrid;  % 52 PRBs
csiRepCfg.NStartBWP       = 0;
csiRepCfg.CodebookType    = 'Type1SinglePanel';
csiRepCfg.PanelDimensions = [8, 2];             % N1=8, N2=2 → 32 ports
csiRepCfg.CodebookMode    = 1;
csiRepCfg.PMIMode         = 'Wideband';
csiRepCfg.CQIMode         = 'Wideband';
csiRepCfg.CQITable        = 'table1';

fprintf('  Config: Type-I Single Panel [N1=%d N2=%d] → %d ports/resource\n\n', ...
    csiRepCfg.PanelDimensions(1), csiRepCfg.PanelDimensions(2), ...
    2 * prod(csiRepCfg.PanelDimensions));
fprintf('  %-4s  %-8s  %-4s  %-14s  %-14s  %-6s\n', ...
    'Res', 'Ports', 'RI', 'PMI i1 (0-based)', 'PMI i2 (0-based)', 'CQI_WB');

ri_per_res  = zeros(1, nResources);
cqi_per_res = zeros(1, nResources);

for resIdx = 1:nResources
    portStart = (resIdx - 1) * nPortsPerRes + 1;
    portEnd   = resIdx * nPortsPerRes;
    slotNum   = slotAssign(resIdx);
    carrier.NSlot = slotNum;

    symStart = slotNum * nSymPerSlot + 1;
    symEnd   = (slotNum + 1) * nSymPerSlot;
    H_res    = H_est_full(:, symStart:symEnd, :, portStart:portEnd);  % [624×14×4×32]
    nVar_res = nVar_all(resIdx);

    % Step 1: RI selection
    [ri, ~, ~] = nr5g.internal.nrRISelect(carrier, csirs{resIdx}, csiRepCfg, H_res, nVar_res);
    ri_per_res(resIdx) = ri;

    % Step 2: CQI + PMI (given RI)
    [cqi, pmiSet, ~, ~] = nr5g.internal.nrCQISelect( ...
        carrier, csirs{resIdx}, csiRepCfg, ri, H_res, nVar_res);

    cqi_wb = cqi(1);                    % Wideband CQI (1st row)
    cqi_per_res(resIdx) = cqi_wb;

    % PMI indices: internal functions use 1-based → convert to 0-based (3GPP)
    i1_str = mat2str(pmiSet.i1 - 1);
    i2_str = mat2str(pmiSet.i2 - 1);

    fprintf('  R%d    %3d-%-3d   %-4d  %-14s  %-14s  %-6d\n', ...
        resIdx-1, portStart-1, portEnd-1, ri, i1_str, i2_str, cqi_wb);
end

fprintf('\n  Combined RI (min): %d   |   Avg CQI: %.1f\n', ...
    min(ri_per_res), mean(cqi_per_res));

% --- Approach B: Full 128-port SVD-based precoder ---
fprintf('\n--- Approach B: Full 128-Port SVD Precoder ---\n');

% Wideband channel: average per-resource at its own slot symbols, then combine.
% Averaging across all 28 symbols would dilute by 2x (half the symbols are
% zeros for each port range due to the 2-slot structure).
H_wb = zeros(nRxAntennas, nTxAntennas);
for resIdx = 1:nResources
    portStart = (resIdx-1)*nPortsPerRes + 1;
    portEnd   = resIdx*nPortsPerRes;
    slotNum   = slotAssign(resIdx);
    symStart  = slotNum*nSymPerSlot + 1;
    symEnd    = (slotNum+1)*nSymPerSlot;
    H_wb(:, portStart:portEnd) = squeeze(mean( ...
        H_est_full(:, symStart:symEnd, :, portStart:portEnd), [1 2]));
end
fprintf('H_wb size: [%s]\n', num2str(size(H_wb)));

% SVD decomposition
[U, S, V] = svd(H_wb, 'econ');
singularValues = diag(S);

% RI = number of singular values above threshold
riThreshold = 0.1 * singularValues(1);  % 10% of largest
ri_svd = sum(singularValues > riThreshold);
ri_svd = min(ri_svd, nRxAntennas);  % Cannot exceed nRx

fprintf('Singular values (top 8): [%s]\n', num2str(singularValues(1:min(8,end)).', '%.2f '));
fprintf('SVD-based RI: %d\n', ri_svd);

% Precoding matrix: first ri_svd right singular vectors
W_svd = V(:, 1:ri_svd);
fprintf('Precoder W size: [%s]\n', num2str(size(W_svd)));

% Effective channel after precoding
H_eff = H_wb * W_svd;  % [nRx x ri_svd]
fprintf('Effective channel H_eff size: [%s]\n', num2str(size(H_eff)));

% SINR estimation per layer (ZF receiver)
W_zf = pinv(H_eff);  % [ri_svd x nRx]
sinr_per_layer = zeros(ri_svd, 1);
for layer = 1:ri_svd
    signal       = abs(W_zf(layer,:) * H_eff(:,layer))^2;
    interference = sum(abs(W_zf(layer,:) * H_eff).^2) - signal;
    noise_term   = nVar_avg * norm(W_zf(layer,:))^2;
    sinr_per_layer(layer) = signal / (interference + noise_term);
end
sinr_dB = 10*log10(sinr_per_layer);
fprintf('Per-layer SINR (dB): [%s]\n', num2str(sinr_dB.', '%.1f '));

% CQI mapping (simplified, TS 38.214 Table 5.2.2.1-2)
cqi_table = [-6.7, -4.7, -2.3, 0.2, 2.4, 4.7, 6.9, 9.3, ...
    10.7, 12.2, 14.1, 15.6, 18.0, 20.3, 22.7];
cqi_svd = zeros(ri_svd, 1);
for layer = 1:ri_svd
    cqi_svd(layer) = max(sum(sinr_dB(layer) >= cqi_table), 1);  % Min CQI = 1
end
fprintf('Per-layer CQI: [%s]\n', num2str(cqi_svd.', '%d '));

%% ========================================================================
%  SECTION 11: BEAMFORMING PATTERN VISUALIZATION
%  ========================================================================

% Visualize beam pattern from SVD precoder (first layer)
figure('Name', 'Beamforming Analysis', 'Position', [100 100 1200 500]);

if ri_svd > 0
    % Beam pattern — azimuth cut with correct 4V x 16H x 2pol array dimensions
    subplot(1,2,1);
    w1   = W_svd(:,1);          % First layer precoder [128 x 1]
    nV   = gnbArraySize(1);     % 4 vertical elements
    nH   = gnbArraySize(2);     % 16 horizontal elements
    nPol = gnbArraySize(3);     % 2 polarizations

    azAngles = -90:0.5:90;
    d = 0.5;  % Element spacing in wavelengths

    beamPattern = zeros(length(azAngles), 1);
    for ai = 1:length(azAngles)
        az     = azAngles(ai) * pi / 180;
        a_h    = exp(1j * 2*pi * d * (0:nH-1).' * sin(az));
        a_v    = ones(nV, 1);          % Broadside in elevation for azimuth cut
        a_full = kron(ones(nPol,1), kron(a_v, a_h));
        beamPattern(ai) = abs(a_full' * w1)^2;
    end
    beamPattern_dB = 10*log10(beamPattern / max(beamPattern));

    plot(azAngles, beamPattern_dB, 'LineWidth', 1.5);
    xlabel('Azimuth (degrees)'); ylabel('Normalized Gain (dB)');
    title(sprintf('Beam Pattern (Azimuth Cut, Layer 1) — %dH x %dV x %dpol', nH, nV, nPol));
    grid on; ylim([-30 0]);
else
    subplot(1,2,1);
    text(0.5, 0.5, 'RI=0: no valid beam', 'HorizontalAlignment', 'center');
    axis off;
end

% Singular value spectrum
subplot(1,2,2);
bar(singularValues(1:min(20,end)));
xlabel('Singular Value Index'); ylabel('Magnitude');
title(sprintf('Channel Singular Values (RI=%d)', ri_svd));
grid on;

%% ========================================================================
%  SECTION 12: SUMMARY
%  ========================================================================

fprintf('\n');
fprintf('======================================================\n');
fprintf('         128T128R CSI-RS EXPERIMENT SUMMARY           \n');
fprintf('======================================================\n');
fprintf(' Carrier:     %3d PRBs, SCS %2d kHz\n', carrier.NSizeGrid, carrier.SubcarrierSpacing);
fprintf(' Antennas:    %dT x %dR\n', nTxAntennas, nRxAntennas);
fprintf(' CSI-RS:      %d x Row18 (32p, CDM8), 2 slots\n', nResources);
fprintf(' Channel:     CDL-C, DS=%.0fns, fD=%.0fHz\n', ...
    channel.DelaySpread*1e9, channel.MaximumDopplerShift);
fprintf(' SNR:         %d dB\n', SNRdB);
fprintf(' CE NMSE:     %.2f dB\n', 10*log10(mean(nmse_per_port)));
fprintf(' SVD RI:      %d layers\n', ri_svd);
fprintf(' SVD CQI:     [%s]\n', num2str(cqi_svd.', '%d '));
fprintf(' Layer SINR:  [%s] dB\n', num2str(sinr_dB.', '%.1f '));
fprintf('======================================================\n');
