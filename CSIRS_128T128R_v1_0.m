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
%    - TS 38.214 S5.2.2.2.1  (Type I Single Panel, Rel-15/16)
%    - TS 38.214 S5.2.2.2.1a (Refined Type I Single Panel, Rel-19)
%    - TS 38.331 (NZP-CSI-RS-ResourceSet)
%
%  Author: ThangTQ23 - VSI
%  Date:   2026-03

clear; close all; clc;
%rng(100);  % Reproducible

%% Approach Selection
%  'A'   - Rel-15/16 per-32-port Type I Single Panel (TS 38.214 S5.2.2.2.1)
%  'B'   - Full 128-port SVD upper bound (ideal reference)
%  'C'   - Rel-19 Type I Single-Panel Mode A (TS 38.214 S5.2.2.2.1a)
%  'D'   - Rel-19 Type I Single-Panel Mode B (TS 38.214 S5.2.2.2.1a)
%  'ALL' - Run all approaches and print comparison table
selectedApproach = 'ALL';

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
carrier.NSizeGrid           = 273;         % 100 MHz @ SCS 30 kHz
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
%  2-slot layout (TS 38.214 S5.2.2.2.1a, Rel-19):
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

% --- gNB antenna array: 4V x 16H x 2pol = 128 ports ---
% Virtualized: [Nv=4, Nh=16, Npol=2] -> (N1,N2)=(16,4) in TS 38.214 notation
gnbArraySize = [4, 16, 2, 1, 1];  % [Nv, Nh, Npol, Mg, Ng]

% --- UE antenna array ---
% 4 antennas: 1V x 2H x 2pol = 4
ueArraySize = [1, 2, 2, 1, 1];

% --- CDL Channel (ONE object, used for the full 2-slot waveform) ---
% CDL-B: near-uniform SV distribution [12,11,10,9] dB → best for modeA RI=4 test
% CDL-C: urban macro, concentrated → SV1 dominant, modeA gives RI=1 (original)
% CDL-D/E: LOS dominant → even worse for rank
channel = nrCDLChannel;
channel.DelayProfile         = 'CDL-B';     % Changed: CDL-C→CDL-B for rank-4 test
channel.DelaySpread          = 450e-9;       % 450ns (Urban Macro)
channel.MaximumDopplerShift  = 136;          % 136 Hz (30 km/h @ 4.9 GHz)
channel.CarrierFrequency     = 4.9e9;        % 4.9 GHz

% Antenna array configuration
channel.TransmitAntennaArray.Size           = gnbArraySize;
channel.TransmitAntennaArray.ElementSpacing = [0.5 0.5 1 1];  % in wavelengths
channel.ReceiveAntennaArray.Size            = ueArraySize;
channel.ReceiveAntennaArray.ElementSpacing  = [0.5 0.5 1 1];

% Set sample rate from OFDM info
ofdmInfo = nrOFDMInfo(carrier);
channel.SampleRate = ofdmInfo.SampleRate;

fprintf('\n--- Channel Configuration ---\n');
fprintf('Channel model: %s\n', channel.DelayProfile);
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
%  Approach A: Rel-15/16 per-32-port Type I Single Panel (TS 38.214 S5.2.2.2.1)
%  Approach B: Full 128-port SVD upper bound (ideal capacity reference)
%  Approach C: Rel-19 Type I Single-Panel Mode A (TS 38.214 S5.2.2.2.1a)
%  Approach D: Rel-19 Type I Single-Panel Mode B (TS 38.214 S5.2.2.2.1a)

fprintf('\n=== CSI FEEDBACK (Approach: %s) ===\n', selectedApproach);

% --- Common: wideband channel matrix H_wb [nRx x nTx] ---
% Average per-resource over its own slot symbols to avoid cross-slot dilution.
H_wb = zeros(nRxAntennas, nTxAntennas);
for resIdx = 1:nResources
    pS = (resIdx-1)*nPortsPerRes + 1;  pE = resIdx*nPortsPerRes;
    sS = slotAssign(resIdx)*nSymPerSlot + 1;
    sE = (slotAssign(resIdx)+1)*nSymPerSlot;
    H_wb(:, pS:pE) = squeeze(mean(H_est_full(:,sS:sE,:,pS:pE), [1 2]));
end
% Broadcast H_wb to [K x L x nRx x nTx] for nrPMIReport (slot-0 grid size)
carrier.NSlot = 0;
nSC  = carrier.NSizeGrid * 12;
nSym = carrier.SymbolsPerSlot;
H_4d = repmat(reshape(H_wb, 1, 1, nRxAntennas, nTxAntennas), [nSC nSym 1 1]);
nVar_wb  = mean(nVar_all);
cqi_tbl  = [-6.7,-4.7,-2.3,0.2,2.4,4.7,6.9,9.3,10.7,12.2,14.1,15.6,18.0,20.3,22.7];
ri_max   = min(nRxAntennas, 4);

% =========================================================================
%  APPROACH A: Rel-15/16 Per-32-Port Type I Single Panel
% =========================================================================
if ismember(selectedApproach, {'A','ALL'})
    fprintf('\n--- Approach A: Rel-15/16 Per-32-Port Type I Single Panel ---\n');
    % reportConfig must be a struct (TS 38.214 S5.2.2.2.1)
    % PanelDimensions [N1,N2]: [8,2] -> 2x8x2 = 32 ports per resource
    csiRepCfg = struct();
    csiRepCfg.NSizeBWP        = carrier.NSizeGrid;
    csiRepCfg.NStartBWP       = 0;
    csiRepCfg.CodebookType    = 'Type1SinglePanel';
    csiRepCfg.PanelDimensions = [8, 2];
    csiRepCfg.CodebookMode    = 1;
    csiRepCfg.PMIMode         = 'Wideband';
    csiRepCfg.CQIMode         = 'Wideband';
    csiRepCfg.CQITable        = 'table1';
    fprintf('  Config: [N1=%d N2=%d] -> %d ports/resource\n\n', ...
        csiRepCfg.PanelDimensions(1), csiRepCfg.PanelDimensions(2), ...
        2*prod(csiRepCfg.PanelDimensions));
    fprintf('  %-4s  %-8s  %-4s  %-14s  %-14s  %-6s  %-16s\n', ...
        'Res', 'Ports', 'RI', 'PMI i1 (0-based)', 'PMI i2 (0-based)', 'CQI_WB', 'Cap (bits/s/Hz)');
    ri_per_res_A  = zeros(1, nResources);
    cqi_per_res_A = zeros(1, nResources);
    cap_per_res_A = zeros(1, nResources);
    for resIdx = 1:nResources
        portStart = (resIdx-1)*nPortsPerRes + 1;
        portEnd   = resIdx*nPortsPerRes;
        slotNum   = slotAssign(resIdx);
        carrier.NSlot = slotNum;
        symStart  = slotNum*nSymPerSlot + 1;
        symEnd    = (slotNum+1)*nSymPerSlot;
        H_res     = H_est_full(:, symStart:symEnd, :, portStart:portEnd);
        nVar_res  = nVar_all(resIdx);
        [ri_res, ~, ~] = nr5g.internal.nrRISelect( ...
            carrier, csirs{resIdx}, csiRepCfg, H_res, nVar_res);
        [cqi_res, pmiSet_res, ~, ~] = nr5g.internal.nrCQISelect( ...
            carrier, csirs{resIdx}, csiRepCfg, ri_res, H_res, nVar_res);
        ri_per_res_A(resIdx)  = ri_res;
        cqi_per_res_A(resIdx) = cqi_res(1);
        % Capacity: SVD of this 32-port sub-block, equal power across ri_res layers
        H_sub = H_wb(:, portStart:portEnd);
        [~, S_sub, ~] = svd(H_sub, 'econ');
        sv_sub = diag(S_sub);
        cap_per_res_A(resIdx) = sum(log2(1 + sv_sub(1:ri_res).^2 / (nVar_wb * ri_res)));
        fprintf('  R%d    %3d-%-3d   %-4d  %-14s  %-14s  %-6d  %.2f\n', ...
            resIdx-1, portStart-1, portEnd-1, ri_res, ...
            mat2str(pmiSet_res.i1-1), mat2str(pmiSet_res.i2-1), cqi_res(1), cap_per_res_A(resIdx));
    end
    % Each resource is a 32-port sub-array covering the same time-frequency REs.
    % The gNB selects the best-performing sub-array -> capacity = max over resources.
    [cap_A, idx_best_A] = max(cap_per_res_A);
    ri_A      = ri_per_res_A(idx_best_A);
    cqi_A_avg = mean(cqi_per_res_A);
    fprintf('\n  Best sub-array: R%d  |  RI=%d  Cap=%.2f bits/s/Hz  |  Avg CQI (all res)=%.1f\n', ...
        idx_best_A-1, ri_A, cap_A, cqi_A_avg);
end

% =========================================================================
%  APPROACH B: Full 128-Port SVD Upper Bound (ideal reference)
% =========================================================================
if ismember(selectedApproach, {'B','ALL'})
    fprintf('\n--- Approach B: Full 128-Port SVD Upper Bound ---\n');
    [~, S_B, V_B]    = svd(H_wb, 'econ');
    singularValues_B = diag(S_B);
    snr_sv_B         = singularValues_B.^2 / nVar_wb;
    riThresh_B       = 0.1 * singularValues_B(1);
    ri_B             = min(sum(singularValues_B > riThresh_B), nRxAntennas);
    W_B              = V_B(:, 1:ri_B);
    H_eff_B          = H_wb * W_B;
    W_zf_B           = pinv(H_eff_B);
    sinr_B           = zeros(ri_B, 1);
    for layer = 1:ri_B
        sig  = abs(W_zf_B(layer,:) * H_eff_B(:,layer))^2;
        intf = sum(abs(W_zf_B(layer,:) * H_eff_B).^2) - sig;
        nse  = nVar_wb * norm(W_zf_B(layer,:))^2;
        sinr_B(layer) = sig / (intf + nse);
    end
    sinr_B_dB = 10*log10(sinr_B);
    cqi_B     = arrayfun(@(s) max(sum(s >= cqi_tbl), 1), sinr_B_dB);
    cap_B     = sum(log2(1 + snr_sv_B(1:ri_B) / ri_B));  % total power = 1 (1/ri per stream)
    fprintf('  RI: %d  |  Cap: %.2f bits/s/Hz\n', ri_B, cap_B);
    fprintf('  %-4s  %-14s  %-12s  %-20s\n', 'SV #','Singular val','SNR (dB)','Cap (bits/s/Hz)');
    for k = 1:length(singularValues_B)
        fprintf('  %-4d  %-14.3f  %-12.1f  %-20.2f\n', k, singularValues_B(k), ...
            10*log10(snr_sv_B(k)), log2(1+snr_sv_B(k)));
    end
    fprintf('  Per-layer SINR: [%s] dB\n', num2str(sinr_B_dB.', '%.1f '));
    fprintf('  Per-layer CQI:  [%s]\n',    num2str(cqi_B.',     '%d '));
end

% =========================================================================
%  APPROACH C: Rel-19 Type I Single-Panel Mode A
% =========================================================================
if ismember(selectedApproach, {'C','D','ALL'})
    % Build shared repCfg for Approach C and D
    repCfg                   = nrCSIReportConfig;
    repCfg.NSizeBWP          = carrier.NSizeGrid;
    repCfg.NStartBWP         = 0;
    repCfg.CodebookType      = 'typeI-SinglePanel-r19';
    repCfg.PanelDimensions   = [1, 16, 4];   % Ng=1, N1=16, N2=4 -> 128 ports
    repCfg.PMIFormatIndicator = 'wideband';
end

if ismember(selectedApproach, {'C','ALL'})
    fprintf('\n--- Approach C: Rel-19 Mode A (TS 38.214 S5.2.2.2.1a) ---\n');
    repCfg.CodebookMode = 1;
    ri_C = 1;  best_rate_C = -Inf;
    fprintf('  RI selection (capacity per rank candidate):\n');
    for ri_try = 1:ri_max
        try
            [~, info_try] = nr5g.internal.nrPMIReport( ...
                carrier, csirs{1}, repCfg, ri_try, H_4d, nVar_wb);
            W_try = info_try.W;
            % Capacity with total power=1: ||W||_F=1 -> (1/nVar)*H*W*W'*H'
            rate_ = real(log2(det(eye(nRxAntennas) + ...
                (1/nVar_wb) * (H_wb*W_try*(H_wb*W_try)'))));
            fprintf('    RI=%d -> %.2f bits/s/Hz\n', ri_try, rate_);
            if rate_ > best_rate_C;  best_rate_C = rate_;  ri_C = ri_try;  end
        catch ME;  fprintf('    [RI=%d failed: %s]\n', ri_try, ME.message);  end
    end
    [pmiSet_C, info_C] = nr5g.internal.nrPMIReport( ...
        carrier, csirs{1}, repCfg, ri_C, H_4d, nVar_wb);
    W_C       = info_C.W;
    cap_C     = real(log2(det(eye(nRxAntennas) + ...
        (1/nVar_wb) * (H_wb*W_C*(H_wb*W_C)'))));
    sinr_C_dB = 10*log10(mean(info_C.SINRPerREPMI, 1, 'omitnan'));
    cqi_C     = arrayfun(@(s) max(sum(s >= cqi_tbl), 1), sinr_C_dB);
    fprintf('  RI=%d  i1=%s  i2=%s\n', ri_C, mat2str(pmiSet_C.i1-1), mat2str(pmiSet_C.i2-1));
    fprintf('  Per-layer SINR: [%s] dB\n', num2str(sinr_C_dB,'%.1f '));
    fprintf('  Per-layer CQI:  [%s]\n',    num2str(cqi_C,'%d '));
end

% =========================================================================
%  APPROACH D: Rel-19 Type I Single-Panel Mode B
% =========================================================================
if ismember(selectedApproach, {'D','ALL'})
    fprintf('\n--- Approach D: Rel-19 Mode B (TS 38.214 S5.2.2.2.1a) ---\n');
    repCfg.CodebookMode = 2;
    ri_D = 1;  best_rate_D = -Inf;
    fprintf('  RI selection (capacity per rank candidate):\n');
    for ri_try = 1:ri_max
        try
            [~, info_try] = nr5g.internal.nrPMIReport( ...
                carrier, csirs{1}, repCfg, ri_try, H_4d, nVar_wb);
            W_try = info_try.W;
            % Capacity with total power=1: ||W||_F=1 -> (1/nVar)*H*W*W'*H'
            rate_ = real(log2(det(eye(nRxAntennas) + ...
                (1/nVar_wb) * (H_wb*W_try*(H_wb*W_try)'))));
            fprintf('    RI=%d -> %.2f bits/s/Hz\n', ri_try, rate_);
            if rate_ > best_rate_D;  best_rate_D = rate_;  ri_D = ri_try;  end
        catch ME;  fprintf('    [RI=%d failed: %s]\n', ri_try, ME.message);  end
    end
    [pmiSet_D, info_D] = nr5g.internal.nrPMIReport( ...
        carrier, csirs{1}, repCfg, ri_D, H_4d, nVar_wb);
    W_D       = info_D.W;
    cap_D     = real(log2(det(eye(nRxAntennas) + ...
        (1/nVar_wb) * (H_wb*W_D*(H_wb*W_D)'))));
    sinr_D_dB = 10*log10(mean(info_D.SINRPerREPMI, 1, 'omitnan'));
    cqi_D     = arrayfun(@(s) max(sum(s >= cqi_tbl), 1), sinr_D_dB);
    fprintf('  RI=%d  i1=%s  i2=%s\n', ri_D, mat2str(pmiSet_D.i1-1), mat2str(pmiSet_D.i2-1));
    fprintf('  Per-layer SINR: [%s] dB\n', num2str(sinr_D_dB,'%.1f '));
    fprintf('  Per-layer CQI:  [%s]\n',    num2str(cqi_D,'%d '));
end

% =========================================================================
%  COMPARISON TABLE (ALL mode)
% =========================================================================
if strcmp(selectedApproach, 'ALL')
    fprintf('\n  --- Capacity Comparison (bits/s/Hz) ---\n');
    fprintf('  %-40s  %-4s  %-16s  %s\n', 'Approach', 'RI', 'Cap (bits/s/Hz)', '% vs B');
    fprintf('  %-40s  %-4d  %-16.2f  %.0f%%\n', 'B: SVD upper bound (reference)', ri_B, cap_B, 100.0);
    fprintf('  %-40s  %-4d  %-16.2f  %.0f%%\n', 'A: Rel-15/16 best sub-array',    ri_A, cap_A, cap_A/cap_B*100);
    fprintf('  %-40s  %-4d  %-16.2f  %.0f%%\n', 'C: Rel-19 Mode A',               ri_C, cap_C, cap_C/cap_B*100);
    fprintf('  %-40s  %-4d  %-16.2f  %.0f%%\n', 'D: Rel-19 Mode B',               ri_D, cap_D, cap_D/cap_B*100);
    fprintf('\n');
end

%% ========================================================================
%  SECTION 11: BEAMFORMING PATTERN VISUALIZATION
%  ========================================================================

nV   = gnbArraySize(1);  nH = gnbArraySize(2);  nPol = gnbArraySize(3);
azAngles = -90:0.5:90;   d  = 0.5;

% --- Approach C: Rel-19 Mode A codebook beam ---
if ismember(selectedApproach, {'C','ALL'})
    figure('Name', 'Beamforming Analysis (Approach C - Rel-19 Mode A)', ...
        'Position', [100 100 1200 500]);
    i11_C = pmiSet_C.i1(1);  i12_C = pmiSet_C.i1(2);
    O1_viz = 4; O2_viz = 4;  N1_viz = 16; N2_viz = 4;
    um = exp(2*pi*1i*i12_C*(0:N2_viz-1)/(O2_viz*N2_viz));
    ul = exp(2*pi*1i*i11_C*(0:N1_viz-1)/(O1_viz*N1_viz)).';
    vlm = reshape((ul.*um).',[],1);
    w1_C = [vlm; exp(1i*pi*(pmiSet_C.i2-1)/2)*vlm] / sqrt(2*nTxAntennas);
    bp_C = zeros(length(azAngles),1);
    for ai = 1:length(azAngles)
        az     = azAngles(ai)*pi/180;
        a_full = kron(ones(nPol,1), kron(ones(nV,1), exp(1j*2*pi*d*(0:nH-1).'*sin(az))));
        bp_C(ai) = abs(a_full'*w1_C)^2;
    end
    subplot(1,2,1);
    plot(azAngles, 10*log10(bp_C/max(bp_C)), 'LineWidth', 1.5);
    xlabel('Azimuth (degrees)'); ylabel('Normalized Gain (dB)');
    title(sprintf('Approach C Beam (i11=%d, i12=%d, i2=%d)', i11_C, i12_C, pmiSet_C.i2-1));
    grid on; ylim([-30 0]);
    subplot(1,2,2);
    text(0.5,0.5,'SVD spectrum: run Approach B','HorizontalAlignment','center');
    axis off;
end

% --- Approach B: SVD beam + singular value spectrum ---
if ismember(selectedApproach, {'B','ALL'})
    figure('Name', 'Beamforming Analysis (Approach B - SVD)', ...
        'Position', [100 100 1200 500]);
    subplot(1,2,1);
    if ri_B > 0
        w1_B   = W_B(:,1);
        bp_B   = zeros(length(azAngles),1);
        for ai = 1:length(azAngles)
            az     = azAngles(ai)*pi/180;
            a_full = kron(ones(nPol,1), kron(ones(nV,1), exp(1j*2*pi*d*(0:nH-1).'*sin(az))));
            bp_B(ai) = abs(a_full'*w1_B)^2;
        end
        plot(azAngles, 10*log10(bp_B/max(bp_B)), 'LineWidth', 1.5);
        xlabel('Azimuth (degrees)'); ylabel('Normalized Gain (dB)');
        title(sprintf('Approach B Beam (Layer 1) — %dH x %dV x %dpol', nH, nV, nPol));
        grid on; ylim([-30 0]);
    else
        text(0.5, 0.5, 'RI=0: no valid beam', 'HorizontalAlignment', 'center');
        axis off;
    end
    subplot(1,2,2);
    bar(singularValues_B(1:min(20,end)));
    xlabel('Singular Value Index'); ylabel('Magnitude');
    title(sprintf('Channel Singular Values (RI=%d)', ri_B));
    grid on;
end

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
fprintf(' Channel:     %s, DS=%.0fns, fD=%.0fHz\n', channel.DelayProfile, ...
    channel.DelaySpread*1e9, channel.MaximumDopplerShift);
fprintf(' SNR:         %d dB\n', SNRdB);
fprintf(' CE NMSE:     %.2f dB\n', 10*log10(mean(nmse_per_port)));
fprintf(' Approach:    %s\n', selectedApproach);
if ismember(selectedApproach, {'A','ALL'})
    fprintf(' [A] Rel-15/16 RI: %d  |  Cap: %.2f bits/s/Hz  |  Avg CQI: %.1f\n', ri_A, cap_A, cqi_A_avg);
end
if ismember(selectedApproach, {'B','ALL'})
    fprintf(' [B] SVD       RI: %d  |  Cap: %.2f bits/s/Hz  |  CQI: [%s]\n', ...
        ri_B, cap_B, num2str(cqi_B.', '%d '));
end
if ismember(selectedApproach, {'C','ALL'})
    fprintf(' [C] Mode A    RI: %d  |  Cap: %.2f bits/s/Hz  |  CQI: [%s]  |  SINR: [%s] dB\n', ...
        ri_C, cap_C, num2str(cqi_C,'%d '), num2str(sinr_C_dB,'%.1f '));
end
if ismember(selectedApproach, {'D','ALL'})
    fprintf(' [D] Mode B    RI: %d  |  Cap: %.2f bits/s/Hz  |  CQI: [%s]  |  SINR: [%s] dB\n', ...
        ri_D, cap_D, num2str(cqi_D,'%d '), num2str(sinr_D_dB,'%.1f '));
end
fprintf('======================================================\n');
