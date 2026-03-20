%% 128T128R NZP-CSI-RS Channel Estimation & CSI Feedback
%  Project: CSI-RS experiment for Massive MIMO 128T128R
%  
%  Configuration:
%    - 128 CSI-RS ports = 4 NZP-CSI-RS Resources x 32 ports (Row 17, CDM4)
%    - Channel: CDL-C (spatial, suitable for massive MIMO)
%    - Pipeline: Signal Gen -> Channel -> CE -> CSI Feedback (PMI/RI/CQI)
%
%  References:
%    - TS 38.211 S7.4.1.5 (CSI-RS signal generation)
%    - TS 38.214 S5.2.2.3 (CSI reporting)
%    - TS 38.331 (NZP-CSI-RS-ResourceSet)
%
%  Author: ThangTQ23 - VSI
%  Date:   2026-03

clear; close all; clc;

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
%  SECTION 2: CONFIGURE 4 NZP-CSI-RS RESOURCES (Row 17, CDM4)
%  ========================================================================
%
%  Row 17: 32 ports, CDM4 (FD2,TD2), density 0.5
%    - k' = 0,1  (2 subcarriers per CDM group)
%    - l' = 0,1  (2 symbols per CDM group)  
%    - 8 CDM groups x 4 ports/group = 32 ports
%    - Each resource occupies 2 OFDM symbols
%
%  4 resources x 2 symbols = 8 symbols -> fits within 14 symbols/slot
%
%  Symbol allocation within slot:
%    Resource #0: symbols 2,3   (ports 0-31)
%    Resource #1: symbols 5,6   (ports 32-63)
%    Resource #2: symbols 8,9   (ports 64-95)
%    Resource #3: symbols 11,12 (ports 96-127)

% Symbol start positions for 4 resources (0-indexed)
symbolStarts = [2, 5, 8, 11];

% Subcarrier locations for Row 17 (from Table 7.4.1.5.3-1)
% Row 17: 4 k_i values required -> [k0, k1, k2, k3] = [0, 2, 4, 6]
% (8 CDM groups with FD-CDM2 spacing of 2 subcarriers, density 0.5)
subcarrierLoc = [0, 2, 4, 6];  % 4 k_i values for Row 17

% CDM type for Row 17
cdmType = 'CDM4';
cdmLengths = [2 2];  % [FD-CDM2, TD-CDM2]

% Create CSI-RS config for each resource
csirs = cell(1, nResources);
for resIdx = 1:nResources
    csirs{resIdx} = nrCSIRSConfig;
    csirs{resIdx}.CSIRSType       = {'nzp'};
    csirs{resIdx}.CSIRSPeriod     = 'on';       % Always on (single slot sim)
    csirs{resIdx}.RowNumber       = 17;
    csirs{resIdx}.Density         = {'dot5even'};    % density 0.5
    csirs{resIdx}.SymbolLocations = {[symbolStarts(resIdx), symbolStarts(resIdx)+1]};
    csirs{resIdx}.SubcarrierLocations = {subcarrierLoc};  % 4 k_i values: [0,2,4,6]
    csirs{resIdx}.NumRB           = carrier.NSizeGrid;
    csirs{resIdx}.NID             = carrier.NCellID;
    
    fprintf('Resource #%d: Row 17, symbols [%d,%d], %d ports\n', ...
        resIdx-1, symbolStarts(resIdx), symbolStarts(resIdx)+1, nPortsPerRes);
end

%% ========================================================================
%  SECTION 3: GENERATE CSI-RS SYMBOLS & MAP TO GRID
%  ========================================================================

% Initialize carrier grid for all 128 ports
% Each resource maps to its corresponding 32-port subset
txGrid = nrResourceGrid(carrier, nTxAntennas);

% Power scaling
powerCSIRS_dB = 0;  % dB
powerScale = db2mag(powerCSIRS_dB);

% Generate and map CSI-RS for each resource
allCsirsInd = cell(1, nResources);
allCsirsSym = cell(1, nResources);

for resIdx = 1:nResources
    % Generate symbols and indices
    sym = nrCSIRS(carrier, csirs{resIdx});
    ind = nrCSIRSIndices(carrier, csirs{resIdx});
    
    % Apply power scaling
    sym = sym * powerScale;
    
    % Port offset: resource #k maps to ports (k-1)*32+1 : k*32
    portOffset = (resIdx - 1) * nPortsPerRes;
    
    % Adjust indices to correct port positions in the full grid.
    % nrCSIRSIndices returns 1-based linear indices for a grid of size
    % [K x L x nPortsPerRes]. Need to offset to [K x L x nTxAntennas].
    gridSizePerRes = [carrier.NSizeGrid*12, carrier.SymbolsPerSlot, nPortsPerRes];
    [subInd, symInd, portInd] = ind2sub(gridSizePerRes, ind);
    
    % Shift port index
    portInd = portInd + portOffset;
    
    % Convert back to linear index in full grid
    gridSizeFull = [carrier.NSizeGrid*12, carrier.SymbolsPerSlot, nTxAntennas];
    fullInd = sub2ind(gridSizeFull, subInd, symInd, portInd);
    
    txGrid(fullInd) = sym;
    
    % Store for later use in channel estimation
    allCsirsInd{resIdx} = ind;
    allCsirsSym{resIdx} = sym;
    
    fprintf('Resource #%d: %d symbols, %d indices generated\n', ...
        resIdx-1, length(sym), length(ind));
end

%% ========================================================================
%  SECTION 4: VISUALIZE CSI-RS RESOURCE GRID
%  ========================================================================

figure('Name', 'CSI-RS Resource Allocation', 'Position', [100 100 1200 500]);

% Overview: show CSI-RS on grid (port 0 of each resource)
subplot(1,2,1);
gridView = zeros(carrier.NSizeGrid*12, carrier.SymbolsPerSlot);
colors = [1, 2, 3, 4];  % Color coding for 4 resources
for resIdx = 1:nResources
    gridSizePerRes = [carrier.NSizeGrid*12, carrier.SymbolsPerSlot, nPortsPerRes];
    [subInd, symInd, ~] = ind2sub(gridSizePerRes, allCsirsInd{resIdx});
    % Only plot port 0 of each resource
    uniquePos = unique([subInd, symInd], 'rows');
    for p = 1:size(uniquePos,1)
        gridView(uniquePos(p,1), uniquePos(p,2)) = colors(resIdx);
    end
end
imagesc(0:carrier.SymbolsPerSlot-1, 1:carrier.NSizeGrid*12, gridView);
axis xy;
colormap([1 1 1; 0.2 0.4 0.8; 0.8 0.2 0.2; 0.2 0.7 0.3; 0.9 0.6 0.1]);
xlabel('OFDM Symbol'); ylabel('Subcarrier');
title('CSI-RS Allocation (4 Resources in 1 Slot)');
legend_entries = arrayfun(@(x) sprintf('Res #%d (ports %d-%d)', ...
    x-1, (x-1)*32, x*32-1), 1:nResources, 'UniformOutput', false);

% Symbol occupancy plot
subplot(1,2,2);
symOccupancy = zeros(carrier.SymbolsPerSlot, 1);
for resIdx = 1:nResources
    s1 = symbolStarts(resIdx);
    symOccupancy(s1+1) = resIdx;       % +1 for 1-indexed
    symOccupancy(s1+2) = resIdx;       % Second symbol (TD-CDM2)
end
barh(0:carrier.SymbolsPerSlot-1, symOccupancy, 'FaceColor', 'flat');
xlabel('Resource Index'); ylabel('OFDM Symbol');
title('Symbol Occupancy per Resource');
set(gca, 'YDir', 'reverse');

%% ========================================================================
%  SECTION 5: CDL-C CHANNEL MODEL (SPATIAL)
%  ========================================================================
%
%  CDL-C: Cluster delay line model with spatial information
%  - Supports AoA/AoD -> essential for massive MIMO beamforming evaluation
%  - Antenna array geometry directly affects channel matrix

% --- gNB antenna array ---
% 128 antennas: 16H x 4V x 2 polarizations = 128
gnbArraySize = [16, 4, 2, 1, 1];  % [Nh, Nv, P, Mg, Ng] 
% Nh=16 horizontal, Nv=4 vertical, P=2 polarizations
% -> 16 x 4 x 2 = 128 elements

% --- UE antenna array ---
% 4 antennas: 2H x 1V x 2 polarizations = 4
ueArraySize = [2, 1, 2, 1, 1];

% --- CDL-C Channel ---
channel = nrCDLChannel;
channel.DelayProfile         = 'CDL-C';
channel.DelaySpread          = 100e-9;       % 100ns (Urban Macro)
channel.MaximumDopplerShift  = 5;            % 5 Hz (low mobility, indoor/pedestrian)
channel.CarrierFrequency     = 3.5e9;        % 3.5 GHz (n78)

% Antenna array configuration
channel.TransmitAntennaArray.Size    = gnbArraySize;
channel.TransmitAntennaArray.ElementSpacing = [0.5 0.5 1 1]; % in wavelengths
channel.ReceiveAntennaArray.Size     = ueArraySize;
channel.ReceiveAntennaArray.ElementSpacing  = [0.5 0.5 1 1];

% Verify antenna count
fprintf('\n--- Channel Configuration ---\n');
fprintf('Channel model: CDL-C\n');
fprintf('Delay spread: %.0f ns\n', channel.DelaySpread*1e9);
fprintf('Doppler: %.1f Hz\n', channel.MaximumDopplerShift);
fprintf('Carrier freq: %.1f GHz\n', channel.CarrierFrequency/1e9);
fprintf('gNB array: %s -> %d antennas\n', mat2str(gnbArraySize), prod(gnbArraySize));
fprintf('UE array:  %s -> %d antennas\n', mat2str(ueArraySize), prod(ueArraySize));

% Set sample rate
ofdmInfo = nrOFDMInfo(carrier);
channel.SampleRate = ofdmInfo.SampleRate;

%% ========================================================================
%  SECTION 6: TRANSMIT THROUGH CHANNEL + AWGN
%  ========================================================================

% OFDM modulation
[txWaveform, ofdmModInfo] = nrOFDMModulate(carrier, txGrid);

% Zero-pad for channel delay
chInfo = info(channel);
maxChDelay = chInfo.MaximumChannelDelay;
txWaveform = [txWaveform; zeros(maxChDelay, size(txWaveform, 2))];

% Pass through channel
[rxWaveform, pathGains, sampleTimes] = channel(txWaveform);

% Perfect channel estimation (ground truth)
pathFilters = getPathFilters(channel);
H_actual = nrPerfectChannelEstimate(carrier, pathGains, pathFilters);

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

%% ========================================================================
%  SECTION 7: TIMING SYNC + OFDM DEMOD
%  ========================================================================

% Timing estimation using Resource #0 as reference
refSym = allCsirsSym{1};
refInd = allCsirsInd{1};
offset = nrTimingEstimate(carrier, rxWaveform, refInd, refSym);
fprintf('Timing offset: %d samples\n', offset);

rxWaveform = rxWaveform(1+offset:end, :);

% OFDM demodulation
rxGrid = nrOFDMDemodulate(carrier, rxWaveform);
fprintf('rxGrid size: [%s]\n', num2str(size(rxGrid)));

%% ========================================================================
%  SECTION 8: CHANNEL ESTIMATION (PER RESOURCE, THEN COMBINE)
%  ========================================================================
%
%  Each resource yields H_est of size [K x L x nRx x 32].
%  Concatenate 4 resources -> H_est_full of size [K x L x nRx x 128].

nSubcarriers = carrier.NSizeGrid * 12;
nSymbols = carrier.SymbolsPerSlot;

H_est_full = zeros(nSubcarriers, nSymbols, nRxAntennas, nTxAntennas);
nVar_all = zeros(1, nResources);

for resIdx = 1:nResources
    % Channel estimation for this resource
    [H_est_res, nVar_res] = nrChannelEstimate(carrier, rxGrid, ...
        allCsirsInd{resIdx}, allCsirsSym{resIdx}, ...
        'CDMLengths', cdmLengths);
    
    % Port range for this resource
    portStart = (resIdx - 1) * nPortsPerRes + 1;
    portEnd   = resIdx * nPortsPerRes;
    
    % Insert into combined channel matrix
    % H_est_res: [K x L x nRx x 32]
    H_est_full(:, :, :, portStart:portEnd) = H_est_res;
    nVar_all(resIdx) = nVar_res;
    
    fprintf('Resource #%d: H_est size [%s], nVar = %.4e\n', ...
        resIdx-1, num2str(size(H_est_res)), nVar_res);
end

nVar_avg = mean(nVar_all);
fprintf('\nCombined H_est_full size: [%s]\n', num2str(size(H_est_full)));
fprintf('Average noise variance: %.4e\n', nVar_avg);

%% ========================================================================
%  SECTION 9: CHANNEL ESTIMATION ERROR ANALYSIS
%  ========================================================================

% Compare H_est_full vs H_actual
H_actual_trimmed = H_actual(:, :, :, 1:size(H_est_full, 4));
H_err = H_est_full - H_actual_trimmed;

% MSE per port
mse_per_port = squeeze(mean(abs(H_err).^2, [1 2 3]));
nmse_per_port = mse_per_port ./ squeeze(mean(abs(H_actual_trimmed).^2, [1 2 3]));

fprintf('\n--- Channel Estimation Error ---\n');
fprintf('Overall MSE:  %.4e\n', mean(mse_per_port));
fprintf('Overall NMSE: %.2f dB\n', 10*log10(mean(nmse_per_port)));

% Plot
figure('Name', 'Channel Estimation Quality', 'Position', [100 100 1400 400]);

subplot(1,3,1);
plot(0:nTxAntennas-1, 10*log10(nmse_per_port), '-o', 'MarkerSize', 3);
xlabel('CSI-RS Port Index'); ylabel('NMSE (dB)');
title('NMSE per Port');
grid on;
xline([32 64 96]-0.5, '--r', {'Res#1','Res#2','Res#3'});

subplot(1,3,2);
imagesc(abs(H_est_full(:,:,1,1)));
axis xy; colorbar;
xlabel('OFDM Symbol'); ylabel('Subcarrier');
title('|H_{est}| (Rx0, Port0)');

subplot(1,3,3);
imagesc(abs(H_actual_trimmed(:,:,1,1)));
axis xy; colorbar;
xlabel('OFDM Symbol'); ylabel('Subcarrier');
title('|H_{actual}| (Rx0, Port0)');

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

% --- Approach A: Per-resource PMI (using MATLAB built-in) ---
fprintf('\n--- Approach A: Per-Resource PMI (32 ports each) ---\n');

reportConfig = nrCSIReportConfig;
reportConfig.PanelDimensions   = [1 8 2];    % [Ng=1, N1=8, N2=2]: 1*8*2*2pol = 32 ports
reportConfig.CodebookType      = 'type1SinglePanel';
reportConfig.CodebookMode      = 1;
reportConfig.CQITable           = 'table1';
reportConfig.SubbandSize        = 4;

% DM-RS config required by nrCSIReportCSIRS (R2025b API)
dmrsConfig = nrPDSCHDMRSConfig;

for resIdx = 1:nResources
    portStart = (resIdx - 1) * nPortsPerRes + 1;
    portEnd   = resIdx * nPortsPerRes;

    H_res = H_est_full(:, :, :, portStart:portEnd);

    % Use local wrapper (bypasses nrCSIReportCSIRS feature gate)
    [csiReport, csiInfo] = myCSIReport(carrier, csirs{resIdx}, ...
        reportConfig, dmrsConfig, H_res, nVar_avg);

    pmiSet = csiReport.PMISet;
    if isstruct(pmiSet) && ~isempty(pmiSet) && isfield(pmiSet(1), 'i1')
        i1_str = num2str(pmiSet(1).i1);
        i2_val = pmiSet(1).i2;
    else
        i1_str = 'N/A'; i2_val = 0;
    end
    fprintf('Resource #%d (ports %3d-%3d): RI=%d, CQI=%d, PMI i1=[%s] i2=%d\n', ...
        resIdx-1, portStart-1, portEnd-1, csiReport.RI, csiReport.CQI(1), ...
        i1_str, i2_val);
end

% --- Approach B: Full 128-port SVD-based precoder ---
fprintf('\n--- Approach B: Full 128-Port SVD Precoder ---\n');

% Wideband channel matrix: average over all subcarriers and symbols.
% A single-RE approach risks landing on a zero (unoccupied pilot RE),
% so the mean over the full slot is more robust.
H_wb = squeeze(mean(mean(H_est_full, 1), 2));  % [nRx x nTx]
fprintf('H_wb size: [%s]\n', num2str(size(H_wb)));

% SVD decomposition
[U, S, V] = svd(H_wb, 'econ');
singularValues = diag(S);

% RI = number of singular values above threshold
riThreshold = 0.1 * singularValues(1);  % 10% of largest
ri_svd = sum(singularValues > riThreshold);
ri_svd = min(ri_svd, nRxAntennas);  % Cannot exceed nRx

fprintf('Singular values (top 8): [%s]\n', ...
    num2str(singularValues(1:min(8,end)).', '%.2f '));
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
    signal = abs(W_zf(layer,:) * H_eff(:,layer))^2;
    interference = sum(abs(W_zf(layer,:) * H_eff).^2) - signal;
    noise_term = nVar_avg * norm(W_zf(layer,:))^2;
    sinr_per_layer(layer) = signal / (interference + noise_term);
end
sinr_dB = 10*log10(sinr_per_layer);
fprintf('Per-layer SINR (dB): [%s]\n', num2str(sinr_dB.', '%.1f '));

% CQI mapping (simplified, TS 38.214 Table 5.2.2.1-2)
cqi_table = [  % Approximate SINR thresholds for CQI 1-15
    -6.7, -4.7, -2.3, 0.2, 2.4, 4.7, 6.9, 9.3, ...
    10.7, 12.2, 14.1, 15.6, 18.0, 20.3, 22.7];
cqi_svd = zeros(ri_svd, 1);
for layer = 1:ri_svd
    cqi_svd(layer) = sum(sinr_dB(layer) >= cqi_table);
    cqi_svd(layer) = max(cqi_svd(layer), 1);  % Minimum CQI = 1
end
fprintf('Per-layer CQI: [%s]\n', num2str(cqi_svd.', '%d '));

%% ========================================================================
%  SECTION 11: BEAMFORMING PATTERN VISUALIZATION
%  ========================================================================

% Visualize beam pattern from SVD precoder (first layer)
figure('Name', 'Beamforming Analysis', 'Position', [100 100 1200 500]);

if ri_svd > 0
    % Beam pattern (azimuth cut)
    subplot(1,2,1);
    w1 = W_svd(:,1);  % First layer precoder [128 x 1]

    % Steering vector for ULA-like response
    azAngles = -90:0.5:90;
    nH   = gnbArraySize(1);  % 16 horizontal elements
    nV   = gnbArraySize(2);  % 4 vertical elements
    nPol = gnbArraySize(3);  % 2 polarizations

    beamPattern = zeros(length(azAngles), 1);
    d = 0.5;  % Element spacing in wavelengths
    for ai = 1:length(azAngles)
        az = azAngles(ai) * pi / 180;
        a_h    = exp(1j * 2 * pi * d * (0:nH-1).' * sin(az));
        a_full = kron(ones(nPol,1), kron(ones(nV,1), a_h));
        beamPattern(ai) = abs(a_full' * w1)^2;
    end
    beamPattern_dB = 10*log10(beamPattern / max(beamPattern));

    plot(azAngles, beamPattern_dB, 'LineWidth', 1.5);
    xlabel('Azimuth (degrees)'); ylabel('Normalized Gain (dB)');
    title('Beam Pattern (Azimuth Cut, Layer 1)');
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
fprintf(' CSI-RS:      %d x Row17(32p, CDM4)\n', nResources);
fprintf(' Channel:     CDL-C, DS=%.0fns, fD=%.0fHz\n', channel.DelaySpread*1e9, channel.MaximumDopplerShift);
fprintf(' SNR:         %d dB\n', SNRdB);
fprintf(' CE NMSE:     %.2f dB\n', 10*log10(mean(nmse_per_port)));
fprintf(' SVD RI:      %d layers\n', ri_svd);
fprintf(' SVD CQI:     [%s]\n', num2str(cqi_svd.', '%d '));
fprintf(' Layer SINR:  [%s] dB\n', num2str(sinr_dB.', '%.1f '));
fprintf('======================================================\n');