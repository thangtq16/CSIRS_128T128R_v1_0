%% NR Cell — 128T128R DL MU-MIMO + DRL Socket (Rel-19 eTypeII, Subband CQI)
% System-level MU-MIMO simulation with nrDRLScheduler connected to a Python
% DRL training server via TCP socket.
%
% Base: CSIRS_128T128R_v3_0_MUMIMO.m  (128T128R, eTypeII-r19, subband PMI+CQI)
% Added: socket API, FTP-3 traffic, allocation-map export (from CSIRS_32T32R_SOCKET.m)
%
% Observation space delivered to Python per TTI (via TTI_START JSON):
%   1. avg_tp       [MaxUEs]            – EMA actual DL throughput (Mbps)
%   2. ue_rank      [MaxUEs]            – RI per UE
%   3. occupied_rbgs / allocMatrix      – per-layer RBG allocation (tracked by Python)
%   4. buf          [MaxUEs]            – DL buffer status (bytes)
%   5. curr_mcs     [MaxUEs]            – effective MCS after OLLA
%   6. ue_i1        [MaxUEs x 2]        – eTypeII beam-group index i1
%   7. sub_cqi      [MaxUEs x numRBGs]  – subband CQI (resampled to RBG grid)
%   8. max_cross_corr [MaxUEs x MaxUEs x numRBGs] – max-column kappa, NOT MATLAB built-in
%
% Author: ThangTQ23 — VSI, 2026-04

%% ====================== 1. PATH SETUP & LOGGING ======================
% Prepend patched wirelessnetwork + 5g libraries so that:
%   • nrGNB supports NumTransmitAntennas = 128
%   • eTypeII-r19 codebook path is activated in nrNodeValidation / nrUEAbstractPHY
clear classes; rehash; close all;

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projectRoot, 'wirelessnetwork')), '-begin');
addpath(genpath(fullfile(projectRoot, '5g')),              '-begin');

wirelessnetworkSupportPackageCheck

% Diary log (timestamped, written to logs_v4/)
logDir  = fullfile(projectRoot, 'logs_v4');
if exist(logDir, 'dir'), rmdir(logDir, 's'); end
mkdir(logDir);
logFile = fullfile(logDir, 'matlab_log_' + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + '.txt');
diary(logFile);

%% ====================== 2. SIMULATION PARAMETERS ======================

% ── PMI/CQI reporting mode ───────────────────────────────────────────────
% 'Subband': per-subband precoder (PMI) + per-subband CQI.
%   With NRB=24 @ 30 kHz → subband size = 4 RBs → numSubbands = 6
%   (TS 38.214 Table 5.2.1.4-2, NRB ∈ [24, 72])
% 'Wideband': single precoder + single CQI — faster, less accurate.
pmiCQIMode = "Subband";   % change to "Wideband" for faster debug runs

rng("default");
numFrameSimulation = 5;
networkSimulator   = wirelessNetworkSimulator.init;

%% ====================== 3. gNB CONFIGURATION (128T128R) ======================
% 128T128R, TDD, n79 4.9 GHz, 10 MHz / 30 kHz SCS.
% Use channelBW = 100e6 / numRB = 273 for production — 10 MHz here for speed.

gNBPosition  = [0 0 30];      % [x y z] metres
duplexMode   = "TDD";

carrierFreq  = 4.9e9;         % n79 centre frequency (Hz)
channelBW    = 10e6;          % channel bandwidth — change to 100e6 for production
scs          = 30e3;          % subcarrier spacing (Hz)
% TS 38.104 Table 5.3.2-1 (FR1, 30 kHz SCS):
%   5→11  10→24  15→38  20→51  40→106  100→273
numRB = 24;                   % matches 10 MHz @ 30 kHz SCS

gNB = nrGNB( ...
    Position             = gNBPosition, ...
    TransmitPower        = 40, ...
    CarrierFrequency     = carrierFreq, ...
    ChannelBandwidth     = channelBW, ...
    SubcarrierSpacing    = scs, ...
    NumResourceBlocks    = numRB, ...
    NumTransmitAntennas  = 128, ...
    NumReceiveAntennas   = 128, ...
    DuplexMode           = duplexMode, ...
    ReceiveGain          = 11, ...
    SRSPeriodicityUE     = 20);

fprintf('gNB: 128T128R | %.1f GHz (n79) | %s | BW=%.0f MHz | NRB=%d\n', ...
    carrierFreq/1e9, duplexMode, channelBW/1e6, numRB);

% ── PMI/CQI mode must be set BEFORE connectUE ────────────────────────────
% nrNodeValidation picks this up during connectUE to configure the UE's
% abstract-PHY CSI report (subband vs wideband CQI vector in csi.CQI).
gNB.PMICQIMode = pmiCQIMode;

%% ====================== 4. DRL SCHEDULER CONFIGURATION ======================

numUEs = 16;   % total UEs in this simulation

% ── MU-MIMO pairing constraints ──────────────────────────────────────────
% eTypeII DOF constraint: L=2 beams × 2 polarisations = 4D subspace.
% K×r ≤ 4 → with rank=2 and K=2 UEs: 4 streams in 4D → orthogonal planes.
muMIMOConfig = struct( ...
    MaxNumUsersPaired    = 4, ...   % ≤ 2 recommended for rank-2 UEs
    MaxNumLayers         = 4, ...
    MinNumRBs            = 2, ...
    SemiOrthogonalityFactor = 0.7, ...
    MinCQI               = 1);

% ── OLLA ─────────────────────────────────────────────────────────────────
% TargetBLER = StepDown / (StepDown + StepUp) = 0.03/0.30 = 10%
% MCSOffset accumulates via HARQ ACK/NACK (no reset — State 3).
% SU-measured CQI overestimates MU-MIMO SINR → OLLA must converge first.
ollaConfig = struct('InitialOffset', 0, 'StepUp', 0.27, 'StepDown', 0.03);

% ── nrDRLScheduler ───────────────────────────────────────────────────────
drlScheduler = nrDRLScheduler();
drlScheduler.EnableDRL           = true;
drlScheduler.TrainingMode        = true;   % TTI_START / LAYER_ACT / TTI_DONE protocol
drlScheduler.NumLayers           = muMIMOConfig.MaxNumLayers;
drlScheduler.MaxUsersPerRBG      = muMIMOConfig.MaxNumUsersPaired;
drlScheduler.MaxUEs              = numUEs;
% SubbandSize: 3GPP subband size for CQI (informational — scheduler uses numRBGs internally)
% NRB=24 → subband size = 4 RBs (TS 38.214 Table 5.2.1.4-2)
drlScheduler.SubbandSize         = 4;
drlScheduler.DRLDebug            = false;

% Pairing constraints: DRL learns its own pairing policy — disable MATLAB filter.
% Set true to let MATLAB enforce i1-match + precoder orthogonality as a hard gate.
drlScheduler.EnablePairingConstraints = false;
drlScheduler.SemiOrthogonalityFactor  = muMIMOConfig.SemiOrthogonalityFactor;
drlScheduler.MU_MCSBackoff            = 0;

% Throughput EMA normalisation (Mbps) — scales fR feature to [0,1]
drlScheduler.MaxTput = 100.0;

% ── Connect to Python DRL server ─────────────────────────────────────────
ok = drlScheduler.connectToDRLAgent('127.0.0.1', 6666);
assert(ok, '[v4] Cannot connect to DRL server. Start Python train script first.');

% ── configureScheduler ───────────────────────────────────────────────────
configureScheduler(gNB, ...
    'Scheduler',               drlScheduler, ...
    'ResourceAllocationType',  0, ...          % RAT-0 (RBG bitmap)
    'MaxNumUsersPerTTI',       numUEs, ...
    'PFSWindowSize',           20, ...         % EMA window for PF throughput tracking
    'MUMIMOConfigDL',          muMIMOConfig, ...
    'LinkAdaptationConfigDL',  ollaConfig, ...
    'CSIMeasurementSignalDL',  "CSI-RS", ...
    'RVSequence',              0);             % RV=0 only (no HARQ retransmissions)
% NOTE: SchedulerStrategy=1 omitted — it triggers nrUEContext.updateUEsServedDataRate
% in MU-MIMO mode → crash "Matrix dimensions must agree".  EMA tracking is handled
% entirely inside nrDRLScheduler.ActualTputEMA.

%% ====================== 5. UE DEPLOYMENT ======================
% 16 UEs in a 120° sector, r ∈ [50, 500] m (uniform on area via sqrt-trick).
% 4Tx/4Rx per UE: needed for rank > 1 with eTypeII precoding.

ueNumTx = 2;
ueNumRx = 4;

rMin = 50;  rMax = 500;
rng(42);
azDeg = -60 + 120 .* rand(numUEs, 1);
rDist = sqrt(rMin^2 + (rMax^2 - rMin^2) .* rand(numUEs, 1));
[xPos, yPos, zPos] = sph2cart(deg2rad(azDeg), zeros(numUEs,1), rDist);
uePositions = [xPos, yPos, zPos] + gNBPosition;

noiseFigMin = 7;  noiseFigMax = 9;
UEs = nrUE.empty(0, numUEs);
for i = 1:numUEs
    nf = noiseFigMin + (noiseFigMax - noiseFigMin) * rand();
    UEs(i) = nrUE( ...
        Name                = "UE-" + i, ...
        Position            = uePositions(i,:), ...
        NumTransmitAntennas = ueNumTx, ...
        NumReceiveAntennas  = ueNumRx, ...
        ReceiveGain         = 0, ...
        NoiseFigure         = nf);
end
fprintf('UE distances (m): %s\n', mat2str(round(rDist), 1));

% ── Traffic: FTP Model 3 (always-on 6 Mbps / 1.5 kB packets) ────────────
% Full-buffer DL is unrealistic for DRL reward — FTP3 gives finite buffer
% so buffer feature (obs #4) has meaningful variation.
connectUE(gNB, UEs, FullBufferTraffic = "off", CSIReportPeriodicity = 10);
for i = 1:numUEs
    src             = networkTrafficOnOff;
    src.PacketSize  = 1500;   % bytes
    src.DataRate    = 6e6;    % bps (= 1500 × 8 × 500 pkt/s)
    src.OnTime      = 1e9;    % always ON
    src.OffTime     = 0;
    addTrafficSource(gNB, src, 'DestinationNode', UEs(i));
end

% Give UE handles to DRL scheduler so it can read MAC.ReceivedBytes per TTI
drlScheduler.UEList = UEs;

%% ====================== 6. NETWORK SIMULATOR NODES ======================

addNodes(networkSimulator, gNB);
addNodes(networkSimulator, UEs);

%% ====================== 7. CHANNEL MODEL (CDL-D) ======================
% CDL-D: strong LoS, DS=100 ns, Doppler=5 Hz — rank-friendly for eTypeII.

channelConfig = struct( ...
    DelayProfile        = "CDL-D", ...
    DelaySpread         = 100e-9, ...
    MaximumDopplerShift = 5);

channels          = hNRCreateCDLChannels(channelConfig, gNB, UEs);
customChannelModel = hNRCustomChannelModel(channels);
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);

%% ====================== 8. LOGGING & VISUALISATION ======================

enableTraces = true;
if enableTraces
    simSchedulingLogger = helperNRSchedulingLogger(numFrameSimulation, gNB, UEs);
    simPhyLogger        = helperNRPhyLogger(numFrameSimulation, gNB, UEs);
end

metricsVisualizer = helperNRMetricsVisualizer(gNB, UEs, ...
    'RefreshRate',         1000, ...
    'PlotSchedulerMetrics', true, ...
    'PlotPhyMetrics',      false, ...
    'PlotCDFMetrics',      true, ...
    'LinkDirection',       0);

%% ====================== 9. SIMULATION CONDITIONS SUMMARY ======================

slotsPerSubfm = scs / 15e3;
slotDurMs     = 1 / slotsPerSubfm;
slotsPerFrm   = 10 * slotsPerSubfm;
csiPeriodicity = 10;
numCSIEvents  = floor(numFrameSimulation * slotsPerFrm / csiPeriodicity);

if strcmpi(pmiCQIMode, 'Subband')
    if numRB >= 145; sbSize = 16; elseif numRB >= 73; sbSize = 8; else; sbSize = 4; end
    numSubbands = ceil(numRB / sbSize);
    modeStr = sprintf('Subband | %d subbands × %d RBs', numSubbands, sbSize);
else
    numSubbands = 1; sbSize = numRB;
    modeStr = 'Wideband';
end

sep = repmat('═', 1, 72);
fprintf('\n%s\n', sep);
fprintf('  SIMULATION CONDITIONS — 128T128R MU-MIMO DL + DRL Socket (v4.0)\n');
fprintf('%s\n', sep);
fprintf('  gNB      : 128T/128R | %.1f GHz (n79) | %s | BW=%.0f MHz | NRB=%d\n', ...
    carrierFreq/1e9, duplexMode, channelBW/1e6, numRB);
fprintf('             SCS=%.0f kHz | Tx=60 dBm | Rx Gain=11 dB\n', scs/1e3);
fprintf('  UEs      : %d | %dTx/%dRx | r∈[%d,%d] m | NF∈[%d,%d] dB\n', ...
    numUEs, ueNumTx, ueNumRx, rMin, rMax, noiseFigMin, noiseFigMax);
fprintf('  Channel  : %s | DS=%.0f ns | fD=%.0f Hz\n', ...
    channelConfig.DelayProfile, channelConfig.DelaySpread*1e9, channelConfig.MaximumDopplerShift);
fprintf('  Scheduler: CSI-RS | MU-MIMO ≤%d paired / ≤%d layers | RAT-0\n', ...
    muMIMOConfig.MaxNumUsersPaired, muMIMOConfig.MaxNumLayers);
fprintf('             OLLA: init=%g up=%.2f down=%.2f → target BLER~%.0f%%\n', ...
    ollaConfig.InitialOffset, ollaConfig.StepUp, ollaConfig.StepDown, ...
    100*ollaConfig.StepDown/(ollaConfig.StepDown+ollaConfig.StepUp));
fprintf('  CSI      : eTypeII-r19 | Panel [1×16×4] | L=2 beams | %s\n', modeStr);
fprintf('             Period=%d slots (%.1f ms) → ~%d reports/UE in %d frames\n', ...
    csiPeriodicity, csiPeriodicity*slotDurMs, numCSIEvents, numFrameSimulation);
fprintf('  DRL      : Socket 127.0.0.1:6666 | TrainingMode=ON | Layers=%d\n', ...
    drlScheduler.NumLayers);
fprintf('  Traffic  : FTP Model 3 | 1500 B pkts | 6 Mbps/UE\n');
fprintf('  Sim      : %d frames | %.0f ms\n', numFrameSimulation, numFrameSimulation*10);
fprintf('%s\n\n', sep);

%% ====================== 10. RUN SIMULATION ======================

simulationTime = numFrameSimulation * 1e-2;   % seconds
fprintf('Running simulation (%.0f ms)...\n', simulationTime * 1e3);
run(networkSimulator, simulationTime);

% Notify Python that episode is complete
drlScheduler.drlSendJSON(struct('type', 'STOP'));
fprintf('Sent STOP to Python DRL server.\n');

%% ====================== 11. KPI SUMMARY & LOGS ======================

displayPerformanceIndicators(metricsVisualizer);

if enableTraces
    % ── Collect logs ─────────────────────────────────────────────────────
    if gNB.DuplexMode == "FDD"
        logInfo = struct('DLTimeStepLogs',[], 'ULTimeStepLogs',[], ...
                         'SchedulingAssignmentLogs',[], 'PhyReceptionLogs',[]);
        [logInfo.DLTimeStepLogs, logInfo.ULTimeStepLogs] = getSchedulingLogs(simSchedulingLogger);
    else
        logInfo = struct('TimeStepLogs',[], 'SchedulingAssignmentLogs',[], 'PhyReceptionLogs',[]);
        logInfo.TimeStepLogs = getSchedulingLogs(simSchedulingLogger);
    end
    logInfo.SchedulingAssignmentLogs = getGrantLogs(simSchedulingLogger);
    logInfo.PhyReceptionLogs         = getReceptionLogs(simPhyLogger);
    save(fullfile(logDir, 'simulationLogs.mat'), 'logInfo');
    save('logInfo.mat', 'logInfo');

    % ── Per-UE Grant/MCS Summary ─────────────────────────────────────────
    colMap   = simSchedulingLogger.GrantLogsColumnIndexMap;
    grantRaw = logInfo.SchedulingAssignmentLogs(2:end, :);
    rntiCol  = colMap('RNTI');
    mcsCol   = colMap('MCS Index');
    lyrCol   = colMap('NumLayers');
    typeCol  = colMap('Grant Type');
    isDL     = strcmp(grantRaw(:, typeCol), 'DL');
    dlData   = grantRaw(isDL, :);

    sep2 = repmat('─', 1, 72);
    fprintf('\n%s\n', sep2);
    fprintf('  gNB DL Grant Summary  (%d grants | %d frames | %s)\n', ...
        sum(isDL), numFrameSimulation, pmiCQIMode);
    fprintf('%s\n', sep2);
    fprintf('  %-6s  %7s  %8s  %8s  %8s  %8s\n', ...
        'UE', 'Grants', 'MCS avg', 'MCS min', 'MCS max', 'Lyr avg');
    fprintf('%s\n', sep2);
    for ueIdx = 1:numUEs
        mask = cell2mat(dlData(:, rntiCol)) == ueIdx;
        if ~any(mask), continue; end
        mcsV = cell2mat(dlData(mask, mcsCol));
        lyrV = cell2mat(dlData(mask, lyrCol));
        fprintf('  UE-%-3d  %7d  %8.1f  %8d  %8d  %8.1f\n', ...
            ueIdx, sum(mask), mean(mcsV), min(mcsV), max(mcsV), mean(lyrV));
    end
    fprintf('%s\n\n', sep2);

    % ── Export allocation maps ────────────────────────────────────────────
    allocMapDir = fullfile(logDir, 'allocation_maps');
    ttiBySlot = [];
    if isprop(drlScheduler, 'TrainingTTISlotIndex') && ~isempty(drlScheduler.TrainingTTISlotIndex)
        maxSlot   = max(drlScheduler.TrainingTTISlotIndex);
        ttiBySlot = -1 * ones(maxSlot + 1, 1);
        for tIdx = 1:numel(drlScheduler.TrainingTTISlotIndex)
            sl = drlScheduler.TrainingTTISlotIndex(tIdx);
            if sl >= 0, ttiBySlot(sl + 1) = tIdx - 1; end
        end
    end
    numMaps = exportDLAllocationMaps(logInfo, duplexMode, 0, ...
        muMIMOConfig.MaxNumLayers, allocMapDir, ttiBySlot);
    fprintf('Saved %d DL allocation maps to %s\n', numMaps, allocMapDir);
end

%% ====================== 12. OLLA CONVERGENCE MONITOR ======================

if enableTraces
    rxLogs   = logInfo.PhyReceptionLogs;
    rxColMap = simPhyLogger.ColumnIndexMap;
    frameCol = rxColMap('Frame');
    failCol  = rxColMap('Number of Decode Failures(DL)');
    pktCol   = rxColMap('Number of Packets(DL)');
    rxData   = rxLogs(2:end, :);
    rxFrames = cell2mat(rxData(:, frameCol));

    blerPerFrame = zeros(numFrameSimulation, 1);
    for f = 0:numFrameSimulation-1
        fMask = rxFrames == f;
        if ~any(fMask), continue; end
        fails = sum(cellfun(@sum, rxData(fMask, failCol)));
        pkts  = sum(cellfun(@sum, rxData(fMask, pktCol)));
        if pkts > 0, blerPerFrame(f+1) = fails / pkts; end
    end

    frameColG   = colMap('Frame');
    mcsPerFrame = zeros(numFrameSimulation, 1);
    dlFrames    = cell2mat(dlData(:, frameColG));
    for f = 0:numFrameSimulation-1
        fMask = dlFrames == f;
        if ~any(fMask), continue; end
        mcsPerFrame(f+1) = mean(cell2mat(dlData(fMask, mcsCol)));
    end

    frames = (1:numFrameSimulation)';
    figure('Name', 'OLLA Convergence Monitor — v4.0');
    tiledlayout(2, 1, TileSpacing='compact');

    nexttile;
    plot(frames, blerPerFrame * 100, 'b-o', MarkerSize=4, LineWidth=1.2);
    yline(10, 'r--', 'Target 10%', LabelHorizontalAlignment='left');
    xlabel('Frame'); ylabel('BLER (%)'); title('Per-Frame DL BLER');
    grid on; ylim([0 100]);

    nexttile;
    plot(frames, mcsPerFrame, 'm-o', MarkerSize=4, LineWidth=1.2);
    xlabel('Frame'); ylabel('MCS avg');
    title('Per-Frame DL MCS Average (OLLA accumulating)');
    grid on; ylim([0 28]);

    fprintf('\n  BLER range : %.1f%% – %.1f%%  (first 5 avg: %.1f%%  |  last 5 avg: %.1f%%)\n', ...
        min(blerPerFrame)*100, max(blerPerFrame)*100, ...
        mean(blerPerFrame(1:min(5,end)))*100, ...
        mean(blerPerFrame(max(1,end-4):end))*100);
    fprintf('  MCS  range : %.1f – %.1f  (first 5 avg: %.1f  |  last 5 avg: %.1f)\n\n', ...
        min(mcsPerFrame), max(mcsPerFrame), ...
        mean(mcsPerFrame(1:min(5,end))), ...
        mean(mcsPerFrame(max(1,end-4):end)));
end

%% ====================== 13. MU-MIMO PAIRING ANALYSIS ======================

if enableTraces
    avgUEsPerRB = calculateAvgUEsPerRBDL(logInfo, gNB.NumResourceBlocks, 0, duplexMode);
    figure('Name', 'MU-MIMO Pairing — v4.0');
    histogram(avgUEsPerRB, BinWidth=0.1);
    title('Distribution of Avg UEs per RB (128T128R, DRL Scheduler)');
    xlabel('Avg UEs per RB'); ylabel('Occurrences'); grid on;

    [pairingCounts, ~] = calculateUEPairingStats(logInfo, gNB.NumResourceBlocks, 0, duplexMode);
    figure('Name', 'UE Pairing Distribution — v4.0');
    bar(pairingCounts(:,1), pairingCounts(:,2));
    title('UE Pairing Distribution per RBG'); xlabel('UEs per RBG'); ylabel('Count');
    xticks(0:max(pairingCounts(:,1))); grid on;

    fprintf('\n=== UE PAIRING STATISTICS ===\n');
    totalRBGSlots = sum(pairingCounts(:,2));
    for i = 1:size(pairingCounts, 1)
        n = pairingCounts(i,1); c = pairingCounts(i,2);
        pct = 100 * c / totalRBGSlots;
        if     n == 0, fprintf('  Empty    : %d (%.1f%%)\n', c, pct);
        elseif n == 1, fprintf('  SU-MIMO  : %d (%.1f%%)\n', c, pct);
        else,          fprintf('  %d-user MU: %d (%.1f%%)\n', n, c, pct);
        end
    end
end

diary('off');

%% ====================== HELPER FUNCTIONS ======================

% ── calculateAvgUEsPerRBDL ───────────────────────────────────────────────
function avgUEsPerRB = calculateAvgUEsPerRBDL(logInfo, numResourceBlocks, ratType, duplexMode)
    if strcmp(duplexMode, 'TDD')
        timeStepLogs    = logInfo.TimeStepLogs;
        freqAllocations = timeStepLogs(:, 5);
    else
        timeStepLogs    = logInfo.DLTimeStepLogs;
        freqAllocations = timeStepLogs(:, 4);
    end
    numOfSlots = size(timeStepLogs, 1) - 1;
    if ~ratType
        numRBG       = size(freqAllocations{2}, 2);
        P            = ceil(numResourceBlocks / numRBG);
        numRBsPerRBG = P * ones(1, numRBG);
        if mod(numResourceBlocks, P) > 0, numRBsPerRBG(end) = mod(numResourceBlocks, P); end
    end
    avgUEsPerRB = zeros(1, numOfSlots);
    for slotIdx = 1:numOfSlots
        if strcmp(duplexMode, 'TDD')
            if ~strcmp(timeStepLogs{slotIdx+1, 4}, 'DL'), continue; end
        end
        freqAlloc = freqAllocations{slotIdx + 1};
        if ~ratType
            total = sum(arrayfun(@(rbg) nnz(freqAlloc(:,rbg) > 0) * numRBsPerRBG(rbg), 1:numRBG));
            avgUEsPerRB(slotIdx) = total / numResourceBlocks;
        else
            usage = zeros(1, numResourceBlocks);
            for u = 1:size(freqAlloc, 1)
                usage(freqAlloc(u,1)+1 : freqAlloc(u,1)+freqAlloc(u,2)) = ...
                    usage(freqAlloc(u,1)+1 : freqAlloc(u,1)+freqAlloc(u,2)) + 1;
            end
            avgUEsPerRB(slotIdx) = mean(usage(usage > 0));
        end
    end
    avgUEsPerRB = avgUEsPerRB(avgUEsPerRB > 0);
end

% ── calculateUEPairingStats ───────────────────────────────────────────────
function [pairingCounts, pairingDistribution] = calculateUEPairingStats( ...
        logInfo, numResourceBlocks, ratType, duplexMode)
    if strcmp(duplexMode, 'TDD')
        timeStepLogs    = logInfo.TimeStepLogs;
        freqAllocations = timeStepLogs(:, 5);
    else
        timeStepLogs    = logInfo.DLTimeStepLogs;
        freqAllocations = timeStepLogs(:, 4);
    end
    numOfSlots  = size(timeStepLogs, 1) - 1;
    numRBG      = size(freqAllocations{2}, 2);
    pairingMap  = zeros(1, 17);   % indices: 0..16 UEs
    pairingDistribution = cell(numOfSlots, 1);
    for slotIdx = 1:numOfSlots
        if strcmp(duplexMode, 'TDD')
            if ~strcmp(timeStepLogs{slotIdx+1, 4}, 'DL'), continue; end
        end
        freqAlloc = freqAllocations{slotIdx + 1};
        if ~ratType
            for rbgIdx = 1:numRBG
                n = nnz(freqAlloc(:, rbgIdx) > 0);
                pairingMap(n + 1) = pairingMap(n + 1) + 1;
            end
        else
            usage = zeros(1, numResourceBlocks);
            for u = 1:size(freqAlloc, 1)
                s = freqAlloc(u,1); l = freqAlloc(u,2);
                if l > 0, usage(s+1:s+l) = usage(s+1:s+l) + 1; end
            end
            for rb = 1:numResourceBlocks
                n = usage(rb);
                pairingMap(n + 1) = pairingMap(n + 1) + 1;
            end
        end
        pairingDistribution{slotIdx} = [];
    end
    validIdx    = find(pairingMap > 0);
    pairingCounts = [(validIdx - 1)', pairingMap(validIdx)'];
end

% ── exportDLAllocationMaps ────────────────────────────────────────────────
function numSaved = exportDLAllocationMaps(logInfo, duplexMode, ratType, ...
        maxNumLayers, outDir, ttiBySlot)
    numSaved = 0;
    if ratType ~= 0
        warning('exportDLAllocationMaps: RAT-0 only.'); return;
    end
    if strcmp(duplexMode, 'TDD')
        timeStepLogs = logInfo.TimeStepLogs;
        freqAllocCol = 5;  slotTypeCol = 4;
    else
        timeStepLogs = logInfo.DLTimeStepLogs;
        freqAllocCol = 4;  slotTypeCol = [];
    end
    if size(timeStepLogs, 1) <= 1, return; end
    if ~exist(outDir, 'dir'), mkdir(outDir); end

    txTypeCol    = findLogColumn(timeStepLogs, 'Tx Type', freqAllocCol + 4);
    signalTypeCol = findLogColumn(timeStepLogs, 'Signal Type', -1);
    freqAllocations = timeStepLogs(:, freqAllocCol);
    numSlots = size(timeStepLogs, 1) - 1;

    for slotIdx = 1:numSlots
        if signalTypeCol > 0
            if strcmp(timeStepLogs{slotIdx+1, signalTypeCol}, "CSIRS"), continue; end
        end
        if strcmp(duplexMode, 'TDD')
            st = timeStepLogs{slotIdx+1, slotTypeCol};
            if ~strcmp(st,'DL') && ~strcmp(st,'S'), continue; end
        end
        freqAlloc = freqAllocations{slotIdx + 1};
        if isempty(freqAlloc), continue; end

        txTypePerUE = {};
        if txTypeCol <= size(timeStepLogs,2)
            txTypePerUE = timeStepLogs{slotIdx+1, txTypeCol};
        end

        numRBG  = size(freqAlloc, 2);
        numUEs_ = size(freqAlloc, 1);
        allocMap = zeros(maxNumLayers, numRBG);

        for rbgIdx = 1:numRBG
            sched = find(freqAlloc(:, rbgIdx) > 0);
            nFill = min(numel(sched), maxNumLayers);
            allocMap(1:nFill, rbgIdx) = sched(1:nFill);
        end

        fig = figure('Visible','off');
        cmap = pythonLikeDiscretePalette(numUEs_);
        imagesc(allocMap.' + 1, [1, numUEs_+1]);
        axis tight; colormap(cmap);
        cb = colorbar; cb.Label.String = 'UE Index';
        cb.Ticks = 1:(numUEs_+1); cb.TickLabels = string(0:numUEs_);
        xlabel('Layer'); ylabel('RBG');
        title(sprintf('DL Alloc — slot %d', slotIdx-1));
        xticks(1:maxNumLayers); yticks(1:numRBG);
        set(gca,'YDir','normal','GridAlpha',0.35); grid on;

        for li = 1:maxNumLayers
            for ri = 1:numRBG
                u = allocMap(li, ri);
                if u <= 0, continue; end
                txTag = normalizeTxTag(txTypePerUE, u);
                cellColor = cmap(u+1,:);
                txtColor  = ternary(sum(cellColor) > 1.7, [0 0 0], [1 1 1]);
                text(li, ri, {sprintf('UE%d',u), txTag}, ...
                    'HorizontalAlignment','center','VerticalAlignment','middle', ...
                    'FontSize',7,'FontWeight','bold','Color',txtColor,'Interpreter','none');
            end
        end

        slotLinear = slotIdx - 1;
        ttiId = -1;
        if ~isempty(ttiBySlot) && (slotLinear+1) <= numel(ttiBySlot)
            ttiId = ttiBySlot(slotLinear + 1);
        end
        if ttiId >= 0
            outFile = fullfile(outDir, sprintf('dl_alloc_tti_%05d.png', ttiId));
        else
            outFile = fullfile(outDir, sprintf('dl_alloc_slot_%05d.png', slotLinear));
        end
        exportgraphics(fig, outFile, 'Resolution', 100);
        close(fig);

        save(fullfile(outDir, sprintf('dl_alloc_slot_%05d.mat', slotLinear)), ...
            'slotLinear', 'allocMap', 'freqAlloc', 'txTypePerUE', 'ttiId');
        numSaved = numSaved + 1;
    end
end

% ── findLogColumn ─────────────────────────────────────────────────────────
function colIdx = findLogColumn(timeStepLogs, colName, fallbackIdx)
    colIdx = fallbackIdx;
    if isempty(timeStepLogs), return; end
    idx = find(strcmp(timeStepLogs(1,:), colName), 1);
    if ~isempty(idx), colIdx = idx; end
end

% ── normalizeTxTag ────────────────────────────────────────────────────────
function txTag = normalizeTxTag(txTypePerUE, ueIdx)
    txTag = 'newTx';
    if isempty(txTypePerUE) || ueIdx < 1 || ueIdx > numel(txTypePerUE), return; end
    entry = txTypePerUE{ueIdx};
    if iscell(entry) && ~isempty(entry), entry = entry{1}; end
    if isstring(entry), entry = char(entry); end
    if isempty(entry), return; end
    el = lower(string(entry));
    if contains(el,'retx'), txTag = 'reTx'; elseif contains(el,'newtx'), txTag = 'newTx'; else, txTag = char(entry); end
end

% ── pythonLikeDiscretePalette ─────────────────────────────────────────────
function cmap = pythonLikeDiscretePalette(numUEs)
    tab20 = [31,119,180;174,199,232;255,127,14;255,187,120; ...
             44,160,44;152,223,138;214,39,40;255,152,150; ...
             148,103,189;197,176,213;140,86,75;196,156,148; ...
             227,119,194;247,182,210;127,127,127;199,199,199; ...
             188,189,34;219,219,141;23,190,207;158,218,229] / 255;
    cmap = zeros(numUEs + 1, 3);
    cmap(1,:) = [0.94 0.94 0.94];
    for u = 1:numUEs
        cmap(u+1,:) = tab20(mod(u-1, size(tab20,1)) + 1, :);
    end
end

% ── ternary ───────────────────────────────────────────────────────────────
function out = ternary(cond, trueVal, falseVal)
    if cond, out = trueVal; else, out = falseVal; end
end
