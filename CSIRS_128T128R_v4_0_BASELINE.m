%% NR Cell — 128T128R DL MU-MIMO Baseline (ProportionalFair / RoundRobin / BestCQI)
% Identical simulation environment as v4_0_SOCKET but with MATLAB built-in scheduler.
% Used as conventional-scheduler baseline for DRL performance comparison.
%
% Scheduler options (change 'schedulerType' below):
%   'ProportionalFair'  — PF: standard DRL baseline in literature
%   'RoundRobin'        — RR: lower bound
%   'BestCQI'           — greedy throughput maximiser, upper bound fairness
%
% KPIs saved to logs_baseline/ for comparison with DRL logs_v4/.
%
% Author: ThangTQ23 — VSI, 2026-04

%% ====================== 1. PATH SETUP & LOGGING ======================
clear classes; rehash; close all;

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projectRoot, 'wirelessnetwork')), '-begin');
addpath(genpath(fullfile(projectRoot, '5g')),              '-begin');

wirelessnetworkSupportPackageCheck

schedulerType = 'ProportionalFair';   % 'ProportionalFair' | 'RoundRobin' | 'BestCQI'

logDir  = fullfile(projectRoot, 'logs_baseline');
if exist(logDir, 'dir'), rmdir(logDir, 's'); end
mkdir(logDir);
logFile = fullfile(logDir, 'matlab_log_' + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + '.txt');
diary(logFile);

%% ====================== 2. SIMULATION PARAMETERS ======================

pmiCQIMode = "Subband";

rng("default");
numFrameSimulation = 50;   % longer run for OLLA convergence
networkSimulator   = wirelessNetworkSimulator.init;

%% ====================== 3. gNB CONFIGURATION (128T128R) ======================

gNBPosition = [0 0 30];
duplexMode  = "TDD";

carrierFreq = 4.9e9;
channelBW   = 10e6;
scs         = 30e3;
numRB       = 24;

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

gNB.PMICQIMode = pmiCQIMode;

%% ====================== 4. SCHEDULER CONFIGURATION ======================

numUEs = 16;

muMIMOConfig = struct( ...
    MaxNumUsersPaired    = 4, ...
    MaxNumLayers         = 4, ...
    MinNumRBs            = 2, ...
    SemiOrthogonalityFactor = 0.7, ...
    MinCQI               = 1);

ollaConfig = struct('InitialOffset', 0, 'StepUp', 0.27, 'StepDown', 0.03);

configureScheduler(gNB, ...
    'Scheduler',               schedulerType, ...
    'ResourceAllocationType',  0, ...
    'MaxNumUsersPerTTI',       numUEs, ...
    'PFSWindowSize',           20, ...
    'MUMIMOConfigDL',          muMIMOConfig, ...
    'LinkAdaptationConfigDL',  ollaConfig, ...
    'CSIMeasurementSignalDL',  "CSI-RS", ...
    'RVSequence',              0);

%% ====================== 5. UE DEPLOYMENT ======================
% Identical seed and positions as v4_0_SOCKET for fair comparison.

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

connectUE(gNB, UEs, FullBufferTraffic = "off", CSIReportPeriodicity = 10);
for i = 1:numUEs
    src             = networkTrafficOnOff;
    src.PacketSize  = 1500;
    src.DataRate    = 6e6;
    src.OnTime      = 1e9;
    src.OffTime     = 0;
    addTrafficSource(gNB, src, 'DestinationNode', UEs(i));
end

%% ====================== 6. NETWORK SIMULATOR NODES ======================

addNodes(networkSimulator, gNB);
addNodes(networkSimulator, UEs);

%% ====================== 7. CHANNEL MODEL (CDL-D) ======================

channelConfig = struct( ...
    DelayProfile        = "CDL-D", ...
    DelaySpread         = 100e-9, ...
    MaximumDopplerShift = 5);

channels           = hNRCreateCDLChannels(channelConfig, gNB, UEs);
customChannelModel = hNRCustomChannelModel(channels);
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);

%% ====================== 8. LOGGING & VISUALISATION ======================

enableTraces = true;
if enableTraces
    simSchedulingLogger = helperNRSchedulingLogger(numFrameSimulation, gNB, UEs);
    simPhyLogger        = helperNRPhyLogger(numFrameSimulation, gNB, UEs);
end

metricsVisualizer = helperNRMetricsVisualizer(gNB, UEs, ...
    'RefreshRate',          1000, ...
    'PlotSchedulerMetrics', true, ...
    'PlotPhyMetrics',       false, ...
    'PlotCDFMetrics',       true, ...
    'LinkDirection',        0);

%% ====================== 9. SIMULATION CONDITIONS SUMMARY ======================

slotsPerSubfm  = scs / 15e3;
slotDurMs      = 1 / slotsPerSubfm;
slotsPerFrm    = 10 * slotsPerSubfm;
csiPeriodicity = 10;
numCSIEvents   = floor(numFrameSimulation * slotsPerFrm / csiPeriodicity);

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
fprintf('  SIMULATION CONDITIONS — 128T128R MU-MIMO DL Baseline (v4.0)\n');
fprintf('%s\n', sep);
fprintf('  gNB       : 128T/128R | %.1f GHz (n79) | %s | BW=%.0f MHz | NRB=%d\n', ...
    carrierFreq/1e9, duplexMode, channelBW/1e6, numRB);
fprintf('              SCS=%.0f kHz | Tx=40 dBm | Rx Gain=11 dB\n', scs/1e3);
fprintf('  UEs       : %d | %dTx/%dRx | r∈[%d,%d] m | NF∈[%d,%d] dB\n', ...
    numUEs, ueNumTx, ueNumRx, rMin, rMax, noiseFigMin, noiseFigMax);
fprintf('  Channel   : %s | DS=%.0f ns | fD=%.0f Hz\n', ...
    channelConfig.DelayProfile, channelConfig.DelaySpread*1e9, ...
    channelConfig.MaximumDopplerShift);
fprintf('  Scheduler : %s | MU-MIMO ≤%d paired / ≤%d layers | RAT-0\n', ...
    schedulerType, muMIMOConfig.MaxNumUsersPaired, muMIMOConfig.MaxNumLayers);
fprintf('              OLLA: init=%g up=%.2f down=%.2f → target BLER~%.0f%%\n', ...
    ollaConfig.InitialOffset, ollaConfig.StepUp, ollaConfig.StepDown, ...
    100*ollaConfig.StepDown/(ollaConfig.StepDown+ollaConfig.StepUp));
fprintf('  CSI       : eTypeII-r19 | Panel [1×16×4] | L=2 beams | %s\n', modeStr);
fprintf('              Period=%d slots (%.1f ms) → ~%d reports/UE in %d frames\n', ...
    csiPeriodicity, csiPeriodicity*slotDurMs, numCSIEvents, numFrameSimulation);
fprintf('  Traffic   : FTP Model 3 | 1500 B pkts | 6 Mbps/UE\n');
fprintf('  Sim       : %d frames | %.0f ms\n', numFrameSimulation, numFrameSimulation*10);
fprintf('%s\n\n', sep);

%% ====================== 10. RUN SIMULATION ======================

simulationTime = numFrameSimulation * 1e-2;
fprintf('Running simulation (%.0f ms)...\n', simulationTime * 1e3);
run(networkSimulator, simulationTime);
fprintf('Simulation complete.\n');

%% ====================== 11. KPI SUMMARY & LOGS ======================

displayPerformanceIndicators(metricsVisualizer);

if enableTraces
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
    save('logInfo_baseline.mat', 'logInfo');

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
    fprintf('  gNB DL Grant Summary  (%d grants | %d frames | %s | %s)\n', ...
        sum(isDL), numFrameSimulation, schedulerType, pmiCQIMode);
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
    figure('Name', sprintf('OLLA Convergence — Baseline (%s)', schedulerType));
    tiledlayout(2, 1, TileSpacing='compact');

    nexttile;
    plot(frames, blerPerFrame * 100, 'b-o', MarkerSize=4, LineWidth=1.2);
    yline(10, 'r--', 'Target 10%', LabelHorizontalAlignment='left');
    xlabel('Frame'); ylabel('BLER (%)');
    title(sprintf('Per-Frame DL BLER — %s', schedulerType));
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
    figure('Name', sprintf('MU-MIMO Pairing — Baseline (%s)', schedulerType));
    histogram(avgUEsPerRB, BinWidth=0.1);
    title(sprintf('Distribution of Avg UEs per RB (%s)', schedulerType));
    xlabel('Avg UEs per RB'); ylabel('Occurrences'); grid on;

    [pairingCounts, ~] = calculateUEPairingStats(logInfo, gNB.NumResourceBlocks, 0, duplexMode);
    figure('Name', sprintf('UE Pairing Distribution — Baseline (%s)', schedulerType));
    bar(pairingCounts(:,1), pairingCounts(:,2));
    title(sprintf('UE Pairing Distribution per RBG — %s', schedulerType));
    xlabel('UEs per RBG'); ylabel('Count');
    xticks(0:max(pairingCounts(:,1))); grid on;

    fprintf('\n=== UE PAIRING STATISTICS (%s) ===\n', schedulerType);
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
    pairingMap  = zeros(1, 17);
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
    validIdx      = find(pairingMap > 0);
    pairingCounts = [(validIdx - 1)', pairingMap(validIdx)'];
end
