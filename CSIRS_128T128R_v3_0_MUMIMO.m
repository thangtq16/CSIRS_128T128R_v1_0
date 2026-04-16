%% NR Cell Performance — 128T128R Downlink MU-MIMO (Rel-19 eTypeII)
% Evaluates DL MU-MIMO system performance using abstract PHY (L2SM),
% 128-antenna gNB with eTypeII-r19 codebook (TS 38.214 §5.2.2.2.5a).
%
% Author: ThangTQ23 — VSI, 2026-04

%% 1. Simulation Setup

% Prepend local patched wirelessnetwork + 5g toolbox overrides so that
% 128T-capable nrGNB (allows NumTransmitAntennas up to 128) and the
% eTypeII-r19 PHY stack are found before the installed support packages.
projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projectRoot, 'wirelessnetwork')), '-begin');
addpath(genpath(fullfile(projectRoot, '5g')),              '-begin');

wirelessnetworkSupportPackageCheck

% ── PMI / CQI reporting mode for 128T eTypeII-r19 ──────────────────────────
% 'Wideband' : single precoder + single CQI across full BW.
%              Fastest simulation; suitable for quick sweeps or low-spec PCs.
% 'Subband'  : per-subband precoder (PMI) + per-subband CQI (Phase 4b).
%              Each subband = 4 RBs (TS 38.214 Table 5.2.1.4-2, NRB∈[24,72]).
%              Best frequency-selective accuracy; ~6× slower LQM compute.
pmiCQIMode = "Subband";   % <-- change to "Wideband" for faster runs

rng("default")
numFrameSimulation = 5;
networkSimulator   = wirelessNetworkSimulator.init;

%% 2. gNB Configuration
% 128T128R massive MIMO, TDD, 4.9 GHz n79, 10 MHz / 30 kHz SCS
% (Use channelBW = 100e6 for production; 10 MHz here for fast simulation.)

gNBPosition  = [0 0 30];       % [x y z] metres
duplexType   = "TDD";
numUEs       = 16;             % total UEs — defined here so SRSPeriodicityUE can auto-scale

carrierFreq  = 4.9e9;          % n79 band centre frequency (Hz)
channelBW    = 10e6;           % channel bandwidth (Hz)
scs          = 30e3;           % subcarrier spacing (Hz)

% Valid NRB per 3GPP TS 38.104 Table 5.3.2-1 for FR1 30 kHz SCS:
%   5 MHz→11, 10 MHz→24, 15 MHz→38, 20 MHz→51, 40 MHz→106, 100 MHz→273
numRB = 24;   % matches channelBW = 10 MHz, SCS = 30 kHz

gNB = nrGNB( ...
    Position             = gNBPosition, ...
    TransmitPower        = 60, ...
    CarrierFrequency     = carrierFreq, ...
    ChannelBandwidth     = channelBW, ...
    SubcarrierSpacing    = scs, ...
    NumResourceBlocks    = numRB, ...
    NumTransmitAntennas  = 128, ...
    NumReceiveAntennas   = 128, ...
    DuplexMode           = duplexType, ...
    ReceiveGain          = 11, ...
    SRSPeriodicityUE     = 20);

disp("gNB: " + duplexType + ", " + carrierFreq/1e9 + " GHz, " + ...
     channelBW/1e6 + " MHz, NRB=" + numRB + ", 128T128R");

%% 3. Scheduler Configuration
% CSI measurement source: "CSI-RS" (eTypeII-r19 feedback) or "SRS" (TDD reciprocity)

csiMeasurementSignalDLType = "CSI-RS";
allocationType             = 0;   % RAT-0 (RBG-based)

% CSI-RS mode fields: SemiOrthogonalityFactor, MinCQI (not MinSINR — that is SRS-only)
% ThangTQ23_128T128R_Rel19: MaxNumUsersPaired=2 matches DOF constraint.
% L=2 basis beams × 2 polarizations = 4D effective subspace.
% K*r ≤ 4 for orthogonality → with rank=2 (RIRestriction=[1 1 0 0]): K ≤ 2.
% K=4 rank-2 = 8 streams in 4D → mutual orthogonality impossible → high BLER.
% K=2 rank-2 = 4 streams in 4D → two orthogonal 2D planes → works.
muMIMOConfiguration = struct( ...
    MaxNumUsersPaired       = 4, ...
    MaxNumLayers            = 8, ...
    MinNumRBs               = 2, ...
    SemiOrthogonalityFactor = 0.7, ...
    MinCQI                  = 1);

% OLLA: TargetBLER = StepDown/(StepDown+StepUp) = 0.03/0.30 = 10%
% State 3: reset removed in nrComponentCarrierContext.updateChannelQualityDL.
% MCSOffset accumulates via HARQ ACK/NACK across the full simulation.
% MU-MIMO CQI is SU-measured → overestimates SINR → OLLA must accumulate
% over many grants to reach steady-state before BLER converges to ~10%.
oLLAConfig = struct('InitialOffset', 0, 'StepUp', 0.27, 'StepDown', 0.03);

configureScheduler(gNB, ...
    ResourceAllocationType  = allocationType, ...
    MaxNumUsersPerTTI       = 10, ...
    MUMIMOConfigDL          = muMIMOConfiguration, ...
    CSIMeasurementSignalDL  = csiMeasurementSignalDLType, ...
    LinkAdaptationConfigDL  = oLLAConfig);

%% 4. UE Deployment
% UEs scattered across a 120° sector, min 50 m – max 500 m from gNB.
% Distance drawn from uniform distribution on area (pdf ∝ r) via sqrt-trick
% so that UEs are spread evenly over the sector area, not clustered near gNB.

ueRxGain       = 0;
ueNumTxAnt     = 4;
ueNumRxAnt     = 4;

rMin = 50;   rMax = 500;   % metres
azDeg  = -60 + 120 .* rand(numUEs, 1);                      % uniform azimuth in [-60°, +60°]
rDist  = sqrt(rMin^2 + (rMax^2 - rMin^2) .* rand(numUEs, 1)); % uniform on area (pdf ∝ r)
[xPos, yPos, zPos] = sph2cart(deg2rad(azDeg), zeros(numUEs,1), rDist);
uePositions = [xPos, yPos, zPos] + gNBPosition;
ueNames     = "UE-" + (1:numUEs);

UEs = nrUE( ...
    Name                = ueNames, ...
    Position            = uePositions, ...
    ReceiveGain         = ueRxGain, ...
    NumTransmitAntennas = ueNumTxAnt, ...
    NumReceiveAntennas  = ueNumRxAnt);

% Apply PMI/CQI mode before connectUE so nrNodeValidation picks it up.
gNB.PMICQIMode = pmiCQIMode;

connectUE(gNB, UEs, FullBufferTraffic = "DL", CSIReportPeriodicity = 10);

%% 5. Network Simulator — Add Nodes

addNodes(networkSimulator, gNB);
addNodes(networkSimulator, UEs);

%% 6. CDL Channel Model
% CDL-B, 100 ns delay spread, 5 Hz Doppler (frequency-selective, rank-friendly)

channelConfig = struct( ...
    DelayProfile        = "CDL-D", ...
    DelaySpread         = 100e-9, ...
    MaximumDopplerShift = 5);

channels = hNRCreateCDLChannels(channelConfig, gNB, UEs);
customChannelModel = hNRCustomChannelModel(channels);
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);

%% 7. Logging and Visualization

enableTraces = true;

if enableTraces
    simSchedulingLogger = helperNRSchedulingLogger(numFrameSimulation, gNB, UEs);
    simPhyLogger        = helperNRPhyLogger(numFrameSimulation, gNB, UEs);
end

numMetricPlotUpdates = 200;
metricsVisualizer = helperNRMetricsVisualizer(gNB, UEs, ...
    RefreshRate         = numMetricPlotUpdates, ...
    PlotSchedulerMetrics = true, ...
    PlotPhyMetrics      = false, ...
    PlotCDFMetrics      = true, ...
    LinkDirection       = 0);

simulationLogFile = "simulationLogs";

%% 8. Simulation Conditions Summary

slotsPerSubfm  = scs / 15e3;           % slots per subframe (2 for 30 kHz SCS)
slotDurMs      = 1 / slotsPerSubfm;    % ms per slot
slotsPerFrm    = 10 * slotsPerSubfm;   % slots per 10-ms frame
csiPeriodicity = 10;                   % slots (set in connectUE call above)
csiPeriodMs    = csiPeriodicity * slotDurMs;
numCSIEvents   = floor(numFrameSimulation * slotsPerFrm / csiPeriodicity);

if strcmpi(pmiCQIMode, 'Subband')
    if numRB >= 145; sbSize = 16; elseif numRB >= 73; sbSize = 8; else; sbSize = 4; end
    numSubbands = ceil(numRB / sbSize);
    oh_cqi = 4 + 2*(numSubbands - 1);   % wideband base + differential per extra subband
    modeStr = sprintf('Subband (PMI+CQI) | %d × %d RBs', numSubbands, sbSize);
else
    sbSize = numRB;  numSubbands = 1;  oh_cqi = 4;
    modeStr = 'Wideband (PMI+CQI)';
end
oh_ri = 2;  oh_pmi = 20;   % RI: ceil(log2(4)); PMI approx (eTypeII-r19, L=2)
oh_total = oh_ri + oh_cqi + oh_pmi;

sep = repmat('═', 1, 67);
fprintf('\n%s\n', sep);
fprintf('  SIMULATION CONDITIONS — 128T128R MU-MIMO DL, Rel-19 eTypeII\n');
fprintf('%s\n', sep);
fprintf('  System\n');
fprintf('    gNB  : %dT/%dR | %.1f GHz (n79) | %s | %d dBm Tx\n', ...
    gNB.NumTransmitAntennas, gNB.NumReceiveAntennas, ...
    carrierFreq/1e9, duplexType, 34);
fprintf('           BW=%.0f MHz | NRB=%d | SCS=%.0f kHz | Rx Gain=11 dB\n', ...
    channelBW/1e6, numRB, scs/1e3);
fprintf('    UE   : %d nodes | %dTx/%dRx | R=500 m | Az=[-60°,+60°] | Rx Gain=0 dB\n', ...
    numUEs, ueNumTxAnt, ueNumRxAnt);
fprintf('  Channel  : %s | RMS delay=%.0f ns | f_D=%.0f Hz\n', ...
    channelConfig.DelayProfile, channelConfig.DelaySpread*1e9, channelConfig.MaximumDopplerShift);
fprintf('  Scheduler\n');
fprintf('    Source : %s | MU-MIMO ≤%d paired / ≤%d layers | RAT-%d\n', ...
    csiMeasurementSignalDLType, muMIMOConfiguration.MaxNumUsersPaired, ...
    muMIMOConfiguration.MaxNumLayers, allocationType);
fprintf('             MinCQI=%d | Semiortho=%.1f | SRS period=20 ms\n', ...
    muMIMOConfiguration.MinCQI, muMIMOConfiguration.SemiOrthogonalityFactor);
fprintf('  CSI Feedback\n');
fprintf('    Signal   : CSI-RS (4×32-port Row-18 resources ≡ 128T)\n');
fprintf('    Codebook : eTypeII-r19 | Panel [1×16×4] | L=2 beams | PC=1\n');
fprintf('    Mode     : %s\n', modeStr);
fprintf('    Period   : %d slots (%.1f ms) → ~%d reports/UE in %d frames\n', ...
    csiPeriodicity, csiPeriodMs, numCSIEvents, numFrameSimulation);
fprintf('    OH est.  : RI=%d + CQI=%d + PMI≈%d ≈ %d bits/report\n', ...
    oh_ri, oh_cqi, oh_pmi, oh_total);
fprintf('    [Live CSI events printed below for UE-1 and UE-2 only]\n');
fprintf('  Simulation : %d frames | %.0f ms | full-buffer DL | logging=%s\n', ...
    numFrameSimulation, numFrameSimulation*10, upper(char(string(enableTraces))));
fprintf('%s\n\n', sep);

%% 9. Run Simulation

simulationTime = numFrameSimulation * 1e-2;   % seconds
run(networkSimulator, simulationTime);

%% 10. Results — KPI Summary
% Display cell throughput, spectral efficiency, and BLER ECDF.

displayPerformanceIndicators(metricsVisualizer);

%% 11. Save Traces

if enableTraces
    simulationLogs = cell(1,1);
    if gNB.DuplexMode == "FDD"
        logInfo = struct(DLTimeStepLogs=[], ULTimeStepLogs=[], ...
                         SchedulingAssignmentLogs=[], PhyReceptionLogs=[]);
        [logInfo.DLTimeStepLogs, logInfo.ULTimeStepLogs] = ...
            getSchedulingLogs(simSchedulingLogger);
    else  % TDD
        logInfo = struct(TimeStepLogs=[], SchedulingAssignmentLogs=[], ...
                         PhyReceptionLogs=[]);
        logInfo.TimeStepLogs = getSchedulingLogs(simSchedulingLogger);
    end
    logInfo.SchedulingAssignmentLogs = getGrantLogs(simSchedulingLogger);
    logInfo.PhyReceptionLogs         = getReceptionLogs(simPhyLogger);
    simulationLogs{1} = logInfo;
    save(simulationLogFile, "simulationLogs");
end

%% 11b. Grant / MCS Summary — gNB Downlink Decisions

if enableTraces
    colMap   = simSchedulingLogger.GrantLogsColumnIndexMap;
    % getGrantLogs prepends an alphabetical header row; skip it, use colMap for column indices.
    grantRaw = logInfo.SchedulingAssignmentLogs(2:end, :);

    rntiCol  = colMap('RNTI');
    mcsCol   = colMap('MCS Index');
    lyrCol   = colMap('NumLayers');
    typeCol  = colMap('Grant Type');

    isDL   = strcmp(grantRaw(:, typeCol), 'DL');
    dlData = grantRaw(isDL, :);
    nDL    = sum(isDL);

    sep2 = repmat('─', 1, 67);
    fprintf('\n%s\n', sep2);
    fprintf('  gNB DL Grant Summary  (%d grants | %d frames | %s mode)\n', ...
        nDL, numFrameSimulation, pmiCQIMode);
    fprintf('%s\n', sep2);
    fprintf('  %-6s  %7s  %8s  %8s  %8s  %8s\n', ...
        'UE', 'Grants', 'MCS avg', 'MCS min', 'MCS max', 'Lyr avg');
    fprintf('%s\n', sep2);
    for ueIdx = 1:numUEs
        mask = cell2mat(dlData(:, rntiCol)) == ueIdx;
        if ~any(mask); continue; end
        mcsV = cell2mat(dlData(mask, mcsCol));
        lyrV = cell2mat(dlData(mask, lyrCol));
        fprintf('  UE-%-3d  %7d  %8.1f  %8d  %8d  %8.1f\n', ...
            ueIdx, sum(mask), mean(mcsV), min(mcsV), max(mcsV), mean(lyrV));
    end
    fprintf('%s\n\n', sep2);
end

%% 11c. Per-Frame BLER and MCS Trend — OLLA Convergence Monitor
% Shows how BLER and average MCS evolve frame-by-frame.
% In State 3 (no OLLA reset), BLER should decrease and MCS avg should drop
% as MCSOffset accumulates. Flat/rising BLER → OLLA not converging.

if enableTraces
    % ── Per-frame BLER from PhyReceptionLogs ─────────────────────────────
    rxLogs   = logInfo.PhyReceptionLogs;
    rxColMap = simPhyLogger.ColumnIndexMap;

    frameCol  = rxColMap('Frame');
    failCol   = rxColMap('Number of Decode Failures(DL)');
    pktCol    = rxColMap('Number of Packets(DL)');

    rxData    = rxLogs(2:end, :);           % skip header row
    rxFrames  = cell2mat(rxData(:, frameCol));

    blerPerFrame = zeros(numFrameSimulation, 1);
    for f = 0:numFrameSimulation-1
        fMask  = rxFrames == f;
        if ~any(fMask); continue; end
        fails  = sum(cellfun(@sum, rxData(fMask, failCol)));
        pkts   = sum(cellfun(@sum, rxData(fMask, pktCol)));
        if pkts > 0
            blerPerFrame(f+1) = fails / pkts;
        end
    end

    % ── Per-frame MCS avg from GrantLogs ─────────────────────────────────
    frameColG  = colMap('Frame');
    mcsPerFrame = zeros(numFrameSimulation, 1);
    dlFrames    = cell2mat(dlData(:, frameColG));
    for f = 0:numFrameSimulation-1
        fMask = dlFrames == f;
        if ~any(fMask); continue; end
        mcsPerFrame(f+1) = mean(cell2mat(dlData(fMask, mcsCol)));
    end

    % ── Plot ─────────────────────────────────────────────────────────────
    frames = (1:numFrameSimulation)';

    figure('Name', 'OLLA Convergence Monitor');
    tiledlayout(2, 1, TileSpacing='compact');

    nexttile;
    plot(frames, blerPerFrame * 100, 'b-o', MarkerSize=4, LineWidth=1.2);
    yline(10, 'r--', 'Target 10%', LabelHorizontalAlignment='left');
    xlabel('Frame'); ylabel('BLER (%)');
    title('Per-Frame DL BLER');
    grid on; ylim([0 100]);

    nexttile;
    plot(frames, mcsPerFrame, 'm-o', MarkerSize=4, LineWidth=1.2);
    xlabel('Frame'); ylabel('MCS avg');
    title('Per-Frame DL MCS Average (OLLA reducing over time)');
    grid on; ylim([0 28]);

    fprintf('\n  Per-frame BLER range : %.1f%% – %.1f%%  (first 5 frames avg: %.1f%%  |  last 5 frames avg: %.1f%%)\n', ...
        min(blerPerFrame)*100, max(blerPerFrame)*100, ...
        mean(blerPerFrame(1:min(5,end)))*100, ...
        mean(blerPerFrame(max(1,end-4):end))*100);
    fprintf('  Per-frame MCS  range : %.1f – %.1f  (first 5 frames avg: %.1f  |  last 5 frames avg: %.1f)\n\n', ...
        min(mcsPerFrame), max(mcsPerFrame), ...
        mean(mcsPerFrame(1:min(5,end))), ...
        mean(mcsPerFrame(max(1,end-4):end)));
end

%% 12. MU-MIMO Pairing Analysis — UEs per RB Distribution
% Histogram shows how often multiple UEs share the same RB in DL slots.

if enableTraces
    avgNumUEsPerRB = calculateAvgUEsPerRBDL(logInfo, gNB.NumResourceBlocks, ...
                                             allocationType, duplexType);
    figure;
    histogram(avgNumUEsPerRB, BinWidth=0.1);
    title("Distribution of Average Number of UEs per RB in DL Slots");
    xlabel("Average Number of UEs per RB");
    ylabel("Number of Occurrences");
    grid on;
end

%% Local Function

function avgUEsPerRB = calculateAvgUEsPerRBDL(logInfo, numResourceBlocks, ratType, duplexMode)
    % Returns average number of UE nodes per RB for each DL slot.
    % Supports RAT-0 (RBG-based) and RAT-1 (contiguous-RB-based) allocation.

    if strcmp(duplexMode, 'TDD')
        timeStepLogs    = logInfo.TimeStepLogs;
        freqAllocations = timeStepLogs(:, 5);
    else
        timeStepLogs    = logInfo.DLTimeStepLogs;
        freqAllocations = timeStepLogs(:, 4);
    end

    numOfSlots = size(timeStepLogs, 1) - 1;

    if ~ratType
        % Derive per-RBG sizes per TS 38.214
        numRBG       = size(freqAllocations{2}, 2);
        P            = ceil(numResourceBlocks / numRBG);
        numRBsPerRBG = P * ones(1, numRBG);
        remainder    = mod(numResourceBlocks, P);
        if remainder > 0
            numRBsPerRBG(end) = remainder;
        end
    end

    avgUEsPerRB = zeros(1, numOfSlots);

    for slotIdx = 1:numOfSlots
        % Skip UL slots in TDD
        if strcmp(duplexMode, 'TDD') && ~strcmp(timeStepLogs{slotIdx+1, 4}, 'DL')
            continue;
        end

        freqAllocation = freqAllocations{slotIdx + 1};

        if ~ratType
            % RAT-0: weight each RBG by its RB count
            totalUniqueUEs = sum(arrayfun( ...
                @(i) nnz(freqAllocation(:,i) > 0) * numRBsPerRBG(i), ...
                1:length(numRBsPerRBG)));
            avgUEsPerRB(slotIdx) = totalUniqueUEs / numResourceBlocks;
        else
            % RAT-1: count UEs per contiguous RB allocation
            ueRBUsage = zeros(1, numResourceBlocks);
            for ueIdx = 1:size(freqAllocation, 1)
                startRB = freqAllocation(ueIdx, 1);
                numRBs  = freqAllocation(ueIdx, 2);
                ueRBUsage(startRB+1 : startRB+numRBs) = ...
                    ueRBUsage(startRB+1 : startRB+numRBs) + 1;
            end
            avgUEsPerRB(slotIdx) = mean(ueRBUsage(ueRBUsage > 0));
        end
    end

    % Remove zero entries (UL slots)
    avgUEsPerRB = avgUEsPerRB(avgUEsPerRB > 0);
end
