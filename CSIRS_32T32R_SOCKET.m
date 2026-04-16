%% ====================== 1. SYSTEM CONFIGURATION ======================
% Ensure MATLAB reloads edited classdefs (nrScheduler / nrUEContext / etc.)
clear classes;
rehash;
close all;
scriptDir = fileparts(mfilename('fullpath'));
addpath(scriptDir)
addpath(fullfile(scriptDir, "nr_framework"))
addpath(fullfile(scriptDir, "utils"))

logDir = fullfile(scriptDir, "logs_new");
rmdir(logDir,"s")
if ~exist(logDir, 'dir')
   mkdir(logDir);
end
logFile = fullfile(logDir, "matlab_log_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss")) + ".txt");
diary(logFile);


% --- gNB CONFIGURATION (32T32R) ---
gNBConfig = struct();
gNBConfig.Position = [0 0 30];            % Vị trí [x,y,z]
gNBConfig.TransmitPower = 30;             % Công suất phát (dBm) ~ theo yêu cầu trung bình
gNBConfig.SubcarrierSpacing = 30000;      % 30 kHz
gNBConfig.CarrierFrequency = 4.9e9;       % 4.9 GHz
gNBConfig.ChannelBandwidth = 100e6;       % 100 MHz
% Với 100MHz @ 30kHz SCS, số lượng RB chuẩn là 273
gNBConfig.NumResourceBlocks = 273;        
gNBConfig.NumTransmitAntennas = 32;       % 32 Anten phát
gNBConfig.NumReceiveAntennas = 32;        % 32 Anten thu
gNBConfig.ReceiveGain = 32;               % Gain thu 32 dBi
gNBConfig.DuplexMode = "TDD";             % TDD
% Explicit TDD Configuration for a 5-slot 'D-D-S-U-U' pattern (2.5ms @ 30kHz)
% gNBConfig.DLULConfigTDD = struct('DLULPeriodicity', 2.5, ...
%     'NumDLSlots', 2, ...         % D-D
%     'NumDLSymbols', 10, ...       % S slot: 10 DL, 3 Guard, 1 UL
%     'NumULSymbols', 1, ...
%     'NumULSlots', 2);            % U-U
gNBConfig.SRSPeriodicity = 5;             % SRS Periodicity = 5 slots

% --- UE CONFIGURATION ---
ueConfig = struct();
ueConfig.NumUEs = 32;                    % 16 UEs connected
ueConfig.NumTransmitAntennas = 1;
ueConfig.NumReceiveAntennas = 1;          % 4 Anten thu mỗi UE
ueConfig.ReceiveGain = 0;                 % Gain 0 dBi
ueConfig.MaxDistance = 2000;              % Bán kính 1200m
ueConfig.MinDistance = 100;                
ueConfig.AzimuthRange = [-90 90];         % Góc phương vị +/- 30 độ
ueConfig.ElevationAngle = 0;    
ueConfig.NoiseFigureMin = 7;              % More realistic NF: 7-9 dB
ueConfig.NoiseFigureMax = 9;             

% --- MU-MIMO & SCHEDULER ---
muMIMOConfig = struct();
muMIMOConfig.MaxNumUsersPaired = 4;       % 
muMIMOConfig.MinNumRBs = 3;               % Min 3 RBs
muMIMOConfig.SemiOrthogonalityFactor = 0.4; 
muMIMOConfig.MinCQI = 1;                  % Tương ứng CQI 1
muMIMOConfig.MaxNumLayers = 4;          % 16 spatial streams per TTI (matches Python --n_layers 16)
schedulerConfig = struct();
schedulerConfig.ResourceAllocationType = 0;  % RB-based
schedulerConfig.MaxNumUsersPerTTI = ueConfig.NumUEs;      % Max 4 UE scheduled per TTI
schedulerConfig.SignalType = "CSI-RS";      % Use CSI-RS for channel measurement
schedulerConfig.PFSWindowSize = 20;                  % EMA window 20 slots (avg_tp_bps cho Python obs)
schedulerConfig.RVSequence = [0];                      % Disable HARQ retransmissions (RV0 only → no reTx) otherwise  RVSequence = [0 3 2 1]

% --- CSI report granularity (Subband/PRG PMI) ---
% Choose subband/PRG size in RBs. Typical values: 2 or 4.
csiReportConfig = struct();
csiReportConfig.SubbandSize = 16;
csiReportConfig.PRGSize = 4;

% --- CHANNEL MODEL ---
channelConfig = struct();
channelConfig.DelayProfile = "CDL-D";       
channelConfig.DelaySpread = 450e-9;         % 450ns
channelConfig.MaxDopplerShift = 136;        % ~136 Hz
channelConfig.Orientation = [60; 0; 0];     % Hướng anten gNB

% --- SIMULATION CONTROL ---
simConfig = struct();
simConfig.NumFrameSimulation = 20;          
simConfig.EnableTraces = true;              

%% ====================== 2. INITIALIZATION ======================

wirelessnetworkSupportPackageCheck
rng("default");
networkSimulator = wirelessNetworkSimulator.init;

% --- Create gNB (32T32R) ---
gNB = nrGNB('Position', gNBConfig.Position, ...
    'TransmitPower', gNBConfig.TransmitPower, ...
    'SubcarrierSpacing', gNBConfig.SubcarrierSpacing, ...
    'CarrierFrequency', gNBConfig.CarrierFrequency, ...
    'ChannelBandwidth', gNBConfig.ChannelBandwidth, ...
    'NumTransmitAntennas', gNBConfig.NumTransmitAntennas, ... 
    'NumReceiveAntennas', gNBConfig.NumReceiveAntennas, ...   
    'DuplexMode', gNBConfig.DuplexMode, ...
    'ReceiveGain', gNBConfig.ReceiveGain, ...
    'SRSPeriodicityUE', gNBConfig.SRSPeriodicity, ... 
    'NumResourceBlocks', gNBConfig.NumResourceBlocks);

% --- Explicit CSI-RS configuration (ensure CSI-RS is enabled for UEs) ---
% Create CSI-RS config manually (same logic as gNB.createCSIRSConfiguration)
csirsRowNumberTable = [
    1 1 1; % Row 1: density 3 (not used)
    1 1 1; % Row 2
    2 1 1; % Row 3
    4 1 1; % Row 4
    4 1 1; % Row 5
    8 4 1; % Row 6
    8 2 1; % Row 7
    8 2 1; % Row 8
    12 6 1; % Row 9
    12 3 1; % Row 10
    16 4 1; % Row 11
    16 4 1; % Row 12
    24 3 2; % Row 13
    24 3 2; % Row 14
    24 3 1; % Row 15
    32 4 2; % Row 16
    32 4 2; % Row 17HARQ
    32 4 1; % Row 18
];
subcarrierSet = [1 3 5 7 9 11]; % k0 k1 k2 k3 k4 k5
symbolSet = [0 4]; % l0 l1

csirsCfg = nrCSIRSConfig(CSIRSType="nzp", NumRB=gNBConfig.NumResourceBlocks);
rowNum = find(csirsRowNumberTable(2:end, 1) == gNBConfig.NumTransmitAntennas, 1) + 1;
csirsCfg.RowNumber = rowNum;
csirsCfg.SubcarrierLocations = subcarrierSet(1:csirsRowNumberTable(rowNum, 2));
csirsCfg.SymbolLocations = symbolSet(1:csirsRowNumberTable(rowNum, 3));
% CSI staleness fix: With Doppler 136Hz, coherence time ~3.7ms
% CSI period must be << coherence time for accurate precoding
csirsCfg.CSIRSPeriod = [10 0]; % Reduced from 10 to 2 slots (~1ms)
gNB.CSIRSConfiguration = csirsCfg;
fprintf('CSI-RS configured: %d ports, Row %d, Period [%d %d]\n', ...
    gNBConfig.NumTransmitAntennas, rowNum, csirsCfg.CSIRSPeriod(1), csirsCfg.CSIRSPeriod(2));

% --- Configure Scheduler ---
if schedulerConfig.SignalType == "SRS"
    muMIMOStruct = struct(...
        'MaxNumUsersPaired', muMIMOConfig.MaxNumUsersPaired, ...
        'MinNumRBs', muMIMOConfig.MinNumRBs, ...
        'MaxNumLayers', muMIMOConfig.MaxNumLayers);
else
    muMIMOStruct = struct(...
        'MaxNumUsersPaired', muMIMOConfig.MaxNumUsersPaired, ...
        'MinNumRBs', muMIMOConfig.MinNumRBs, ...
        'SemiOrthogonalityFactor', muMIMOConfig.SemiOrthogonalityFactor, ...
        'MinCQI', muMIMOConfig.MinCQI, ...
        'MaxNumLayers', muMIMOConfig.MaxNumLayers);
end

% === DRL Scheduler (nrDRLScheduler với Socket) ===
drlScheduler = nrDRLScheduler();

% Cấu hình DRL scheduler
drlScheduler.EnableDRL = true;
drlScheduler.TrainingMode = true;  % Enable layer-by-layer training protocol (train_matlab.py)
drlScheduler.NumLayers = muMIMOConfig.MaxNumLayers;  % 16 layers
drlScheduler.MaxUsersPerRBG = muMIMOConfig.MaxNumUsersPaired;
drlScheduler.MaxUEs = ueConfig.NumUEs;  % Số UE tối đa (cho feature matrix)
drlScheduler.SubbandSize = csiReportConfig.SubbandSize;  % Subband size cho CQI features
drlScheduler.DRLDebug = false;  % Bật debug output
drlScheduler.MinFreeHARQForNewTx = 1;  % Keep HARQ headroom; avoid freeHarqId = -1 lock-up
drlScheduler.MaxDLHARQBusyTTI = 30;  % HARQ watchdog: force-release stale busy DL HARQ processes
drlScheduler.HARQReleaseDebug = false;  % Detailed logs for HARQ watchdog release behavior

% CSI-RS based MU-MIMO constraints
% Set to false to disable both i1 matching AND precoder orthogonality checks
drlScheduler.EnablePairingConstraints = false;
drlScheduler.SemiOrthogonalityFactor = muMIMOConfig.SemiOrthogonalityFactor;  % 0.6
drlScheduler.MU_MCSBackoff = 0;                      % Reduce MCS by 2 per co-scheduled UE
drlScheduler.MaxDLReTxOccupancyRatio = 0.6;  % Leave more room for DL ReTx to prevent HARQ starvation

% Kết nối tới Python DRL server qua socket
ok = drlScheduler.connectToDRLAgent('127.0.0.1', 6666);
assert(ok, 'Cannot connect to DRL server. Start python train_drl_with_matlab.py first.');

% --- Link Adaptation (OLLA) for MU-MIMO ---
% BLER was 40-60%, need more aggressive OLLA to converge faster
% Target BLER ~10% (0.1): StepUp/StepDown ratio should be ~9
linkAdaptationConfig = struct(...
    'InitialOffset', 10, ...   % Reduced from 12 (start more conservative)
    'StepUp', 0.2, ...        % Reduced from 1.0 (increase MCS slower on ACK)
    'StepDown', 0.1);         % Increased from 0.02 (decrease MCS faster on NACK)       

% configureScheduler(gNB, ...
%     'Scheduler', drlScheduler, ...
%     'ResourceAllocationType', schedulerConfig.ResourceAllocationType, ...
%     'MaxNumUsersPerTTI', schedulerConfig.MaxNumUsersPerTTI, ...
%     'PFSWindowSize', schedulerConfig.PFSWindowSize, ...
%     'MUMIMOConfigDL', muMIMOStruct, ...
%     'LinkAdaptationConfigDL', linkAdaptationConfig, ...
%     'CSIMeasurementSignalDL', schedulerConfig.SignalType);

    configureScheduler(gNB, ...
    'Scheduler', drlScheduler, ...
    'ResourceAllocationType', schedulerConfig.ResourceAllocationType, ...
    'MaxNumUsersPerTTI', schedulerConfig.MaxNumUsersPerTTI, ...
    'PFSWindowSize', schedulerConfig.PFSWindowSize, ...
    'MUMIMOConfigDL', muMIMOStruct, ...
    'LinkAdaptationConfigDL', linkAdaptationConfig, ...
    'CSIMeasurementSignalDL', schedulerConfig.SignalType, ...
    'RVSequence', schedulerConfig.RVSequence);
% NOTE: SchedulerStrategy=1 bị xoá — nó trigger updateUEsServedDataRate của
% MATLAB internal (nrUEContext) trong MU-MIMO mode → crash "Matrix dimensions
% must agree". EMA tracking được xử lý hoàn toàn bên trong TrainingTputEMA
% của nrDRLScheduler (cùng công thức PFS, không phụ thuộc internal path).

% --- Create UEs ---
UEs = nrUE.empty(0, ueConfig.NumUEs); 
rng(42); 

ueAzimuths = ueConfig.AzimuthRange(1) + (ueConfig.AzimuthRange(2) - ueConfig.AzimuthRange(1)) * rand(ueConfig.NumUEs, 1);
ueElevations = zeros(ueConfig.NumUEs, 1);
ueDistances = ueConfig.MinDistance + (ueConfig.MaxDistance - ueConfig.MinDistance) * rand(ueConfig.NumUEs, 1);

[xPos, yPos, zPos] = sph2cart(deg2rad(ueAzimuths), deg2rad(ueElevations), ueDistances);
uePositions = [xPos yPos zPos] + gNBConfig.Position;

fprintf("UE distances: %s\n",mat2str(ueDistances))
fprintf('Khoi tao %d UEs (gNB: 32T32R, Ptx: %ddBm)...\n', ueConfig.NumUEs, gNBConfig.TransmitPower);
for i = 1:ueConfig.NumUEs
    currentNoise = ueConfig.NoiseFigureMin + (ueConfig.NoiseFigureMax - ueConfig.NoiseFigureMin) * rand();
    UEs(i) = nrUE('Name', "UE-" + string(i), ...
                  'Position', uePositions(i, :), ...
                  'NumReceiveAntennas', ueConfig.NumReceiveAntennas, ... 
                  'NoiseFigure', currentNoise, ... 
                  'ReceiveGain', ueConfig.ReceiveGain, ...
                  'NumTransmitAntennas',ueConfig.NumTransmitAntennas);         
    UEs(i).MACEntity.HARQDebug = true;  
end

% numFullBuffer = floor(ueConfig.NumUEs / 2);
numFullBuffer = 0;
numFTP3 = ueConfig.NumUEs - numFullBuffer;

% fullBufferUEs = UEs(1:numFullBuffer);
ftp3UEs       = UEs(numFullBuffer+1:end);

% Group 1: Full Buffer DL
% connectUE(gNB, fullBufferUEs, FullBufferTraffic="DL", CSIReportPeriodicity=10);

% Group 2: FTP Model 3 — no full buffer, custom traffic source
connectUE(gNB, ftp3UEs, FullBufferTraffic="off", CSIReportPeriodicity=10);

% FTP Model 3: 1.5 kB packets at 500 packets/s = 6 Mbps per UE
% Use networkTrafficOnOff (always-on) — networkTrafficFTP is for large file transfers
for i = 1:numel(ftp3UEs)
    ftpSrc = networkTrafficOnOff;
    ftpSrc.PacketSize = 1500;   % bytes (1.5 kB)
    ftpSrc.DataRate   = 6e6;    % bps = 1500 * 8 * 500
    ftpSrc.OnTime     = 1e9;    % effectively always ON (no off periods)
    ftpSrc.OffTime    = 0;      % no silence gaps
    addTrafficSource(gNB, ftpSrc, 'DestinationNode', ftp3UEs(i));
end

% connectUE(gNB, UEs, FullBufferTraffic="on", ...
%     CSIReportPeriodicity=10, CSIRSConfig=gNB.CSIRSConfiguration, ...  % Reduced to match CSI-RS period
%     CustomContext=csiReportConfig);

% Cấp UE objects cho DRL scheduler để đọc MAC.ReceivedBytes thực tế
drlScheduler.UEList = UEs;

% % === MIXED TRAFFIC SCENARIO ===
% % UE 1-6: Video streaming (high data rate, bursty)
% for i = 1:6
%     traffic = networkTrafficVideoConference('HasJitter', true);
%     addTrafficSource(gNB, traffic, 'DestinationNode', UEs(i));
% end
% 
% % UE 7-10: VoIP (low latency, small packets)
% for i = 7:10
%     traffic = networkTrafficVoIP;
%     addTrafficSource(gNB, traffic, 'DestinationNode', UEs(i));
% end
% 
% % UE 11-13: FTP (bursty file transfer)
% for i = 11:13
%     traffic = networkTrafficFTP;
%     addTrafficSource(gNB, traffic, 'DestinationNode', UEs(i));
% end
% 
% % UE 14-16: On-Off Application (web browsing pattern)
% for i = 14:16
%     traffic = networkTrafficOnOff( ...
%         'OnTime', 0.02, ...           % 20ms ON
%         'OffTime', 0.1, ...           % 100ms OFF  
%         'DataRate', 5e6, ...          % 5 Mbps when ON
%         'PacketSize', 1500);          % 1500 bytes/packet
%     addTrafficSource(gNB, traffic, 'DestinationNode', UEs(i));
% end
% 
% fprintf('Traffic configured: UE1-6 Video, UE7-10 VoIP, UE11-13 FTP, UE14-16 OnOff\n');

addNodes(networkSimulator, gNB);
addNodes(networkSimulator, UEs);

%% ====================== 3. CHANNEL MODEL ======================

cdlConfig = struct(...
    'DelayProfile', channelConfig.DelayProfile, ...
    'DelaySpread', channelConfig.DelaySpread, ...
    'MaximumDopplerShift', channelConfig.MaxDopplerShift, ...
    'TransmitArrayOrientation', channelConfig.Orientation);

% Tạo kênh truyền CDL
channels = hNRCreateCDLChannels(cdlConfig, gNB, UEs);
customChannelModel  = hNRCustomChannelModel(channels);
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);

%% ====================== 4. RUN SIMULATION ======================

if simConfig.EnableTraces
    simSchedulingLogger = helperNRSchedulingLogger(simConfig.NumFrameSimulation, gNB, UEs);
    simPhyLogger = helperNRPhyLogger(simConfig.NumFrameSimulation, gNB, UEs);
end

% Visualizer
metricsVisualizer = helperNRMetricsVisualizer(gNB, UEs, ...
    'RefreshRate', 1000, ... 
    'PlotSchedulerMetrics', true, ...
    'PlotPhyMetrics', false, ...
    'PlotCDFMetrics', true, ...
    'LinkDirection', 0);

simulationLogFile = "simulationLogs_128T128R"; 
simulationTime = simConfig.NumFrameSimulation * 1e-2;

fprintf('Dang chay mo phong 128T128R trong %.2f giay...\n', simulationTime);
run(networkSimulator, simulationTime);

% Notify Python that simulation is complete
drlScheduler.drlSendJSON(struct('type', 'STOP'));
%% ====================== 5. LOGS & METRICS ======================
displayPerformanceIndicators(metricsVisualizer);

if simConfig.EnableTraces
    simulationLogs = cell(1, 1);
    % Logic lấy log TDD/FDD
    if gNB.DuplexMode == "FDD"
        logInfo = struct('DLTimeStepLogs',[], 'ULTimeStepLogs',[], 'SchedulingAssignmentLogs',[], 'PhyReceptionLogs',[]);
        [logInfo.DLTimeStepLogs, logInfo.ULTimeStepLogs] = getSchedulingLogs(simSchedulingLogger);
    else 
        logInfo = struct('TimeStepLogs',[], 'SchedulingAssignmentLogs',[], 'PhyReceptionLogs',[]);
        logInfo.TimeStepLogs = getSchedulingLogs(simSchedulingLogger);
    end
   
    logInfo.SchedulingAssignmentLogs = getGrantLogs(simSchedulingLogger);
    logInfo.PhyReceptionLogs = getReceptionLogs(simPhyLogger);
    save(simulationLogFile, "simulationLogs");

    % Export actual DL allocation maps (post-constraint, includes reTx + newTx)
    allocMapDir = fullfile(scriptDir, "logs_new", "allocation_maps");
    % Map linear slot index -> Training TTI (0-based) for naming
    ttiBySlot = [];
    if isprop(drlScheduler, 'TrainingTTISlotIndex') && ~isempty(drlScheduler.TrainingTTISlotIndex)
        maxSlotLinear = max(drlScheduler.TrainingTTISlotIndex);
        ttiBySlot = -1 * ones(maxSlotLinear + 1, 1);
        for ttiIdx = 1:numel(drlScheduler.TrainingTTISlotIndex)
            slotLinear = drlScheduler.TrainingTTISlotIndex(ttiIdx);
            if slotLinear >= 0
                ttiBySlot(slotLinear + 1) = ttiIdx - 1; % 0-based TTI index
            end
        end
    end
    numAllocMaps = exportDLAllocationMaps(logInfo, gNBConfig.DuplexMode, ...
        schedulerConfig.ResourceAllocationType, muMIMOConfig.MaxNumLayers, allocMapDir, ttiBySlot);
    fprintf('Saved %d DL allocation maps to %s\n', numAllocMaps, allocMapDir);
    save("logInfo.mat","logInfo");
    % Plot Histrogram UE/RB
    avgNumUEsPerRB = calculateAvgUEsPerRBDL(logInfo, gNB.NumResourceBlocks, ...
        schedulerConfig.ResourceAllocationType, gNBConfig.DuplexMode);
    
    figure; theme("light");
    histogram(avgNumUEsPerRB, 'BinWidth', 0.1);
    title('Distribution of Avg UEs per RB (128T128R Configuration)');
    xlabel('Average Number of UEs per RB');
    ylabel('Frequency');
    grid on;
    
    % Plot UE Pairing Statistics (how many UEs paired per RBG)
    [pairingCounts, pairingDistribution] = calculateUEPairingStats(logInfo, ...
        gNB.NumResourceBlocks, schedulerConfig.ResourceAllocationType, gNBConfig.DuplexMode);
    
    figure; theme("light");
    bar(pairingCounts(:,1), pairingCounts(:,2));
    title('UE Pairing Distribution per RBG');
    xlabel('Number of UEs Paired on RBG');
    ylabel('Count (RBG-slot occurrences)');
    xticks(0:max(pairingCounts(:,1)));
    grid on;
    
    % Add text labels on bars
    for i = 1:size(pairingCounts, 1)
        text(pairingCounts(i,1), pairingCounts(i,2), num2str(pairingCounts(i,2)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    end
    
    % Print summary
    fprintf('\n=== UE PAIRING STATISTICS ===\n');
    totalRBGSlots = sum(pairingCounts(:,2));
    for i = 1:size(pairingCounts, 1)
        numUEs = pairingCounts(i,1);
        count = pairingCounts(i,2);
        pct = 100 * count / totalRBGSlots;
        if numUEs == 0
            fprintf('  Empty RBGs: %d (%.1f%%)\n', count, pct);
        elseif numUEs == 1
            fprintf('  Single UE (no pairing): %d (%.1f%%)\n', count, pct);
        else
            fprintf('  %d UEs paired: %d (%.1f%%)\n', numUEs, count, pct);
        end
    end
    
    % Plot Number of UEs allocated for new Tx per slot
    numNewTxUEsPerSlot = calculateNewTxUEsPerSlot(logInfo, gNBConfig.DuplexMode);
    
    figure; theme("light");
    plot(1:length(numNewTxUEsPerSlot), numNewTxUEsPerSlot, '-o', 'LineWidth', 1.5, 'MarkerSize', 4);
    title('Number of UEs Allocated for New Tx per DL Slot');
    xlabel('DL Slot Index');
    ylabel('Number of UEs');
    grid on;
    
    if ~isempty(numNewTxUEsPerSlot)
        fprintf('\nAverage UEs allocated for new Tx per DL slot: %.2f\n', mean(numNewTxUEsPerSlot));
    end
end

diary("off");

%% ====================== HELPER FUNCTION ======================
function avgUEsPerRB = calculateAvgUEsPerRBDL(logInfo, numResourceBlocks, ratType, duplexMode)
    if strcmp(duplexMode, 'TDD')
        timeStepLogs = logInfo.TimeStepLogs;
        freqAllocations = timeStepLogs(:, 5);
    elseif strcmp(duplexMode, 'FDD')
        timeStepLogs = logInfo.DLTimeStepLogs;
        freqAllocations = timeStepLogs(:, 4);
    end
    numOfSlots = size(timeStepLogs, 1) - 1;
    if ~ratType
        numRBG = size(freqAllocations{2}, 2);
        P = ceil(numResourceBlocks / numRBG);
        numRBsPerRBG = P * ones(1, numRBG);
        if mod(numResourceBlocks, P) > 0, numRBsPerRBG(end) = mod(numResourceBlocks, P); end
    end
    avgUEsPerRB = zeros(1, numOfSlots);
    for slotIdx = 1:numOfSlots
        if strcmp(duplexMode, 'TDD')
            slotType = timeStepLogs{slotIdx + 1, 4};
            if ~strcmp(slotType, 'DL'), continue; end
        end
        freqAllocation = freqAllocations{slotIdx + 1};
        if ~ratType
            totalUniqueUEs = sum(arrayfun(@(rbgIdx) nnz(freqAllocation(:, rbgIdx) > 0) * numRBsPerRBG(rbgIdx), 1:length(numRBsPerRBG)));
            avgUEsPerRB(slotIdx) = totalUniqueUEs / numResourceBlocks;
        else
            ueRBUsage = zeros(1, numResourceBlocks);
            for ueIdx = 1:size(freqAllocation, 1)
                startRB = freqAllocation(ueIdx, 1);
                ueRBUsage(startRB + 1:(startRB + freqAllocation(ueIdx, 2))) = ueRBUsage(startRB + 1:(startRB + freqAllocation(ueIdx, 2))) + 1;
            end
            avgUEsPerRB(slotIdx) = mean(ueRBUsage(ueRBUsage > 0));
        end
    end
    avgUEsPerRB = avgUEsPerRB(avgUEsPerRB > 0);
end

function [pairingCounts, pairingDistribution] = calculateUEPairingStats(logInfo, numResourceBlocks, ratType, duplexMode)
    % Calculate UE pairing statistics per RBG
    % Returns:
    %   pairingCounts: Nx2 matrix [numUEs, count] - histogram of how many UEs paired
    %   pairingDistribution: cell array with per-slot pairing info
    
    if strcmp(duplexMode, 'TDD')
        timeStepLogs = logInfo.TimeStepLogs;
        freqAllocations = timeStepLogs(:, 5);
    elseif strcmp(duplexMode, 'FDD')
        timeStepLogs = logInfo.DLTimeStepLogs;
        freqAllocations = timeStepLogs(:, 4);
    end
    
    numOfSlots = size(timeStepLogs, 1) - 1;
    
    % Determine number of RBGs (for RAT0)
    if ~ratType && numOfSlots > 0
        numRBG = size(freqAllocations{2}, 2);
    else
        numRBG = numResourceBlocks; % RAT1: use RBs directly
    end
    
    % Count pairing occurrences: pairingMap(numUEs+1) = count
    % Index 1 = 0 UEs, Index 2 = 1 UE, Index 3 = 2 UEs, etc.
    maxPossibleUEs = 16; % Maximum UEs that can be paired (MU-MIMO limit)
    pairingMap = zeros(1, maxPossibleUEs + 1);
    
    pairingDistribution = cell(numOfSlots, 1);
    
    for slotIdx = 1:numOfSlots
        % Skip non-DL slots in TDD
        if strcmp(duplexMode, 'TDD')
            slotType = timeStepLogs{slotIdx + 1, 4};
            if ~strcmp(slotType, 'DL'), continue; end
        end
        
        freqAllocation = freqAllocations{slotIdx + 1};
        
        if ~ratType
            % RAT0: freqAllocation is [numUEs x numRBGs] binary matrix
            slotPairing = zeros(1, numRBG);
            for rbgIdx = 1:numRBG
                % Count unique UEs on this RBG (non-zero entries in column)
                numUEsOnRBG = nnz(freqAllocation(:, rbgIdx) > 0);
                slotPairing(rbgIdx) = numUEsOnRBG;
                pairingMap(numUEsOnRBG + 1) = pairingMap(numUEsOnRBG + 1) + 1;
            end
        else
            % RAT1: freqAllocation is [numUEs x 2] with [startRB, numRBs]
            % Need to count UEs per RB then aggregate
            uePerRB = zeros(1, numResourceBlocks);
            for ueIdx = 1:size(freqAllocation, 1)
                startRB = freqAllocation(ueIdx, 1);
                numRBs = freqAllocation(ueIdx, 2);
                if numRBs > 0
                    uePerRB(startRB + 1 : startRB + numRBs) = ...
                        uePerRB(startRB + 1 : startRB + numRBs) + 1;
                end
            end
            slotPairing = uePerRB;
            % Count pairing per RB for RAT1
            for rb = 1:numResourceBlocks
                numUEsOnRB = uePerRB(rb);
                pairingMap(numUEsOnRB + 1) = pairingMap(numUEsOnRB + 1) + 1;
            end
        end
        
        pairingDistribution{slotIdx} = slotPairing;
    end
    
    % Convert to Nx2 format [numUEs, count], only include non-zero counts
    validIdx = find(pairingMap > 0);
    pairingCounts = zeros(length(validIdx), 2);
    for i = 1:length(validIdx)
        pairingCounts(i, 1) = validIdx(i) - 1;  % numUEs (0-indexed in map)
        pairingCounts(i, 2) = pairingMap(validIdx(i));
    end
end

function numSaved = exportDLAllocationMaps(logInfo, duplexMode, ratType, maxNumLayers, outDir, ttiBySlot)
    % Export per-slot DL allocation heatmaps [Layer x RBG] from final scheduler output.
    % This reflects actual resources passed downstream (reTx + newTx combined).
    numSaved = 0;
    if ratType ~= 0
        warning('exportDLAllocationMaps currently supports RAT-0 only.');
        return;
    end

    if strcmp(duplexMode, 'TDD')
        timeStepLogs = logInfo.TimeStepLogs;
        freqAllocCol = 5;
        slotTypeCol = 4;
    elseif strcmp(duplexMode, 'FDD')
        timeStepLogs = logInfo.DLTimeStepLogs;
        freqAllocCol = 4;
        slotTypeCol = [];
    else
        warning('Unsupported duplex mode: %s', duplexMode);
        return;
    end

    if size(timeStepLogs, 1) <= 1
        return;
    end

    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    txTypeCol = findLogColumn(timeStepLogs, 'Tx Type', freqAllocCol + 4);
    signalTypeCol = findLogColumn(timeStepLogs, 'Signal Type', -1);
    freqAllocations = timeStepLogs(:, freqAllocCol);
    allocationSnapshots = struct('slotIndex', {}, 'ttiId', {}, 'allocationMatrix', {}, ...
        'txTagMatrix', {}, 'freqAllocation', {}, 'txTypePerUE', {});

    numSlots = size(timeStepLogs, 1) - 1;
    for slotIdx = 1:numSlots
        if signalTypeCol > 0
            signalType = timeStepLogs{slotIdx + 1, signalTypeCol};
            if strcmp(signalType, "CSIRS")
                continue;
            end
        end

        if strcmp(duplexMode, 'TDD')
            slotType = timeStepLogs{slotIdx + 1, slotTypeCol};
            % Include both DL and Special (S) slots
            if ~strcmp(slotType, 'DL') && ~strcmp(slotType, 'S')
                continue;
            end
        end

        freqAllocation = freqAllocations{slotIdx + 1};
        if isempty(freqAllocation)
            continue;
        end

        txTypePerUE = {};
        if txTypeCol <= size(timeStepLogs, 2)
            txTypePerUE = timeStepLogs{slotIdx + 1, txTypeCol};
        end

        numRBG = size(freqAllocation, 2);
        numUEs = size(freqAllocation, 1);
        allocMap = zeros(maxNumLayers, numRBG);
        allocTxTag = repmat({''}, maxNumLayers, numRBG);

        for rbgIdx = 1:numRBG
            scheduledUEs = find(freqAllocation(:, rbgIdx) > 0);
            if isempty(scheduledUEs)
                continue;
            end
            numFill = min(numel(scheduledUEs), maxNumLayers);
            allocMap(1:numFill, rbgIdx) = scheduledUEs(1:numFill);
            for layerIdx = 1:numFill
                ueIdx = scheduledUEs(layerIdx);
                allocTxTag{layerIdx, rbgIdx} = normalizeTxTag(txTypePerUE, ueIdx);
            end
        end

        fig = figure('Visible', 'off');
        colorMap = pythonLikeDiscretePalette(numUEs);
        imagesc(allocMap.' + 1, [1 numUEs + 1]);
        axis tight;
        colormap(colorMap);
        cb = colorbar;
        cb.Label.String = 'UE Index';
        cb.Ticks = 1:(numUEs + 1);
        cb.TickLabels = string(0:numUEs);
        xlabel('Layer Index');
        ylabel('RBG Index');
        title(sprintf('MATLAB DL Allocation Map - slot %d', slotIdx - 1));
        if maxNumLayers > 0
            xticks(1:maxNumLayers);
        end
        if numRBG > 0
            yticks(1:numRBG);
        end
        set(gca, 'YDir', 'normal', 'GridAlpha', 0.35, 'GridColor', [0.2 0.2 0.2]);
        grid on;

        % Annotate each occupied layer/RBG with UE and tx type.
        for layerIdx = 1:maxNumLayers
            for rbgIdx = 1:numRBG
                ueIdx = allocMap(layerIdx, rbgIdx);
                if ueIdx <= 0
                    continue;
                end
                txTag = allocTxTag{layerIdx, rbgIdx};
                if isempty(txTag)
                    txTag = 'newTx';
                end
                cellColor = colorMap(ueIdx + 1, :);
                if sum(cellColor) > 1.7
                    txtColor = [0 0 0];
                else
                    txtColor = [1 1 1];
                end
                text(layerIdx, rbgIdx, {sprintf('UE%d', ueIdx), txTag}, ...
                    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                    'FontSize', 8, 'FontWeight', 'bold', 'Color', txtColor, 'Rotation', 0, 'Interpreter', 'none');
            end
        end

        slotLinear = slotIdx - 1;
        ttiId = -1;
        if exist('ttiBySlot', 'var') && ~isempty(ttiBySlot)
            if slotLinear + 1 <= numel(ttiBySlot)
                ttiId = ttiBySlot(slotLinear + 1);
            end
        end
        if ttiId >= 0
            outFile = fullfile(outDir, sprintf('dl_alloc_tti_%05d.png', ttiId));
        else
            outFile = fullfile(outDir, sprintf('dl_alloc_slot_%05d.png', slotLinear));
        end
        exportgraphics(fig, outFile, 'Resolution', 120);
        close(fig);

        slotIndex = slotLinear;
        outMat = fullfile(outDir, sprintf('dl_alloc_slot_%05d.mat', slotIndex));
        save(outMat, 'slotIndex', 'allocMap', 'allocTxTag', 'freqAllocation', 'txTypePerUE', 'ttiId');

        allocationSnapshots(end+1) = struct( ... %#ok<AGROW>
            'slotIndex', slotIndex, ...
            'ttiId', ttiId, ...
            'allocationMatrix', allocMap, ...
            'txTagMatrix', {allocTxTag}, ...
            'freqAllocation', freqAllocation, ...
            'txTypePerUE', {txTypePerUE});

        numSaved = numSaved + 1;
    end

    if ~isempty(allocationSnapshots)
        save(fullfile(outDir, 'dl_allocation_all_slots.mat'), 'allocationSnapshots');
    end
end

function colIdx = findLogColumn(timeStepLogs, colName, fallbackIdx)
    colIdx = fallbackIdx;
    if isempty(timeStepLogs)
        return;
    end
    headers = timeStepLogs(1, :);
    idx = find(strcmp(headers, colName), 1);
    if ~isempty(idx)
        colIdx = idx;
    end
end

function txTag = normalizeTxTag(txTypePerUE, ueIdx)
    txTag = 'newTx';
    if isempty(txTypePerUE) || ueIdx > numel(txTypePerUE) || ueIdx < 1
        return;
    end

    entry = txTypePerUE{ueIdx};
    if iscell(entry) && ~isempty(entry)
        entry = entry{1};
    end
    if isstring(entry)
        entry = char(entry);
    end
    if isempty(entry)
        return;
    end

    entryLower = lower(string(entry));
    if contains(entryLower, "retx")
        txTag = 'reTx';
    elseif contains(entryLower, "newtx")
        txTag = 'newTx';
    else
        txTag = char(entry);
    end
end

function cmap = pythonLikeDiscretePalette(numUEs)
    % Python-like categorical colors (matplotlib tab20), with index 0 reserved for empty.
    tab20 = [ ...
        31,119,180; 174,199,232; 255,127,14; 255,187,120; ...
        44,160,44; 152,223,138; 214,39,40; 255,152,150; ...
        148,103,189; 197,176,213; 140,86,75; 196,156,148; ...
        227,119,194; 247,182,210; 127,127,127; 199,199,199; ...
        188,189,34; 219,219,141; 23,190,207; 158,218,229] / 255;

    cmap = zeros(numUEs + 1, 3);
    cmap(1, :) = [0.94 0.94 0.94];
    for ueIdx = 1:numUEs
        cmap(ueIdx + 1, :) = tab20(mod(ueIdx - 1, size(tab20, 1)) + 1, :);
    end
end

function numNewTxUEs = calculateNewTxUEsPerSlot(logInfo, duplexMode)
    if strcmp(duplexMode, 'TDD')
        timeStepLogs = logInfo.TimeStepLogs;
        slotTypeCol = findLogColumn(timeStepLogs, 'Type', 4);
    elseif strcmp(duplexMode, 'FDD')
        timeStepLogs = logInfo.DLTimeStepLogs;
        slotTypeCol = [];
    end
    
    txTypeCol = findLogColumn(timeStepLogs, 'Tx Type', -1);
    signalTypeCol = findLogColumn(timeStepLogs, 'Signal Type', -1);
    if txTypeCol == -1
        numNewTxUEs = [];
        return;
    end
    
    numOfSlots = size(timeStepLogs, 1) - 1;
    numNewTxUEs = zeros(1, numOfSlots);
    isDLSlot = true(1, numOfSlots);
    
    for slotIdx = 1:numOfSlots
        if signalTypeCol > 0
            signalType = timeStepLogs{slotIdx + 1, signalTypeCol};
            if strcmp(signalType, "CSIRS")
                isDLSlot(slotIdx) = false;
                continue;
            end
        end
        
        if strcmp(duplexMode, 'TDD')
            slotType = timeStepLogs{slotIdx + 1, slotTypeCol};
            if ~strcmp(slotType, 'DL') && ~strcmp(slotType, 'S')
                isDLSlot(slotIdx) = false;
                continue;
            end
        end
        
        txTypes = timeStepLogs{slotIdx + 1, txTypeCol};
        if isempty(txTypes)
            continue;
        end
        
        newTxCount = 0;
        for ueIdx = 1:numel(txTypes)
            entry = txTypes{ueIdx};
            if iscell(entry) && ~isempty(entry), entry = entry{1}; end
            if isstring(entry), entry = char(entry); end
            if ischar(entry) && contains(lower(entry), 'newtx')
                newTxCount = newTxCount + 1;
            end
        end
        numNewTxUEs(slotIdx) = newTxCount;
    end
    
    if strcmp(duplexMode, 'TDD')
        numNewTxUEs = numNewTxUEs(isDLSlot);
    end
end
