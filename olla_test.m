%% olla_test.m — Fast OLLA verification (4T4R, 4 UEs, Wideband, 10 frames)
%
% Goal: confirm OLLA fix (LinkAdaptationConfigDL) drives BLER from ~60% → ~10%.
% Strategy: use 4T instead of 128T so nrRISelect runs on 4×4 matrix (32× faster).
% OLLA logic is identical regardless of antenna count — so this is a valid proxy.
%
% Runtime estimate: ~5–10 min (vs ~8 h for 128T 64UE 50 frame).

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projectRoot, 'wirelessnetwork')), '-begin');
addpath(genpath(fullfile(projectRoot, '5g')),              '-begin');
wirelessnetworkSupportPackageCheck

rng("default")
numFrameSimulation = 10;
numUEs             = 32;   % overloaded MU-MIMO → inter-user interference → BLER > 0
pmiCQIMode         = "Wideband";   % avoids 6× subband LQM calls

networkSimulator = wirelessNetworkSimulator.init;

gNB = nrGNB( ...
    Position             = [0 0 30], ...
    TransmitPower        = 60, ...
    CarrierFrequency     = 4.9e9, ...
    ChannelBandwidth     = 10e6, ...
    SubcarrierSpacing    = 30e3, ...
    NumResourceBlocks    = 24, ...
    NumTransmitAntennas  = 4, ...   % 4T instead of 128T — 32× faster PMI
    NumReceiveAntennas   = 4, ...
    DuplexMode           = "TDD", ...
    ReceiveGain          = 11, ...
    SRSPeriodicityUE     = 20);

muMIMOConfiguration = struct( ...
    MaxNumUsersPaired       = 12, ...  % same as 128T config → heavy MU-MIMO interference
    MaxNumLayers            = 8, ...
    MinNumRBs               = 2, ...
    SemiOrthogonalityFactor = 0.5, ...
    MinCQI                  = 1);

% ── OLLA config (the fix being tested) ──────────────────────────────────
% TargetBLER = StepDown / (StepDown + StepUp) = 0.03 / 0.30 = 10%
oLLAConfig = struct('InitialOffset', 0, 'StepUp', 0.27, 'StepDown', 0.03);

configureScheduler(gNB, ...
    ResourceAllocationType  = 0, ...
    MaxNumUsersPerTTI       = 32, ...
    MUMIMOConfigDL          = muMIMOConfiguration, ...
    CSIMeasurementSignalDL  = "CSI-RS", ...
    LinkAdaptationConfigDL  = oLLAConfig);   % ← the fix

rMin = 50; rMax = 500;
azDeg = -60 + 120 .* rand(numUEs, 1);
rDist = sqrt(rMin^2 + (rMax^2 - rMin^2) .* rand(numUEs, 1));
[xPos, yPos, zPos] = sph2cart(deg2rad(azDeg), zeros(numUEs,1), rDist);

UEs = nrUE( ...
    Name                = "UE-" + (1:numUEs), ...
    Position            = [xPos, yPos, zPos] + [0 0 30], ...
    ReceiveGain         = 0, ...
    NumTransmitAntennas = 4, ...
    NumReceiveAntennas  = 4);

gNB.PMICQIMode = pmiCQIMode;
connectUE(gNB, UEs, FullBufferTraffic="DL", CSIReportPeriodicity=10);

addNodes(networkSimulator, gNB);
addNodes(networkSimulator, UEs);

channelConfig    = struct(DelayProfile="CDL-B", DelaySpread=100e-9, MaximumDopplerShift=5);
channels         = hNRCreateCDLChannels(channelConfig, gNB, UEs);
customChannelModel = hNRCustomChannelModel(channels);
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);

simSchedulingLogger = helperNRSchedulingLogger(numFrameSimulation, gNB, UEs);
simPhyLogger        = helperNRPhyLogger(numFrameSimulation, gNB, UEs);

fprintf('\n=== OLLA test: %dT, %d UEs, %s, %d frames ===\n\n', ...
    gNB.NumTransmitAntennas, numUEs, pmiCQIMode, numFrameSimulation);

tic
run(networkSimulator, numFrameSimulation * 1e-2);
elapsed = toc;
fprintf('\nSimulation time: %.1f s\n', elapsed);

%% BLER per UE
totalSlots = numFrameSimulation * (10 * gNB.SubcarrierSpacing / 15e3);
dlMetrics  = getPhyMetrics(simPhyLogger, 1, totalSlots, 1:numUEs);
dlMetrics  = dlMetrics(:, 1);

fprintf('\n--- OLLA check (target BLER ~0.10) ---\n');
blerAll = zeros(1, numUEs);
for u = 1:numUEs
    pkts = dlMetrics(u).TotalPackets;
    fail = dlMetrics(u).DecodeFailures;
    bler = fail / max(pkts, 1);
    blerAll(u) = bler;
    tag = '';
    if bler > 0.30, tag = '  ← HIGH (OLLA not working?)'; end
    if bler < 0.15, tag = '  ← OK';  end
    fprintf('  UE-%d  BLER=%.3f  (%d fail / %d pkts)%s\n', u, bler, fail, pkts, tag);
end
fprintf('  Mean BLER = %.3f  (expect ~0.10–0.20 after 10 frames)\n', mean(blerAll));

%% MCS summary
colMap   = simSchedulingLogger.GrantLogsColumnIndexMap;
grantRaw = getGrantLogs(simSchedulingLogger);
grantRaw = grantRaw(2:end, :);
isDL     = strcmp(grantRaw(:, colMap('Grant Type')), 'DL');
dlData   = grantRaw(isDL, :);

fprintf('\n--- MCS summary (expect avg < 25 with OLLA reducing high-MCS grants) ---\n');
fprintf('  %-6s  %7s  %8s  %8s  %8s\n', 'UE','Grants','MCS avg','MCS min','MCS max');
for u = 1:numUEs
    mask = cell2mat(dlData(:, colMap('RNTI'))) == u;
    if ~any(mask); continue; end
    mcsV = cell2mat(dlData(mask, colMap('MCS Index')));
    fprintf('  UE-%-3d  %7d  %8.1f  %8d  %8d\n', u, sum(mask), mean(mcsV), min(mcsV), max(mcsV));
end
