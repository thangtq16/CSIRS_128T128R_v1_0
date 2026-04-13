%% quick_test.m — 5-frame smoke test for Bug #1 and Bug #2 fixes
% Bug #1: OLLA reset on every CSI update → BLER ~60%
% Bug #2: Subband SINR all identical → per-subband CRB filtering added

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projectRoot, 'wirelessnetwork')), '-begin');
addpath(genpath(fullfile(projectRoot, '5g')),              '-begin');
wirelessnetworkSupportPackageCheck

rng("default")
numFrameSimulation = 5;
pmiCQIMode = "Subband";

networkSimulator = wirelessNetworkSimulator.init;

gNB = nrGNB( ...
    Position             = [0 0 30], ...
    TransmitPower        = 60, ...
    CarrierFrequency     = 4.9e9, ...
    ChannelBandwidth     = 10e6, ...
    SubcarrierSpacing    = 30e3, ...
    NumResourceBlocks    = 24, ...
    NumTransmitAntennas  = 128, ...
    NumReceiveAntennas   = 128, ...
    DuplexMode           = "TDD", ...
    ReceiveGain          = 11, ...
    SRSPeriodicityUE     = 20);

muMIMOConfiguration = struct( ...
    MaxNumUsersPaired       = 12, ...
    MaxNumLayers            = 24, ...
    MinNumRBs               = 2, ...
    SemiOrthogonalityFactor = 0.5, ...
    MinCQI                  = 1);

configureScheduler(gNB, ...
    ResourceAllocationType = 0, ...
    MaxNumUsersPerTTI      = 32, ...
    MUMIMOConfigDL         = muMIMOConfiguration, ...
    CSIMeasurementSignalDL = "CSI-RS");

numUEs  = 8;   % small for speed
rMin = 50; rMax = 500;
azDeg = -60 + 120 .* rand(numUEs, 1);
rDist = sqrt(rMin^2 + (rMax^2 - rMin^2) .* rand(numUEs, 1));
[xPos, yPos, zPos] = sph2cart(deg2rad(azDeg), zeros(numUEs,1), rDist);
uePositions = [xPos, yPos, zPos] + [0 0 30];

UEs = nrUE( ...
    Name                = "UE-" + (1:numUEs), ...
    Position            = uePositions, ...
    ReceiveGain         = 0, ...
    NumTransmitAntennas = 4, ...
    NumReceiveAntennas  = 4);

gNB.PMICQIMode = pmiCQIMode;
connectUE(gNB, UEs, FullBufferTraffic="DL", CSIReportPeriodicity=10);

addNodes(networkSimulator, gNB);
addNodes(networkSimulator, UEs);

channelConfig = struct(DelayProfile="CDL-B", DelaySpread=100e-9, MaximumDopplerShift=5);
channels = hNRCreateCDLChannels(channelConfig, gNB, UEs);
customChannelModel = hNRCustomChannelModel(channels);
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);

enableTraces = true;
simSchedulingLogger = helperNRSchedulingLogger(numFrameSimulation, gNB, UEs);
simPhyLogger        = helperNRPhyLogger(numFrameSimulation, gNB, UEs);

fprintf('\n=== Quick test: %d frames, %d UEs, Subband PMI ===\n\n', numFrameSimulation, numUEs);
run(networkSimulator, numFrameSimulation * 1e-2);

%% BLER check — Bug #1
% getPhyMetrics returns struct array [numUEs x 2] with fields:
%   RNTI, TotalPackets, DecodeFailures
% Column 1 = DL, Column 2 = UL
totalSlots = numFrameSimulation * (10 * gNB.SubcarrierSpacing / 15e3);
dlMetrics = getPhyMetrics(simPhyLogger, 1, totalSlots, 1:numUEs);
dlMetrics = dlMetrics(:, 1);  % DL only

fprintf('\n--- Bug #1 check (BLER per UE, expect ~0.1 not ~0.6) ---\n');
for u = 1:numUEs
    pkts = dlMetrics(u).TotalPackets;
    fail = dlMetrics(u).DecodeFailures;
    if pkts > 0
        fprintf('  UE-%d  BLER=%.3f  (%d fail / %d pkts)\n', u, fail/pkts, fail, pkts);
    end
end

%% Subband SINR check — Bug #2 already visible in CSI lines above
fprintf('\n(Bug #2: see CSI lines above — SINR should differ across 6 subbands)\n');
