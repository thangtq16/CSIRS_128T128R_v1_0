%% 128T128R NZP-CSI-RS — SNR Sweep: RI / MCS / Throughput Average
%
%  Compares CSI feedback approaches B / C / D / E over multiple SNR points
%  (representing cell radius: far / medium / near) and channel realizations.
%
%  Approaches:
%    B — Full 128-port SVD upper bound          (ideal reference)
%    C — Rel-19 Type I Single-Panel Mode A      (TS 38.214 S5.2.2.2.1a)
%    D — Rel-19 Type I Single-Panel Mode B      (TS 38.214 S5.2.2.2.1a)
%    E — Rel-19 Refined eTypeII 128-port        (TS 38.214 S5.2.2.2.5a)
%        ThangTQ23_128T128R_eTypeII_Rel19
%
%  References:
%    TS 38.211 S7.4.1.5    (CSI-RS signal generation)
%    TS 38.214 S5.2.2.2.1a (Refined Type I Single Panel, Rel-19)
%    TS 38.214 S5.2.2.2.5a (Refined eTypeII, Rel-19)
%
%  Author: ThangTQ23 - VSI
%  Date:   2026-03

clear; close all; clc;

%% ── CUSTOM LIBRARY PATH ──────────────────────────────────────────────────
addpath(genpath(fullfile(pwd, '5g')));
addpath(genpath(fullfile(pwd, 'wirelessnetwork')));
rehash toolboxcache;

%% ── SECTION 1: SYSTEM CONFIG (fixed — outside all loops) ────────────────

% Carrier
carrier = nrCarrierConfig;
carrier.NSizeGrid         = 52;    % 10 MHz @ SCS 30 kHz (faster simulation)
carrier.SubcarrierSpacing = 30;    % kHz
carrier.NSlot             = 0;
carrier.CyclicPrefix      = 'normal';

% Antennas
nTxAntennas  = 128;
nRxAntennas  = 4;
gnbArraySize    = [4, 16, 2, 1, 1];   % 4V x 16H x 2pol = 128 ports
ueArraySize     = [1,  2, 2, 1, 1];   % 1V x  2H x 2pol =   4 ports
panelDimensions = [gnbArraySize(5), gnbArraySize(2), gnbArraySize(1)];  % [Ng, N1=Nh, N2=Nv] = [1,16,4]

% PMI / CQI mode selection
pmiMode = 'Subband';   % 'Subband' | 'Wideband'
cqiMode = 'Wideband';  % 'Wideband' | 'Subband'
%   Subband CQI: fb.cqi_C/D/E = [WB; SB1; SB2; ...] — finer granularity
%   Note: Approach D PMI is always wideband (Mode B beam search constraint),
%         but Approach D CQI follows cqiMode like Approach C.

% ThangTQ23_128T128R_eTypeII_Rel19: Approach E ParameterCombination
% Per TS 38.214 Table 5.2.2.2.5-1: 1-6 → L=2/4, 7-8 → L=6 (maxRank=2)
% PC=5: L=4, pv(v=1..4)=1/4, β=1/4 — balanced overhead, supports rank 1~4
paramCombE = 5;

% Channel parameters
channelCfg.DelayProfile        = 'CDL-B';   % near-uniform SV → rank diversity
% channelCfg.DelaySpread         = 450e-9;    % 450 ns (Urban Macro)
channelCfg.DelaySpread         = 100e-9;    % 100 ns (Urban Micro, Indoor) → better SNR after estimation → higher RI/MCS
% channelCfg.MaximumDopplerShift = 136;       % 136 Hz (30 km/h @ 4.9 GHz)
channelCfg.MaximumDopplerShift = 5;         % 5 Hz (1 km/h @ 4.9 GHz) → more coherent channel → better estimation → higher RI/MCS   
channelCfg.CarrierFrequency    = 4.9e9;     % 4.9 GHz

fprintf('=== 128T128R CSI-RS SNR Sweep (v2) ===\n');
fprintf('Carrier: %d PRBs, SCS %d kHz\n', carrier.NSizeGrid, carrier.SubcarrierSpacing);
fprintf('Channel: %s, DS=%.0fns, fD=%.0fHz, fc=%.1fGHz\n', ...
    channelCfg.DelayProfile, channelCfg.DelaySpread*1e9, ...
    channelCfg.MaximumDopplerShift, channelCfg.CarrierFrequency/1e9);

%% ── SECTION 2+3: BUILD TX GRID (one-time — reused for all realizations) ──
[csirs, txGrid_2slots, allCsirsInd, allCsirsSym, slotAssign, cdmLengths] = ...
    csirs_buildGrid(carrier, nTxAntennas);

%% ── SECTION 5: CHANNEL OBJECT (created once, release() per realization) ──
channel = csirs_channelSetup(carrier, gnbArraySize, ueArraySize, channelCfg);

%% ── SNR SWEEP ─────────────────────────────────────────────────────────────
%  3 points covering far / medium / near cell radius
snrList = [-5, 10, 25];    % dB
nRealiz = 20;              % channel realizations per SNR point

nSNR    = length(snrList);
results = struct();

fprintf('\nStarting sweep: %d SNR points x %d realizations each\n', nSNR, nRealiz);
fprintf('(Total runs: %d)\n\n', nSNR * nRealiz);

for iSNR = 1:nSNR
    SNRdB = snrList(iSNR);

    % Accumulate metrics over realizations: rows = realizations, cols = B C D E
    acc_ri  = zeros(nRealiz, 4);
    acc_mcs = zeros(nRealiz, 4);
    acc_tp  = zeros(nRealiz, 4);

    for iReal = 1:nRealiz
        release(channel);   % new channel realization

        % Section 6+7: TX → channel → AWGN → demod
        [rxGrid_s0, rxGrid_s1, ~, ~, ~, ~] = ...
            csirs_txRx(carrier, txGrid_2slots, allCsirsInd, allCsirsSym, ...
                       channel, SNRdB, nRxAntennas);

        % Section 8: channel estimation
        [H_est_full, nVar_all] = csirs_channelEstimate( ...
            carrier, rxGrid_s0, rxGrid_s1, allCsirsInd, allCsirsSym, ...
            slotAssign, cdmLengths, nTxAntennas, nRxAntennas);

        % Section 10: CSI feedback (B / C / D / E)
        % ThangTQ23_128T128R_eTypeII_Rel19: pass paramCombE for Approach E
        fb = csirs_feedback(carrier, csirs, H_est_full, nVar_all, slotAssign, pmiMode, cqiMode, panelDimensions, paramCombE);

        % RI + MCS + Throughput
        m = csirs_computeMetrics(fb, carrier);

        acc_ri(iReal,  :) = m.ri;
        acc_mcs(iReal, :) = m.mcs;
        acc_tp(iReal,  :) = m.tp;
    end

    % Average across realizations
    results(iSNR).snr = SNRdB;
    results(iSNR).ri  = mean(acc_ri,  1);
    results(iSNR).mcs = mean(acc_mcs, 1);
    results(iSNR).tp  = mean(acc_tp,  1);

    fprintf('  SNR=%+3ddB  |  RI: B=%.2f C=%.2f D=%.2f E=%.2f  |  TP(Mbps): B=%.1f C=%.1f D=%.1f E=%.1f\n', ...
        SNRdB, ...
        results(iSNR).ri(1),  results(iSNR).ri(2),  results(iSNR).ri(3),  results(iSNR).ri(4), ...
        results(iSNR).tp(1),  results(iSNR).tp(2),  results(iSNR).tp(3),  results(iSNR).tp(4));
end

%% ── OUTPUT TABLE ──────────────────────────────────────────────────────────
csirs_printSweepTable(results, snrList);
