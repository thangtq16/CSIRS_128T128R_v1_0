function [rxGrid_s0, rxGrid_s1, pathGains, sampleTimes, offset, ofdmInfo] = ...
    csirs_txRx(carrier, txGrid_2slots, allCsirsInd, allCsirsSym, channel, SNRdB, nRxAntennas)
%CSIRS_TXRX  OFDM modulate, pass through CDL channel + AWGN, demodulate.
%
%  Inputs:
%    carrier        - nrCarrierConfig
%    txGrid_2slots  - [K x 28 x nTx] 2-slot TX resource grid
%    allCsirsInd    - {1x4} CSI-RS indices (for timing estimation)
%    allCsirsSym    - {1x4} CSI-RS symbols  (for timing estimation)
%    channel        - nrCDLChannel object (caller must release() before call)
%    SNRdB          - SNR in dB
%    nRxAntennas    - number of UE receive antennas
%
%  Outputs:
%    rxGrid_s0   - [K x 14 x nRx] demodulated slot 0
%    rxGrid_s1   - [K x 14 x nRx] demodulated slot 1
%    pathGains   - channel path gains (for perfect CE in NMSE analysis)
%    sampleTimes - sample times (for perfect CE)
%    offset      - timing offset (samples)
%    ofdmInfo    - struct from nrOFDMInfo (Nfft etc.)

% ── OFDM modulate full 2-slot grid ───────────────────────────────────────
carrier.NSlot = 0;
[txWaveform, ofdmInfo] = nrOFDMModulate(carrier, txGrid_2slots);

% ── Channel ──────────────────────────────────────────────────────────────
chInfo    = info(channel);
txWaveform = [txWaveform; zeros(chInfo.MaximumChannelDelay, size(txWaveform,2))];

[rxWaveform, pathGains, sampleTimes] = channel(txWaveform);

% ── AWGN ─────────────────────────────────────────────────────────────────
SNR = 10^(SNRdB/10);
N0  = 1/sqrt(2.0 * nRxAntennas * double(ofdmInfo.Nfft) * SNR);
rxWaveform = rxWaveform + N0 * complex(randn(size(rxWaveform)), randn(size(rxWaveform)));

% ── Timing estimation (using Resource #0, Slot 0) ────────────────────────
carrier.NSlot = 0;
offset     = nrTimingEstimate(carrier, rxWaveform, allCsirsInd{1}, allCsirsSym{1});
rxWaveform = rxWaveform(1+offset:end, :);

% ── OFDM demodulate full 2-slot waveform ─────────────────────────────────
carrier.NSlot = 0;
rxGrid_2slots = nrOFDMDemodulate(carrier, rxWaveform);

nSymPerSlot = carrier.SymbolsPerSlot;  % 14
rxGrid_s0   = rxGrid_2slots(:, 1:nSymPerSlot,              :);
rxGrid_s1   = rxGrid_2slots(:, (nSymPerSlot+1):2*nSymPerSlot, :);

end
