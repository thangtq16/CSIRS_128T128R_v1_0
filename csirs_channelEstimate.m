function [H_est_full, nVar_all] = csirs_channelEstimate( ...
    carrier, rxGrid_s0, rxGrid_s1, allCsirsInd, allCsirsSym, ...
    slotAssign, cdmLengths, nTxAntennas, nRxAntennas)
%CSIRS_CHANNELESTIMATE  Per-resource channel estimation → combined H_est_full.
%
%  Each of the 4 resources (32 ports each) is estimated in its own slot.
%  Results are assembled into a single [K x 28 x nRx x 128] matrix.
%
%  Inputs:
%    carrier      - nrCarrierConfig
%    rxGrid_s0    - [K x 14 x nRx] demodulated slot 0
%    rxGrid_s1    - [K x 14 x nRx] demodulated slot 1
%    allCsirsInd  - {1x4} CSI-RS indices per resource
%    allCsirsSym  - {1x4} CSI-RS symbols per resource
%    slotAssign   - [1x4] slot index (0 or 1) per resource
%    cdmLengths   - [1x2] e.g. [2 4] for Row 18 (FD-CDM2 x TD-CDM4)
%    nTxAntennas  - 128
%    nRxAntennas  - 4
%
%  Outputs:
%    H_est_full - [K x 28 x nRx x nTx] full 2-slot estimated channel
%    nVar_all   - [1x4] noise variance per resource

nResources   = 4;
nPortsPerRes = nTxAntennas / nResources;   % 32
nSymPerSlot  = carrier.SymbolsPerSlot;     % 14
nSubcarriers = carrier.NSizeGrid * 12;
nSymbols_2s  = 2 * nSymPerSlot;           % 28

H_est_full = zeros(nSubcarriers, nSymbols_2s, nRxAntennas, nTxAntennas);
nVar_all   = zeros(1, nResources);

for r = 1:nResources
    slotNum = slotAssign(r);
    carrier.NSlot = slotNum;

    if slotNum == 0
        rxGrid_slot = rxGrid_s0;
    else
        rxGrid_slot = rxGrid_s1;
    end

    [H_est_res, nVar_res] = nrChannelEstimate(carrier, rxGrid_slot, ...
        allCsirsInd{r}, allCsirsSym{r}, 'CDMLengths', cdmLengths);

    % H_est_res: [K x 14 x nRx x 32]
    portStart = (r-1)*nPortsPerRes + 1;
    portEnd   = r*nPortsPerRes;
    symStart  = slotNum*nSymPerSlot + 1;
    symEnd    = (slotNum+1)*nSymPerSlot;

    H_est_full(:, symStart:symEnd, :, portStart:portEnd) = H_est_res;
    nVar_all(r) = nVar_res;
end

end
