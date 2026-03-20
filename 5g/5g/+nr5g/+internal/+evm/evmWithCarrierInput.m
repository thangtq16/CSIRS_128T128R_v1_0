function [rmsEVM,peakEVM,info] = evmWithCarrierInput( ...
        carrier,channel,rxGrids,optionalInputs)
%evmWithCarrierInput Process received grid and measure EVM
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
% This function performs these steps:
%   1. Ideal signal i2 generation having only reference signals
%   2. Exclude DC subcarrier location in reference and received grid
%   3. Channel estimation
%   4. ZF equalization
%   5. Common phase error (CPE) compensation
%   6. Ideal signal i1 re-construction through hard slicing
%   7. EVM measurement
%
%   INFO contains these fields:
%   * ErrorVectorGrid
%   * EqualizedGrid
%   * ReferenceGrid
%   * LocationMap
%   * LLRs

% Copyright 2024-2025 The MathWorks, Inc.

%#codegen

    % Initialize the parameters
    dcInd = optionalInputs.TxDirectCurrentLocation;
    expSlots = optionalInputs.NSlot;
    reservedRE = optionalInputs.ReservedRE;
    nSymbSlot = carrier.SymbolsPerSlot;
    isDownlink = isa(channel,"nrPDSCHConfig") || isa(channel,"nrPDCCHConfig");
    % Loop through allocated slots and form a reference grid containing
    % reference signals
    [refGridAllRS,notionalGridAllSlots,rsRefIndex] = ...
        nr5g.internal.evm.getI2ReferenceSignalGrid(carrier,channel,expSlots,reservedRE,class(rxGrids));

    % Store reference grid containing all reference signals with no
    % DC exclusion to generate the ideal grid appropriately
    refGridAllRSNoExclusion = refGridAllRS;

    % Exclude the DC subcarrier location, by setting the resource element
    % values in DC subcarrier of reference and received grid to 0
    [rxGrids,refGridAllRS] = nr5g.internal.evm.excludeDCSubcarrier(dcInd, ...
        rxGrids,refGridAllRS,true);

    % Obtain channel estimates
    cdmLengths = [1 1];
    if isprop(channel,"DMRS")
        cdmLengths = channel.DMRS.CDMLengths;
    end
    tmpRefGrid = refGridAllRS;
    tmpRefGrid(notionalGridAllSlots~=rsRefIndex) = 0;
    Hest = nr5g.internal.evm.channelEstimateEVM(rxGrids,tmpRefGrid, ...
        cdmLengths,nSymbSlot,isDownlink);

    % Extrapolate channel estimates for the RBs where there is channel
    % allocation with no channel estimate.
    Hest = nr5g.internal.evm.adjustChannelEstimate(Hest,notionalGridAllSlots, ...
        rsRefIndex,nSymbSlot);

    % Perform equalization for all different signals in the channel for all
    % the edges
    noiseEst = 0;
    [eqGrid,csiGrid] = nr5g.internal.evm.equalizeChannel(rxGrids,Hest, ...
        notionalGridAllSlots,noiseEst);

    % Perform CPE estimation and compensation for all the OFDM symbols and
    % edges
    eqGrid = nr5g.internal.evm.compensateCPE(channel,eqGrid,refGridAllRS, ...
        notionalGridAllSlots,expSlots,carrier.CyclicPrefix);

    % Re-construct the i1(v) reference signal
    [idealSlotsAllEdges,equalizedSlotsAllEdges,llrs] = nr5g.internal.evm.getI1ReferenceSignalGrid( ...
        carrier,channel,eqGrid,csiGrid,refGridAllRSNoExclusion,notionalGridAllSlots,noiseEst,expSlots);

    % When DC subcarrier is present, set the equalized and ideal symbols at
    % DC subcarrier to 0 to avoid computation of EVM.
    [idealSlotsAllEdges,equalizedSlotsAllEdges] = nr5g.internal.evm.excludeDCSubcarrier(dcInd, ...
        idealSlotsAllEdges,equalizedSlotsAllEdges);

    % Measure EVM
    [rmsEVM,peakEVM,evGrid] = nr5g.internal.evm.evm(equalizedSlotsAllEdges, ...
        idealSlotsAllEdges,notionalGridAllSlots,carrier.CyclicPrefix, ...
        carrier.SubcarrierSpacing);
    info.ErrorVectorGrid = evGrid;
    info.EqualizedGrid = equalizedSlotsAllEdges;
    info.ReferenceGrid = idealSlotsAllEdges;
    info.LocationMap = notionalGridAllSlots;
    info.LLRs = llrs;

end
