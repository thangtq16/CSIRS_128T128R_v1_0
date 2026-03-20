function [rmsEVM,peakEVM,info] = nrEVM(varargin)
%nrEVM Measure error vector magnitude
%   [RMSEVM,PEAKEVM,EVMINFO] = nrEVM(CARRIER,CHANNEL,RXGRID) measures the
%   error vector magnitude (EVM) for the given carrier configuration object
%   CARRIER, channel configuration object CHANNEL, and received grid
%   RXGRID. CARRIER is a scalar nrCarrierConfig object. CHANNEL is a scalar
%   nrPDSCHConfig, nrPUSCHConfig, or nrPDCCHConfig object representing
%   physical downlink shared channel (PDSCH), physical uplink shared
%   channel (PUSCH), or physical downlink control channel (PDCCH),
%   respectively. RXGRID is a K-by-N-by-R-by-E complex array where K is the
%   number of subcarriers, N is the number of OFDM symbols, R is the number
%   of receive antennas, and E is number of edges. K must be equal to the
%   number of subcarriers of the configured carrier. N must be a multiple
%   of number of OFDM symbols in a slot. E must be 1 or 2. This syntax
%   performs channel estimation using reference signals, zero-forcing (ZF)
%   equalization, reference data symbol re-construction using hard slicing,
%   and EVM measurement using equalized and reconstructed symbols. When E
%   is 1, this syntax performs channel estimation using nrChannelEstimate
%   function. When E is 2, this syntax performs channel estimation as
%   mentioned in TS 38.104 for PDSCH and TS 38.101-1 (FR1) / 38.101-2 (FR2)
%   for PUSCH. The function returns the EVM in two forms: root mean square
%   RMSEVM and peak PEAKEVM. The function also returns the structural
%   information EVMINFO, which contains:
%
%   ErrorVectorGrid - A K-by-N-by-P-by-E complex array containing the
%                     normalized error vectors. Error vector is defined as
%                     the ratio of difference in equalized symbols and
%                     re-constructed symbols to that of normalization
%                     factor. The normalization factor is computed using
%                     the re-constructed symbols for each slot and each
%                     layer independently. K is number of subcarriers, N is
%                     number of OFDM symbols, P is number of reference
%                     ports, and E is number of edges.
%   EqualizedGrid   - A K-by-N-by-P-by-E complex array containing the
%                     equalized symbols of all the entities in the channel.
%                     For example, equalized grid contains data, DM-RS and
%                     PT-RS (if present) symbols for PDSCH or PUSCH. In
%                     case of PUSCH with transform precoding enabled, this
%                     grid represents the data and PT-RS (if present)
%                     symbols before transform precoding.
%
%   Note that the EVM values in the RMSEVM and PEAKEVM are in linear scale,
%   and not in percent. To obtain EVM in percent, multiply the value of the
%   RMSEVM and PEAKEVM by 100.
%
%   [RMSEVM,PEAKEVM,EVMINFO] = nrEVM(CARRIER,CHANNEL,RXGRID,NAME=VALUE)
%   measures EVM as above with additional options as NAME=VALUE pairs:
%
%   'TxDirectCurrentLocation' - A nonnegative integer scalar representing
%                               the location of direct current (DC)
%                               subcarrier in the carrier. This value is
%                               provided by higher-layer parameter
%                               txDirectCurrentLocation of
%                               uplinkTxDirectCurrentBWP and
%                               SCS-SpecificCarrier Radio Resource Control
%                               (RRC) information elements (IEs). Set the
%                               value to empty ([]) to not exclude the DC
%                               subcarrier. When TxDirectCurrentLocation is
%                               nonempty, the information in the DC
%                               subcarrier is excluded in EVM measurement.
%                               The default value is [].
%
%   [RMSEVM,PEAKEVM,EVMINFO] = nrEVM(EQGRID,REFGRID) returns the root mean
%   square EVM, peak EVM, and additional EVM information, given the
%   equalized grid EQGRID and reference grid REFGRID. EQGRID is a
%   K-by-N-by-P-by-E complex array containing the equalized symbols.
%   REFGRID contains reference symbols and is of same size as EQGRID. All
%   the non-zero elements of REFGRID are considered as valid symbols and
%   are used for EVM measurement.
%
%   Example 1:
%   % Create a resource grid containing the data and DM-RS of PDSCH and add
%   % noise to it. Measure EVM of PDSCH and associated reference signals.
%
%   rng(0);
%   % Create and populate resource grid with PDSCH data
%   carrier = nrCarrierConfig;
%   pdsch = nrPDSCHConfig;
%   pdsch.Modulation = "256QAM";
%   [pdschIndices,pdschInfo] = nrPDSCHIndices(carrier,pdsch);
%   cw = randi([0 1],pdschInfo.G,1);
%   pdschSym = nrPDSCH(carrier,pdsch,cw);
%   pdschDMRSInd = nrPDSCHDMRSIndices(carrier,pdsch);
%   pdschDMRSSym = nrPDSCHDMRS(carrier,pdsch);
%   txGrid = nrResourceGrid(carrier);
%   txGrid(pdschIndices) = pdschSym;
%   txGrid(pdschDMRSInd) = pdschDMRSSym;
%
%   % Perform OFDM modulation, add noise, and OFDM demodulation
%   [txWaveform,ofdmInfo] = nrOFDMModulate(carrier,txGrid);
%   evmPercent = 2.0;
%   noise = evmPercent/(100*sqrt(ofdmInfo.Nfft))* randn(size(txWaveform),like=1i);
%   rxWaveform = txWaveform + noise;
%   rxGrid = nrOFDMDemodulate(carrier,rxWaveform);
%
%   % Measure EVM
%   [rmsEVM,peakEVM,evmInfo] = nrEVM(carrier,pdsch,rxGrid);
%   disp("EVM RMS of PDSCH data = " + (rmsEVM*100) + " %")
%   % Compute EVM of PDSCH DM-RS using the additional information
%   ev = evmInfo.ErrorVectorGrid;
%   dmrsEV = ev(pdschDMRSInd);
%   rmsDMRSEVM = sqrt(mean(abs(dmrsEV).^2));
%   disp("EVM RMS of PDSCH DM-RS = " + (rmsDMRSEVM*100) + " %")
%
%   % Plot the equalized and ideal constellation of data and DM-RS
%   f = figure(1);
%   ax = axes(f);
%   plot(ax,evmInfo.EqualizedGrid(pdschIndices),"b.")
%   hold on
%   plot(ax,pdschSym,"r+")
%   plot(ax,evmInfo.EqualizedGrid(pdschDMRSInd),"y.")
%   plot(ax,pdschDMRSSym,"k+")
%   axis(ax,'equal')
%   hold off
%   legend(ax,"Data Eq","Data Ref","DM-RS Eq","DM-RS Ref")
%   title(ax,"Constellation Diagram")
%   ylabel(ax,"Quadrature Amplitude")
%   xlabel(ax,"In-phase Amplitude")
%
%   % Plot the EVM resource grid
%   f2 = figure(2);
%   ax = axes(f2);
%   surf(ax,abs(ev)*100)
%   shading flat
%   view(-30,60)
%   title(ax,"EVM Resource Grid")
%   xlabel(ax,"OFDM Symbols")
%   ylabel(ax,"Subcarriers")
%   zlabel(ax,"EVM (%)")
%
%   Example 2:
%   % Generate a grid of random QPSK constellation with a defined EVM.
%   % Measure the RMS and Peak EVM.
%
%   rng(0)
%   idealGrid = zeros([624 14],like=1i);
%   numREs = numel(idealGrid);
%   idealSym = nrSymbolModulate(randi([0 1],numREs*2,1),"QPSK");
%   idealGrid(1:numREs) = idealSym;
%
%   % Add noise with defined EVM
%   evmPercent = 14;
%   N0 = randn(size(idealGrid),like=1i);
%   noise = N0*(evmPercent/100);
%   rxGrid = idealGrid+noise;
%
%   % Measure and display EVM in percent
%   [rmsEVM,peakEVM] = nrEVM(rxGrid,idealGrid);
%   disp("EVM RMS = " + (rmsEVM*100) + " %")
%   disp("EVM Peak = " + (peakEVM*100) + " %")
%
%   See also nrSSBMeasurements, nrCSIRSMeasurements.

%   Copyright 2024-2025 The MathWorks, Inc.

%#codegen

narginchk(2,5);

fcnName = "nrEVM";

if nargin > 2
    %nrEVM(CARRIER,CHANNEL,RXGRIDS)
    %nrEVM(CARRIER,CHANNEL,RXGRIDS,Name=Value)

    % Validate carrier
    carrier = varargin{1};
    coder.internal.errorIf(~(isa(carrier,"nrCarrierConfig") && isscalar(carrier)), ...
        "nr5g:nrEVM:InvalidCarrierInput")

    % Validate channel
    channel = varargin{2};
    isPDSCH = isa(channel,"nrPDSCHConfig");
    isPDCCH = isa(channel,"nrPDCCHConfig");
    isPUSCH = isa(channel,"nrPUSCHConfig");
    cond = (isscalar(channel) && (isPDSCH || isPDCCH || isPUSCH));
    coder.internal.errorIf(~cond,"nr5g:nrEVM:InvalidChannelInput")
    % For PDSCH and PDSCH channels, check if valid DM-RS is available
    % before proceeding further
    nSymbSlot = carrier.SymbolsPerSlot;
    if isPDSCH || isPUSCH
        % Error for custom DM-RS symbol set
        coder.internal.errorIf(~isempty(channel.DMRS.CustomSymbolSet), ...
            "nr5g:nrEVM:UnsupportedCustomDMRS")
        % Error if there are no DM-RS symbols within the allocation
        if ~(isempty(channel.SymbolAllocation) || (channel.SymbolAllocation(end)==0))
            symbolset = channel.SymbolAllocation(1) + (0:channel.SymbolAllocation(end)-1);
            symbolset = symbolset(symbolset < nSymbSlot);
            mappingType = strcmpi(channel.MappingType,"B");
            typeAPos = channel.DMRS.DMRSTypeAPosition;
            dmrsLen = channel.DMRS.DMRSLength;
            dmrsAddPos = channel.DMRS.DMRSAdditionalPosition;
            if isPDSCH
                chName = "PDSCH";
                nr5g.internal.pdsch.validateInputs(carrier,channel);
                dmrssymbolset = nr5g.internal.pdsch.lookupPDSCHDMRSSymbols( ...
                    symbolset,mappingType,typeAPos,dmrsLen,dmrsAddPos);
            else
                chName = "PUSCH";
                [~,~,~,~,~,freqHopping] = nr5g.internal.pusch.validateInputs(carrier,channel);
                dmrssymbolset = nr5g.internal.pusch.lookupPUSCHDMRSSymbols( ...
                    symbolset,mappingType,typeAPos,dmrsLen,dmrsAddPos,...
                    strcmpi(freqHopping,"intraSlot"));
            end
            coder.internal.errorIf(isempty(dmrssymbolset), ...
                "nr5g:nrPXSCH:DMRSParametersNoSymbols",chName)
        end
    end
    channel = updateChannelProps(channel);

    % Validate receive grid
    rxGrids = varargin{3};
    evm3GPP = size(rxGrids,4)>1;
    if evm3GPP
        dim = 2;
    else
        dim = NaN;
    end
    validateattributes(rxGrids,{'double','single'},{ ...
        "nonempty","finite","size", ...
        [double(carrier.NSizeGrid)*12 carrier.SymbolsPerSlot NaN dim]}, ...
        fcnName,"RXGRID",3)

    % Validate optional input arguments and get the slot numbers to process
    if nargin > 3
        firstOptArg = 4;
        opts = nr5g.internal.parseOptions( ...
            fcnName,{"TxDirectCurrentLocation"},varargin{firstOptArg:end});
        dcInd = opts.TxDirectCurrentLocation;                % 0-based
    else
        % When there are no optional inputs, there is no DC subcarrier
        % exclusion.
        dcInd = [];
    end

    % Measure EVM
    optionalInputs = struct();
    optionalInputs.TxDirectCurrentLocation = dcInd;
    optionalInputs.NSlot = carrier.NSlot;
    optionalInputs.ReservedRE = {};
    [rmsEVM,peakEVM,tmpInfo] = nr5g.internal.evm.evmWithCarrierInput( ...
        carrier,channel,rxGrids,optionalInputs);
    info = struct;
    info.ErrorVectorGrid = tmpInfo.ErrorVectorGrid;
    info.EqualizedGrid = tmpInfo.EqualizedGrid;
else
    %nrEVM(EQGRID,REFGRID)
    equalizedGrid = varargin{1};
    refGrid = varargin{2};

    if size(equalizedGrid,4) > 2
        dim = 2;
    else
        dim = nan;
    end
    validateattributes(equalizedGrid,{'double','single'},{ ...
        "nonempty","finite","size",[NaN NaN NaN dim]},fcnName,"EQGRID",1)
    validateattributes(refGrid,{'double','single'}, ...
        {"nonempty","finite","size",size(equalizedGrid)}, ...
        fcnName,"REFGRID",2)

    % Get the data types of both the inputs
    eqDataType = class(equalizedGrid);
    refDataType = class(refGrid);
    if strcmpi(eqDataType,refDataType)
        % Both inputs have same data types, do not perform any cast
        equalizedGridToProcess = equalizedGrid;
        refGridToProcess = refGrid;
    else
        % One of the input is of type single and other is of type double.
        % Convert the input of type double to single.
        if strcmpi(eqDataType,"single")
            equalizedGridToProcess = equalizedGrid;
        else
            equalizedGridToProcess = cast(equalizedGrid,like=refGrid);
        end
        if strcmpi(refDataType,"single")
            refGridToProcess = refGrid;
        else
            refGridToProcess = cast(refGrid,like=equalizedGrid);
        end
    end

    % Measure EVM
    [nSC,nSym,nLayers] = size(equalizedGridToProcess);
    notionalGridAllSlots = zeros([nSC,nSym,nLayers]);
    notionalGridAllSlots(refGrid(:,:,:,1) ~= 0) = 1;
    eq = equalizedGridToProcess;
    eq(refGrid == 0) = zeros(1,like=eq);
    [rmsEVM,peakEVM,evGrid] = nr5g.internal.evm.evm(eq,refGridToProcess,notionalGridAllSlots);
    info.ErrorVectorGrid = evGrid;
    info.EqualizedGrid = eq;
end

end

function out = updateChannelProps(in)
% Returns the channel object with updated properties. This allows to
% support codegen for variable length property change, like,
% TransmissionScheme in PUSCH.

    if isa(in,"nrPUSCHConfig")
        % For PUSCH, set the transmission scheme to nonCodebook with all
        % other parameters same as input. Use constructor call for
        % properties that have variable length.
        out = nrPUSCHConfig(TransmissionScheme="nonCodebook" ,...
            DMRS=in.DMRS, ...
            PTRS=in.PTRS, ...
            SymbolAllocation=in.SymbolAllocation, ...
            PRBSet=in.PRBSet, ...
            NSizeBWP=in.NSizeBWP, ...
            NStartBWP=in.NStartBWP);
        % Exclude the properties set in constructor and the read-only
        % properties
        props = properties(in);
        excludeProps = {'TransmissionScheme','DMRS','PTRS','SymbolAllocation', ...
            'PRBSet','NSizeBWP','NStartBWP','NumCodewords'};
        for idx = 1:length(props)
            if ~any(strcmpi(props{idx},excludeProps))
                out.(props{idx}) = in.(props{idx});
            end
        end
    else
        out = in;
    end

end
