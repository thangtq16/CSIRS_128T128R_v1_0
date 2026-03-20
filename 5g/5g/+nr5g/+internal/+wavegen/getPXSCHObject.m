function pxsch = getPXSCHObject(wavePXSCH, symbperslot, reservedPRBIn, reservedRE, isDownlink)
%getPXSCHObject Creates nrPXSCHConfig object
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   PXSCH = getPXSCHObject(WAVEPXSCH,SYMBPERSLOT,RESERVEDPRBIN,RESERVEDRE,ISDOWNLINK) provides
%   the PXSCH configuration object nrPXSCHConfig, given the input
%   nrWavegenPXSCHConfig object WAVEPXSCH, number of symbols per slot
%   SYMBPERSLOT, reserved resource blocks RESERVEDPRBIN, reserved RE (for
%   CSI-RS) RESERVEDRE, and link direction ISDOWNLINK.

%   Copyright 2020-2025 The MathWorks, Inc.

%#codegen

    % Maximum number of OFDM symbols that can be allocated in a slot from
    % the specified starting OFDM symbol
    symAlloc = double(wavePXSCH.SymbolAllocation);
    if ~isempty(symAlloc)
        numSymbols = min(symbperslot-symAlloc(1), symAlloc(2));
    else
        numSymbols = 0;
    end

    % Cap the duration (to the max allowed) back to the object, to avoid
    % any OFDM symbols spanning more than one slot
    if numSymbols > 0
        symbolAllocation = [wavePXSCH.SymbolAllocation(1) numSymbols];
    else
        % In the case of extended cyclic prefix, the number of allocated
        % symbols can be negative. In this case, set the symbol allocation
        % to empty.
        symbolAllocation = zeros(0,1);
    end
    
    if isDownlink
        pxsch = nrPDSCHConfig('SymbolAllocation', symbolAllocation, 'PRBSet', unique(wavePXSCH.PRBSet(:)), ...
            'ReservedRE', reservedRE, ...
            'DMRS', wavePXSCH.DMRS, 'PTRS', wavePXSCH.PTRS);
        pxsch.ReservedPRB = reservedPRBIn;
        
        commonProps = {'Modulation', 'NumLayers', 'MappingType', 'PRBSetType', ...
            'VRBToPRBInterleaving', 'VRBBundleSize', 'EnablePTRS', 'NID', 'RNTI'};
        for idx = 1:length(commonProps)
            pxsch.(commonProps{idx}) = wavePXSCH.(commonProps{idx});
        end
        
    else % Uplink
        pxsch = nrPUSCHConfig('SymbolAllocation', symbolAllocation, 'PRBSet', unique(wavePXSCH.PRBSet(:)), ...
            'DMRS', wavePXSCH.DMRS, 'PTRS', wavePXSCH.PTRS);
        
        commonProps = {'Modulation', 'NumLayers', 'MappingType', 'TransformPrecoding',...
            'TransmissionScheme', 'NumAntennaPorts', 'TPMI', 'CodebookType', ...
            'FrequencyHopping', 'SecondHopStartPRB', 'Interlacing','RBSetIndex','InterlaceIndex',...
            'BetaOffsetACK', 'BetaOffsetCSI1', 'BetaOffsetCSI2',...
            'UCIScaling', 'NID', 'RNTI', 'NRAPID', 'EnablePTRS'};
        for idx = 1:length(commonProps)
            pxsch.(commonProps{idx}) = wavePXSCH.(commonProps{idx});
        end
    end
end