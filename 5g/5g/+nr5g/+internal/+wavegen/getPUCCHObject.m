function pucchObj = getPUCCHObject(wavePUCCH, formatPUCCH, symbperslot)
%getPUCCHObject Creates nrPUCCHxConfig object
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   PUCCHOBJ = getPUCCHObject(WAVEPUCCH,FORMATPUCCH,SYMBPERSLOT) provides
%   the PUCCH configuration object nrPUCCHxConfig PUCCHOBJ, given the input
%   nrWavegenPUCCHxConfig object WAVEPUCCH, PUCCH format FORMATPUCCH,
%   number of symbols per slot SYMBPERSLOT.

%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen
        
    % Maximum number of OFDM symbols that can be allocated in a slot from
    % the specified starting OFDM symbol
    symAlloc = double(wavePUCCH.SymbolAllocation);
    if ~isempty(symAlloc)
        numSymbols = min(symbperslot-symAlloc(1), symAlloc(2));
    else
        numSymbols = 0;
    end

    % Cap the duration (to the max allowed) back to the object, to avoid
    % any OFDM symbols spanning more than one slot
    if numSymbols > 0
        symbolAllocation = [wavePUCCH.SymbolAllocation(1) numSymbols];
    else
        % In the case of extended cyclic prefix, the number of allocated
        % symbols can be negative. In this case, set the symbol allocation
        % to empty.
        symbolAllocation = zeros(0,1);
    end
    
    % Define the PUCCH object and its list of properties
    switch formatPUCCH
        case 0
            pucchObj = nrPUCCH0Config('SymbolAllocation', symbolAllocation);
            commonProps = {'PRBSet','FrequencyHopping','SecondHopStartPRB',...
                'Interlacing','RBSetIndex','InterlaceIndex'...
                ,'InitialCyclicShift','GroupHopping','HoppingID'};
        case 1
            pucchObj = nrPUCCH1Config('SymbolAllocation', symbolAllocation);
            commonProps = {'PRBSet','FrequencyHopping','SecondHopStartPRB',...
                'Interlacing','RBSetIndex','InterlaceIndex',...
                'InitialCyclicShift','GroupHopping','HoppingID','OCCI'};
        case 2
            pucchObj = nrPUCCH2Config('SymbolAllocation', symbolAllocation);
            commonProps = {'PRBSet','FrequencyHopping','SecondHopStartPRB',...
                'Interlacing','RBSetIndex','InterlaceIndex','SpreadingFactor',...
                'OCCI','RNTI','NID','NID0'};
        case 3
            pucchObj = nrPUCCH3Config('SymbolAllocation', symbolAllocation);
            commonProps = {'PRBSet','FrequencyHopping','SecondHopStartPRB',...
                'Interlacing','RBSetIndex','InterlaceIndex','Modulation',...
                'GroupHopping','HoppingID','SpreadingFactor','OCCI',...
                'AdditionalDMRS','DMRSUplinkTransformPrecodingR16','NID',...
                'RNTI','NID0'};
        otherwise % case 4
            pucchObj = nrPUCCH4Config('SymbolAllocation', symbolAllocation);
            commonProps = {'PRBSet','FrequencyHopping','SecondHopStartPRB',...
                'Modulation','GroupHopping','HoppingID','SpreadingFactor',...
                'OCCI','AdditionalDMRS','DMRSUplinkTransformPrecodingR16',...
                'NID','RNTI','NID0'};
    end

    % Assign the values to the PUCCH properties
    for idx = 1:length(commonProps)
        pucchObj.(commonProps{idx}) = wavePUCCH.(commonProps{idx});
    end
end