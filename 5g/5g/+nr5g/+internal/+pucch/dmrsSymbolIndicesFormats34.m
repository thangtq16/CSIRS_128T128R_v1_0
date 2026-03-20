function l = dmrsSymbolIndicesFormats34(symAllocation,freqHopping,additionalDMRS)
%dmrsSymbolIndicesFormats34 DM-RS OFDM symbol indices for PUCCH formats 3 and 4
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   L = dmrsSymbolIndicesFormats34(SYMALLOCATION,FREQHOPPING,ADDITIONALDMRS)
%   provides the resource element indices (0-based) for the DM-RS symbols of
%   PUCCH formats 3 and 4, considering these inputs:
%   SYMALLOCATION  - Symbol allocation for PUCCH transmission. It is a
%                    two-element vector, where first element is the symbol
%                    index corresponding to first OFDM symbol of the PUCCH
%                    transmission in the slot and second element is the
%                    number of OFDM symbols allocated for PUCCH
%                    transmission, which is in range 4 and 14.
%   FREQHOPPING    - Intra-slot frequency hopping flag. It is either 1 or 0
%                    representing the set {'enabled', or 'disabled'}
%                    provided by higher-layer parameter
%                    intraSlotFrequencyHopping.
%   ADDITIONALDMRS - Additional DM-RS flag. It is either true or false,
%                    provided by higher-layer parameter additionalDMRS.
%                    When the number of OFDM symbols allocated for PUCCH is
%                    greater than 9 and ADDITIONALDMRS is 1 or 0, there are
%                    4 or 2 DM-RS OFDM symbols, respectively.
%
%   Example:
%   % Get the OFDM symbols indices for DM-RS symbols given the starting
%   % symbol of PUCCH allocation as 0, number of symbols allocated as 10,
%   % frequency hopping disabled and with additional DM-RS.
%
%   symAllocation = [0 10];
%   freqHopping = 1;
%   additionalDMRS = true;
%   l = nr5g.internal.pucch.dmrsSymbolIndicesFormats34(symAllocation,freqHopping,additionalDMRS)

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    % Get the number of OFDM symbols allocated for PUCCH and starting
    % symbol of PUCCH
    nPUCCHSym = symAllocation(2);
    symIndex = symAllocation(1);

    % Get the DM-RS OFDM symbol locations, relative to the starting OFDM
    % symbol of PUCCH (TS 38.211 Table 6.4.1.3.3.2-1)
    if nPUCCHSym == 4
        if freqHopping
            sym = [0 2];
        else
            sym = 1;
        end
    elseif nPUCCHSym <= 9
        indexTable = [0 3;...
                      1 4;...
                      1 4;...
                      1 5;...
                      1 6];
        sym = indexTable(nPUCCHSym-4,:);
    else
        if additionalDMRS
            indexTable = [1 3 6 8;...
                          1 3 6 9;...
                          1 4 7 10;...
                          1 4 7 11;...
                          1 5 8 12];
        else
            indexTable = [2 7;...
                          2 7;...
                          2 8;...
                          2 9;...
                          3 10];
        end
        sym = indexTable(nPUCCHSym-9,:);
    end

    % Get the DM-RS OFDM symbol locations of PUCCH
    l = symIndex+sym; % 0-based

end
