%hSSBurstStartSymbols Synchronization Signal Burst OFDM starting symbols
%   STARTSYM = hSSBurstStartSymbols(BURST) returns the first OFDM symbol
%   STARTSYM of every Synchronization Signal block in a burst BURST.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen

function ssbStartSymbols = hSSBurstStartSymbols(burst)
    
    L = length(burst.TransmittedBlocks);
    
    cases = nrWavegenSSBurstConfig.BlockPattern_Values;
    caseIdx = find(strcmpi(burst.BlockPattern,cases), 1);
        
    % The first symbols of the candidate SS/PBCH blocks have indexes that
    % can be written as i + m*n, where i, m, and n are case dependent.
    % These are the values of m, i and length of n for block pattern cases
    % A through G.
    m = [14 28 14 28 56 14 14]; 
    i = {[2; 8] [4; 8; 16; 20] [2; 8] [4; 8; 16; 20] [8; 12; 16; 20; 32; 36; 40; 44] [2; 9] [2; 9]};
    nl = [2 1 2 16 8 32 32];
    if ismember(caseIdx, [1 2 3])
        if (L==4)
            nlcase = nl(caseIdx(1));
        else %if (L==8)
            nlcase = nl(caseIdx(1)) * 2;
        end
    else % nrDLCarrierConfig ensures L = 64 for FR2.
        nlcase = nl(caseIdx(1));
    end

    % 'alln' gives the overall set of SS block indices 'n' described in
    % TS 38.213 Section 4.1, from which a subset is used for each Case
    % A-G. There are different sets for Cases A-E and Cases F-G.
    if caseIdx <= 5
        n = [0:3 5:8 10:13 15:18];
    else % FR2-2 and eventually shared spectrum channel access
        n = (0:31);
    end

    I = i{caseIdx(1)};
    N = n(1:nlcase);
    M = m(caseIdx(1));
    ssbStartSymbols = reshape(I + M*N,1,[]);
    
end