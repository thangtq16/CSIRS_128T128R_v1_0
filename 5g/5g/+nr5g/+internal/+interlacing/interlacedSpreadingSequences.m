function wn = interlacedSpreadingSequences(nIRB,nRE,sf,occi,numOFDMSymbols)
%interlacedSpreadingSequences Spreading sequences for interlaced PUCCH format 2
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   interlacedSpreadingSequences(NIRB,NRE,SF,OCCI,NSYM) returns the
%   NIRB-dependent spreading sequences for PUCCH format 2 with interlacing.
%   The output is an N-by-SF matrix of spreading sequences. N is the number
%   of REs available for PUCCH transmission in NIRB resource blocks (N =
%   numel(NIRB)*(NRE/SF), with NRE = 8 the number of RE per PRB available
%   for PUCCH format 2) and NSYM number of OFDM symbols.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    % Calculate the spreading sequence for each interlaced RB (wn). The
    % index n of wn depends on the OCCI and the index of the RB in the
    % interlace. Each row corresponds to 1 IRB and each column to a
    % subcarrier. The QPSK modulated symbols will be spread along SF
    % subcarriers.
    n0 = double(occi);
    n = mod(n0 + nIRB,sf);
    wn = nr5g.internal.pucch.spreadingSequence(sf,n);

    % Repeat the IRB-dependent spreading sequences (rows of wn) such that
    % all PUCCH modulation symbols in a PRB are spread with the same
    % sequence.
    wn = repelem(wn,nRE/sf(1),1);

    % Replicate spreading sequences to match the number of OFDM symbols
    wn = repmat(wn,numOFDMSymbols(1),1);

end