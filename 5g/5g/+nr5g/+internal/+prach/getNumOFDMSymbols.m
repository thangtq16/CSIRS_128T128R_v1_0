function L = getNumOFDMSymbols(prach)
%getNumOFDMSymbols Get the number of OFDM symbols for the given PRACH configuration.
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.

%  Copyright 2022 The MathWorks, Inc.

%#codegen

    % Get the nominal PRACH duration in OFDM symbols
    L = prach.PRACHDuration(1);

    % One time occasion of preamble format C2 spans 6 OFDM symbols,
    % according to TS 38.211 Tables 6.3.3.2-2 to 6.3.3.2-4. However, note
    % that the number of OFDM symbols per PRACH slot appears as parts of
    % the expressions for N_u in TS 38.211 Table 6.3.3.1-2. This is 4 for
    % preamble format C2. That is, considering that the cyclic prefix for
    % preamble format C2 is approximately one OFDM symbol long, there is a
    % gap of one OFDM symbol between the first and the second time occasion
    % of the PRACH preamble for format C2. Thus, the last OFDM symbol in
    % each time occasion for preamble format C2 is empty. You can refer to
    % TDoc R1-1805220 Figures 1 and 2 for a visual representation of this
    % concept.
    if strcmpi(prach.Format,'C2')
        L = L - 2;
    end

    % There might be cases for short PRACH preambles in which the cyclic
    % prefix is equal to or larger than one OFDM symbol. From the
    % definition of the FFT size and the N_CP column of TS 38.211 Table
    % 6.3.3.1-2, these cases can be represented through these two conditions:
    %  1. N_CP >= 2048
    %  2. (SCS >= (2048 - N_CP)*15/n) && isActive(prach) && prachCrossHalfSubframe
    % The number of samples n=16 is added when the PRACH occasion crosses
    % half subframe (time instants 0 and 0.5 ms), specified in the above
    % condition (2) by prachCrossHalfSubframe. When either of these
    % conditions is true, the function considers a length of the preamble
    % that is one symbol longer than the value of PRACHDuration to account
    % for the additional symbol added in place of the cyclic prefix.
    % Practically, these two conditions are true only for PRACH format C2.
    if strcmpi(prach.Format,'C2')
        L = L + 1;
    end

end