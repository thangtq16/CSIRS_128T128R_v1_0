function beta = ptrsPowerFactorDFTsOFDM(modulation)
% ptrsPowerFactorDFTsOFDM Power factor of PT-RS symbols in DFT-s-OFDM
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   BETA = ptrsPowerFactorDFTsOFDM(MODULATION) returns the power factor,
%   BETA, applied to PT-RS in DFT-s-OFDM waveform, according to TS 38.214
%   Table 6.2.3.2-1, based on the input modulation scheme, MODULATION. BETA
%   is the ratio between amplitude of one of the outermost constellation
%   points for the modulation scheme used for PUSCH and one of the
%   outermost constellation points for pi/2-BPSK.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    % TS 38.214 Table 6.2.3.2-1
    switch lower(modulation)
        case '16qam'
            beta = 3/sqrt(5);
        case '64qam'
            beta = 7/sqrt(21);
        case '256qam'
            beta = 15/sqrt(85);
        otherwise
            beta = 1;
    end

end
