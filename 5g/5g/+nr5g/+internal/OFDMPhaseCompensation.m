function symbolPhases = OFDMPhaseCompensation(nfft,cpLengths,SCS,f0)
%OFDMPhaseCompensation OFDM phase compensation
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   This function calculates the vector of phase precompensation values
%   SYMBOLPHASES for each OFDM symbol, according to the phase term per OFDM
%   symbol in TS 38.211 Section 5.4.

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen

    N_mu_u = nfft;
    N_mu_CP = cpLengths;
    n_mu_tot = cumsum(N_mu_CP + N_mu_u);
    n_mu_start = [0 n_mu_tot(1:end-1)];
    
    SR = nfft * SCS * 1e3;
    t_mu_start = n_mu_start / SR;
    t_mu_CP = N_mu_CP / SR;
    
    symbolPhases = 2 * pi * f0 * (- t_mu_start - t_mu_CP);

end
