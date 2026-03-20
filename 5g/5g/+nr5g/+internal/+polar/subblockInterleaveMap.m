function jn = subblockInterleaveMap(N)
%subblockInterleaveMap Subblock interleaving pattern for Polar rate-matching
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   out = nr5g.internal.polar.subblockInterleaveMap(N) returns the
%   sub-block interleaving pattern for length N.
% 
%   See also nrRateMatchPolar, nrRateRecoverPolar.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

%   Reference:
%   [1] 3GPP TS 38.212, "3rd Generation Partnership Project; Technical 
%   Specification Group Radio Access Network; NR; Multiplexing and channel 
%   coding (Release 15). Section 5.4.1.1.

    % Table 5.4.1.1-1: Sub-block interleaver pattern
    pi = [0;1;2;4; 3;5;6;7; 8;16;9;17; 10;18;11;19;
          12;20;13;21; 14;22;15;23; 24;25;26;28; 27;29;30;31];

    jn = zeros(N,1);
    for n = 0:N-1
        i = floor(32*n/N);
        jn(n+1) = pi(i+1)*(N/32)+ mod(n,N/32);
    end

end
