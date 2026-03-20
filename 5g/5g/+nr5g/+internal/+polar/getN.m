function N = getN(K,E,nMax)
%getN Returns N for a given K, E and nMax
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   N = nr5g.internal.polar.getN(K,E,NMAX) returns the mother code block
%   length for the specified number of input bits (K), number of
%   rate-matched output bits (E) and maximum value of n (NMAX).
%
%   See also nrPolarEncode.

%   Copyright 2018 The MathWorks, Inc.

% References:
%   [1] 3GPP TS 38.212, "3rd Generation Partnership Project; Technical 
%   Specification Group Radio Access Network; NR; Multiplexing and channel 
%   coding (Release 15). Section 5.3.1.

%#codegen

    % Get n, N, Section 5.3.1
    cl2e = ceil(log2(E));
    if (E <= (9/8) * 2^(cl2e-1)) && (K/E < 9/16)
        n1 = cl2e-1;
    else
        n1 = cl2e;
    end

    rmin = 1/8;
    n2 = ceil(log2(K/rmin));

    nMin = 5;
    n = max(min([n1 n2 nMax]),nMin);
    N = 2^n;

end
