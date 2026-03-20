function y = blockWiseSpread(d,Mrb,sf,occi)
%blockWiseSpread Blockwise spreading for PUCCH formats 3 and 4
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   Y = blockWiseSpread(D,MRB,SF,OCCI) returns the blockwise spread
%   symbols Y given the modulated symbols D, the number of resource blocks
%   MRB, spreading factor SF, and orthogonal cover code index OCCI, as
%   defined in TS 38.211 Section 6.3.2.6.3.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    sf = double(sf(1));
    Msc = double(Mrb(1))*12;

    % Get the orthogonal cover code sequence based on sf and occi values
    wn = nr5g.internal.pucch.blockWiseSpreadingSequence(sf,occi);
    w = repmat(wn,Msc/sf,1);
    y = zeros(sf*length(d),1,'like',d);
    k = 0:Msc-1;
    nSymbols = sf * length(d)/Msc;
    for l = 0:nSymbols-1
        y(l*Msc + k + 1) = w(:).*d(l*(Msc/sf) + mod(k,(Msc/sf)) + 1);
    end

end