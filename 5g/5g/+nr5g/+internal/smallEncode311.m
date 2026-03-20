function cout = smallEncode311(uciBits)
%smallEncode311 Encoding for small block lengths of 3...11 bits
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   OUT = nr5g.internal.smallEncode311(IN) encodes the input bits IN as
%   per Section 5.3.3.3 of TS 38.212. IN must be a binary column vector of
%   length within a range of [3, 11].
%
%   % Example: Encode 4-bits
%
%   out = nr5g.internal.smallEncode311([1;0;0;1])
%
%   See also nrUCIEncode, nrUCIDecode.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    % Table 5.3.3.3-1, Section 5.3.3.3, TS 38.212.
    basisSeq = [1 1 0 0 0 0 0 0 0 0 1
               1 1 1 0 0 0 0 0 0 1 1
               1 0 0 1 0 0 1 0 1 1 1
               1 0 1 1 0 0 0 0 1 0 1
               1 1 1 1 0 0 0 1 0 0 1
               1 1 0 0 1 0 1 1 1 0 1
               1 0 1 0 1 0 1 0 1 1 1
               1 0 0 1 1 0 0 1 1 0 1
               1 1 0 1 1 0 0 1 0 1 1
               1 0 1 1 1 0 1 0 0 1 1
               1 0 1 0 0 1 1 1 0 1 1
               1 1 1 0 0 1 1 0 1 0 1
               1 0 0 1 0 1 0 1 1 1 1
               1 1 0 1 0 1 0 1 0 1 1
               1 0 0 0 1 1 0 1 0 0 1
               1 1 0 0 1 1 1 1 0 1 1
               1 1 1 0 1 1 1 0 0 1 0
               1 0 0 1 1 1 0 0 1 0 0
               1 1 0 1 1 1 1 1 0 0 0
               1 0 0 0 0 1 1 0 0 0 0
               1 0 1 0 0 0 1 0 0 0 1
               1 1 0 1 0 0 0 0 0 1 1
               1 0 0 0 1 0 0 1 1 0 1
               1 1 1 0 1 0 0 0 1 1 1
               1 1 1 1 1 0 1 1 1 1 0
               1 1 0 0 0 1 1 1 0 0 1
               1 0 1 1 0 1 0 0 1 1 0
               1 1 1 1 0 1 0 1 1 1 0
               1 0 1 0 1 1 1 0 1 0 0
               1 0 1 1 1 1 1 1 1 0 0
               1 1 1 1 1 1 1 1 1 1 1
               1 0 0 0 0 0 0 0 0 0 0];

    uciBitsD = cast(uciBits,'double');
    out = zeros(32,1);
    for idx = 1:length(uciBits)
        out = out + uciBitsD(idx).*basisSeq(:,idx);
    end
    cout = cast(mod(out,2),class(uciBits));

end