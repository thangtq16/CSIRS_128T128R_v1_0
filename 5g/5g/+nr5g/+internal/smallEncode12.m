function out = smallEncode12(uciBits,modulation)
%smallEncode12 Encoding for small block lengths of 1 or 2 bits
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   OUT = nr5g.internal.smallEncode12(IN,MODULATION) encodes the input bits
%   IN as per Section 5.3.3.1 and 5.3.3.2 of TS 38.212. IN must be a binary
%   scalar or column vector of length 2. MODULATION specifies the
%   modulation scheme used as one of
%   'pi/2-BPSK','QPSK','16QAM','64QAM','256QAM'. The place-holder bits "x"
%   and "y" in tables 5.3.3.1-1 and 5.3.3.2-1 are represented by "-1" and
%   "-2", respectively.
%
%   % Example: Encode 1-bit for 16QAM.
%
%   out = nr5g.internal.smallEncode12(1,'16QAM')
%
%   See also nrUCIEncode, nrUCIDecode.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    % Get modulation order
    Qm = nr5g.internal.getQm(modulation);

    % Encode, with placeholder bits
    x = -1;
    y = -2;
    if length(uciBits)==1 % A=1
        if Qm==1
            out = uciBits;
        else
            out = [uciBits; y; repmat(x,Qm-2,1)];
        end
    else % A=2
        c2 = xor(uciBits(1),uciBits(2));
        if Qm==1
            out = [uciBits; c2];
        else
            ib = reshape(repmat([uciBits; c2],2,1),2,3);
            xb = repmat(x,Qm-2,3);
            out = reshape([ib;xb],[],1);
        end
    end

end