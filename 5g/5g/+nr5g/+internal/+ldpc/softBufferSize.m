function [NBuffer,K,Kd] = softBufferSize(cbsinfo,Ncb)
%softBufferSize Calculate soft buffer size according to TS 38.212 5.4.2
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   See also nrRateRecoverLDPC.

%   Copyright 2022 The MathWorks, Inc.

%#codegen

    % Puncture systematic bits
    K = cbsinfo.K - 2*cbsinfo.Zc;

    % Exclude fillers
    Kd = K - cbsinfo.F;

    % Get number of filler bits inside the circular buffer
    NFillerBits = max(min(K,Ncb)-Kd,0);

    % Buffer size without filler bits
    NBuffer = Ncb - NFillerBits;

end
