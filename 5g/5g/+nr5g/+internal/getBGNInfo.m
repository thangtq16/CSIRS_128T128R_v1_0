function info = getBGNInfo(A,R)
% getBGNInfo provides the information of CRC attachment and base graph number 
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   INFO = nr5g.internal.getBGNInfo(A,R) provides the information of the
%   base graph number based on the inputs payload size A and target code
%   rate R.
%   1) Payload size (A) should be a scalar nonnegative integer
%   2) Target code rate (R) should be a numeric between 0 and 1
%
%   INFO contains the following fields:
%   CRC - CRC polynomial
%   L   - Number of parity bits
%   BGN - Base graph number
%   B   - Payload size after CRC attachment
%
%   Example:
%   % Base graph number 2 is used if payload size <= 292 or if rate <= 0.25
%   % or if (payload size <= 3824, rate <=0.67). Else base graph number 1
%
%   info = nr5g.internal.getBGNInfo(3824,0.8)
%   info1 = nr5g.internal.getBGNInfo(3824,0.5)

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    % Cast A to double, to make all the output fields have same data type
    A = double(A);

    % LDPC base graph selection
    if A <= 292 || (A <= 3824 && R <= 0.67) || R <= 0.25
      bgn = 2;
    else
      bgn = 1;
    end

    % Get transport block size after CRC attachment according to 38.212
    % 6.2.1 and 7.2.1, and assign CRC polynomial to CRC field of output
    % structure info
    if A > 3824
      L        = 24;
      info.CRC = '24A';
    else
      L        = 16;
      info.CRC = '16';
    end

    % Get the length of transport block after CRC attachment
    B = A + L;

    % Get the remaining fields of output structure info
    info.L   = L;
    info.BGN = bgn;
    info.B   = B;

end