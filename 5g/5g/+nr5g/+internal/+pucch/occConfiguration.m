function [sf,occi] = occConfiguration(pucch,varargin)
%occConfiguration Orthogonal cover code configuration for PUCCH
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    if nargin < 2
        formatPUCCH = nr5g.internal.pucch.getPUCCHFormat(pucch);
    else
        formatPUCCH = varargin{1};
    end
    
    if (formatPUCCH == 1)
        sf = [];
        occi = double(pucch.OCCI(1));
    elseif (formatPUCCH == 4) || ( formatPUCCH>1 && nr5g.internal.interlacing.isInterlaced(pucch) && length(pucch.InterlaceIndex) < 2 && pucch.SpreadingFactor > 1)
        % (Format 4) or (Formats 2 or 3 with a single interlace)
        sf = double(pucch.SpreadingFactor(1));
        occi = double(pucch.OCCI(1));
    else % (Format 0) or (Formats 2 or 3 without interlacing or more than 1 interlace)
        sf = [];
        occi = [];
    end

end