function interlaced = isInterlaced(channel)
%isInterlaced check if PUCCH or PUSCH configuration is interlaced
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%  

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    interlaced = isa(channel,'nr5g.internal.interlacing.InterlacingConfig') && channel.Interlacing;
    
end