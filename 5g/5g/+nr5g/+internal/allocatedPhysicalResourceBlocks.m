function prbset = allocatedPhysicalResourceBlocks(carrier,channel)
%allocatedPhysicalResourceBlocks PRB set allocated to channel
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023 The MathWorks, Inc.

%#codegen
    
    % When Interlacing = true, the frequency resources are the intersection
    % of the RB set between intracell guard bands and the resources of the
    % interlace. Otherwise, use PRBSet.
    interlacing = nr5g.internal.interlacing.isInterlaced(channel);
    if interlacing
        [~,prbset] = nr5g.internal.interlacing.interlacedResourceBlockSet(carrier,channel);
    else
        prbset = unique(double(channel.PRBSet(:).'));
    end

end
