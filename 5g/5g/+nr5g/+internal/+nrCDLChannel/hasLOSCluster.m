function has = hasLOSCluster(DelayProfile,HasLOSCluster)
%hasLOSCluster indicates whether or not a delay profile has an LOS cluster
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2017-2018 The MathWorks, Inc.

%#codegen

    if (strcmpi(DelayProfile,'Custom'))
        has = HasLOSCluster;
    else
        has = wireless.internal.channelmodels.hasLOSPath(DelayProfile);
    end

end
