function pathDelays = getPathDelays(theStruct)
%getPathDelays get path delays
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2017-2020 The MathWorks, Inc.

%#codegen

    objinfo = wireless.internal.channelmodels.CDLChannelDelayProfileInfo(theStruct);
    pathDelays = objinfo.PathDelays;

end
