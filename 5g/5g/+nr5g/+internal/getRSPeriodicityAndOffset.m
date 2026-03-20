function [Trs,Toff] = getRSPeriodicityAndOffset(RSPeriod)
% getRSPeriodicityAndOffset returns the slot periodicity and offset of a
% reference signal (RS).
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%   [TRS,TOFF] = getRSPeriodicityAndOffset(RSPERIOD) returns the slot
%   periodicity TRS and offset TOFF of an RS with a slot periodicity and
%   offset specified by the input RSPERIOD. RSPERIOD can be 'on', 'off', or
%   a tuple [TRS,TOFF], where TRS is the periodicity and TOFF the offset of
%   the RS in slots. When RSPERIOD = 'on', TRS = 1 and TOFF = 0. When
%   RSPERIOD = 'off', TRS = 0 and TOFF = 0. For more information about
%   RSPERIOD, see the configuration objects nrCSIRSConfig and nrSRSConfig.
%
%   Example:
%   rsPeriod = 'on'; 
%   [Trs,Toff] = nr5g.internal.getRSPeriodicityAndOffset(rsPeriod)
%

%  Copyright 2019-2020 The MathWorks, Inc.

%#codegen
    
    Trs = 1;
    Toff = 0;
    if isnumeric(RSPeriod)
        Trs = double(RSPeriod(1));
        Toff = double(RSPeriod(2));
    elseif strcmpi(RSPeriod,'off')
        Trs = 0;
    end
end