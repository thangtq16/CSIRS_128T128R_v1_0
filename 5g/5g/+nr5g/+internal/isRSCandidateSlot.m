function out = isRSCandidateSlot(NSlot,NFrame,SlotsPerFrame,RSPeriod)
% isRSCandidateSlot indicates if a slot is a candidate for transmission of
% a reference signal (RS).
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%   out = isRSCandidateSlot(NSLOT,NFRAME,SLOTSPERFRAME,RSPERIOD)
%   returns true if the frame and slot specified by system frame number
%   NFRAME and slot number NSLOT, respectively, can contain an RS with
%   periodicity and offset RSPERIOD for a frame with SLOTSPERFRAME slots
%   per frame. RSPERIOD can be 'on', 'off', or a tuple [TRS,TOFF], where
%   TRS if the periodicity and TOFF the offset of the RS in slots. For more
%   information, see TS 38.211 Section 6.4.1.4.4.
%
%   Example:
%   rsPeriod = [2 1]; % periodicity 2 slots, offset 1 slot.
%   out = nr5g.internal.isRSCandidateSlot(3,0,rsPeriod)

%  Copyright 2019-2020 The MathWorks, Inc.

%#codegen

    [Trs,Toff] = nr5g.internal.getRSPeriodicityAndOffset(RSPeriod);
    
    [NSlot, NFrame] = nr5g.internal.getRelativeNSlotAndSFN(double(NSlot),double(NFrame),SlotsPerFrame);
    
    % Not a RS candidate slot if the period is 0 or not the right offset-period pair
    out = ~( Trs == 0 || mod(SlotsPerFrame*NFrame + NSlot - Toff, Trs) );
end