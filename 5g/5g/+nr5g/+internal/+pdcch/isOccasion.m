function [isOccasion,slotNum] = isOccasion(carrier,pdcch)
%isOccasion Return true if a PDCCH monitoring occasion exists in current slot
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   See also nrPDCCHResources, nrPDCCHSpace.

%   Reference:
%   [1] 3GPP TS 38.213, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Physical layer
%   procedures for control. Sections 10.
%
%   0-based frame and slot numbers, for the current NSlot only
%   For monitoring, start from offset for duration in slots

%   Copyright 2019-2021 The MathWorks, Inc.

%#codegen

    % Get the monitoring occasion in a slot
    slotsPerFrame = carrier.SlotsPerFrame;
    ssPeriodOffset = double(pdcch.SearchSpace.SlotPeriodAndOffset); % [ks os]
    ssDuration = double(pdcch.SearchSpace.Duration);

    % Extract time counters
    absNSlot = double(carrier.NSlot);  % Absolute slot number
    NFrame   = double(carrier.NFrame); % Absolute frame number

    % Get relative slot number and relative frame number
    [slotNum,frameNum] = nr5g.internal.getRelativeNSlotAndSFN( ...
        absNSlot,NFrame,carrier.SlotsPerFrame);

    ssPeriod = ssPeriodOffset(1);

    % Slot number measured from SS slot offset + d, where d =
    % (0...ssDuration-1). SS slot offset + d is limited to the SS period.
    slotOffsets = ssPeriodOffset(2)+(0:ssDuration-1);
    slots = frameNum*slotsPerFrame+slotNum-slotOffsets(slotOffsets<ssPeriod);
    
    % If any of these slot numbers are a multiple of the SS period, this is
    % a PDCCH monitoring occasion.
    isOccasion = any(mod(slots, ssPeriod) == 0);

end
