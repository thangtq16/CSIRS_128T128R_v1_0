function [pucch, isSeqHopValid] = validatePUCCHSequenceHopping(pucch,formatPUCCH,Mrb)
%validatePUCCHSequenceHopping Validate use of sequence hopping for PUCCH
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   The symbols for PUCCH formats 0 and 1 and the DM-RS symbols for PUCCH
%   formats 3 and 4 are generated from a low-PAPR sequence type 1. For this
%   sequence, sequence hopping (i.e., GroupHopping set to 'disable') can be
%   enabled only if the number of subcarriers allocated for PUCCH is
%   greater than or equal to 72, as defined in Section 5.2.2 of TS 38.211.
%   For interlaced PUCCH formats 0 and 1, Mrb = 1.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    isSeqHopValid = true;

    % For interlaced transmissions, Mrb = 1 for formats 0 and 1.
    if any(formatPUCCH == [0 1]) && pucch.Interlacing
        mrb = 1;
    else
        mrb = Mrb;
    end

    if (formatPUCCH ~= 2) && strcmpi(pucch.GroupHopping,'disable') && mrb < 6
        pucch.GroupHopping = 'neither';
        isSeqHopValid = false;
    end
end