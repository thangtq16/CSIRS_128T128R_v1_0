function [prsStruct,PRSPresence,numRes,nSlot] = validateAndSchedulePRS(carrier,prsObj)
% validateAndSchedulePRS Validates the inputs and returns the presence of
% all the PRS resources within a PRS resource set
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%    [PRSSTRUCT,PRSPRESENCE,NUMRES,NSLOT] = nr5g.internal.validateAndSchedulePRS(CARRIER,PRSOBJ)
%    validates the carrier specific configuration object CARRIER and
%    positioning reference signal configuration object PRSOBJ. The function
%    returns a configuration structure PRSSTRUCT containing the validated
%    and scalar expanded PRS parameters. The function also returns the
%    state (present/absent in the operating slot, as a binary vector) of
%    all the PRS resources within a PRS resource set PRSPRESENCE, the
%    number of configured PRS resources NUMRES, and the relative slot
%    number NSLOT.

%  Copyright 2020 The MathWorks, Inc.

%#codegen

    % Validate inputs
    prsStruct = validateInputs(carrier,prsObj);

    % Extract the following properties of carrier and cast the necessary
    % ones to double
    nFrame     = double(carrier.NFrame);
    nSlot      = double(carrier.NSlot);
    NFrameSlot = carrier.SlotsPerFrame;
    % Get the relative slot number and relative frame number
    [nSlot,nFrame] = nr5g.internal.getRelativeNSlotAndSFN(nSlot,nFrame,NFrameSlot);

    % Get the number of PRS resources configured in the PRS resource set
    numRes = numel(prsStruct.PRSResourceOffset);
    % Turn off all the PRS resources
    PRSPresence = zeros(1,numRes);

    % Make all the PRS resources active, if PRSResourceSetPeriod is 'on'
    if ischar(prsStruct.PRSResourceSetPeriod)
        if strcmpi(prsStruct.PRSResourceSetPeriod,'on')
            PRSPresence = ones(1,numRes);
        end
        return;
    end

    % Extract the PRS slot configuration properties
    TPRSPer      = prsStruct.PRSResourceSetPeriod(1);
    TPRSOffset   = prsStruct.PRSResourceSetPeriod(2);
    TPRSRep      = prsStruct.PRSResourceRepetition;
    TPRSGap      = prsStruct.PRSResourceTimeGap;
    ValidOptions = TPRSGap*(0:TPRSRep-1);

    % Extract the PRS muting configuration related properties
    b1         = prsStruct.MutingPattern1;
    TPRSMuting = prsStruct.MutingBitRepetition;
    b2         = prsStruct.MutingPattern2;
    % Length of option-1 muting bit pattern
    L = length(b1);

    % Loop over all the PRS resources and update the presence of each PRS
    % resource as per TS 38.211 Section 7.4.1.7.4
    for resIdx = 1:numRes
        TPRSOffsetRes = prsStruct.PRSResourceOffset(resIdx);
        temp = NFrameSlot*nFrame + nSlot - TPRSOffset - TPRSOffsetRes;
        PRSCheck = mod(temp,TPRSPer);
        check1 = 0;
        if any(PRSCheck == ValidOptions)
            check1 = 1;
        end

        bi1 = 1; % Bit value corresponding to option-1 muting bit pattern
        bi2 = 1; % Bit value corresponding to option-2 muting bit pattern
        if ~isempty(b1)
            % The case of option-1 muting bit pattern is configured
            i_1 = mod(floor(temp/(TPRSMuting*TPRSPer)),L);
            bi1 = b1(i_1 + 1);
        end
        if ~isempty(b2)
            % The case of option-2 muting bit pattern is configured
            i_2 = mod(floor(PRSCheck/TPRSGap),TPRSRep);
            bi2 = b2(i_2 + 1);
        end
        PRSPresence(resIdx) = check1 && bi1 && bi2;
    end
end

function prsStruct = validateInputs(carrier,prsObj)
%validateInputs Validates input configuration objects
%
%    PRSSTRUCT = validateInputs(CARRIER,PRSOBJ) validates the carrier
%    specific configuration object CARRIER and positioning reference signal
%    configuration object PRSOBJ. The function returns a configuration
%    structure PRSSTRUCT containing the validated and scalar expanded PRS
%    parameters.

    % Validate inputs
    coder.internal.errorIf(~(isa(carrier,'nrCarrierConfig') && isscalar(carrier)),...
        'nr5g:nrPRS:InvalidCarrierInput');
    coder.internal.errorIf(~(isa(prsObj,'nrPRSConfig') && isscalar(prsObj)),...
        'nr5g:nrPRS:InvalidPRSInput');

    % Validate PRS configuration object and get the structure with
    % validated and scalar expanded PRS parameters.
    prsStruct = validateConfig(prsObj);

    % Validate the PRS allocation in time-domain
    LPRS      = prsStruct.NumPRSSymbols;
    lPRSStart = prsStruct.SymbolStart;
    for resIdx = 1:numel(LPRS)
        lastSym = lPRSStart(resIdx) + LPRS(resIdx); % 1-based
        coder.internal.errorIf(lastSym > carrier.SymbolsPerSlot,...
            'nr5g:nrPRS:InvalidPRSSymbolAllocation',lPRSStart(resIdx),...
            LPRS(resIdx),resIdx,carrier.SymbolsPerSlot)
    end

    % Validate the PRS allocation in frequency-domain
    coder.internal.errorIf(prsStruct.RBOffset + prsStruct.NumRB > carrier.NSizeGrid,...
        'nr5g:nrPRS:InvalidPRSFrequencyAllocation',prsStruct.RBOffset,...
        prsStruct.NumRB,carrier.NSizeGrid);
end