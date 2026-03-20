function [cp,groupHopping,freqHopping,Mrb,optargs] = ...
    validatePUCCHInputs(ack,sr,symAllocation,cp,nslot,nid,groupHopping,initialCS,freqHopping,fcnName,varargin)
% validatePUCCHInputs Validate common inputs of PUCCH format 0 and 1
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.

%  Copyright 2018-2023 The MathWorks, Inc.

%#codegen

    % Check ack
    if ~(isnumeric(ack) && all(size(ack)==0))
        validateattributes(ack,{'numeric','logical'},{'column','binary'},fcnName,'ACK');
    end
    lenACK = length(ack);
    coder.internal.errorIf(lenACK>2,'nr5g:nrPUCCH:InvalidACKLength',lenACK);

    % Check sr
    if ~((isnumeric(sr) || islogical(sr)) && isempty(sr))
        validateattributes(sr,{'numeric','logical'},{'scalar','binary'},fcnName,'SR');
    end

    % Check symAllocation
    validateattributes(symAllocation,{'numeric'},{'real','vector','integer','nonnegative'},fcnName,'SYMALLOCATION');
    coder.internal.errorIf(numel(symAllocation)~=2,'nr5g:nrPUCCH:InvalidSymAllocationLength',numel(symAllocation));

    symStart = double(symAllocation(1));    % OFDM symbol index corresponding to start of PUCCH
    nPUCCHSym = double(symAllocation(2));   % Number of symbols allocated for PUCCH

    if strcmpi(fcnName,'nrPUCCH0')
        coder.internal.errorIf((nPUCCHSym < 1) || (nPUCCHSym > 2),'nr5g:nrPUCCH:InvalidNumSymbolsPUCCH0',nPUCCHSym);
    else
        coder.internal.errorIf((nPUCCHSym < 4) || (nPUCCHSym > 14),'nr5g:nrPUCCH:InvalidNumSymbolsPUCCH1',nPUCCHSym);
    end

    % Check cp
    cp = validatestring(cp,{'normal','extended'},fcnName,'CP');
    if strcmpi(cp,'extended')
        % Check the number of symbols in a slot
        coder.internal.errorIf((symStart+nPUCCHSym)>12,'nr5g:nrPUCCH:SymAllocationSumExceed',symStart,nPUCCHSym,12);
    else
        % Check the number of symbols in a slot
        coder.internal.errorIf((symStart+nPUCCHSym)>14,'nr5g:nrPUCCH:SymAllocationSumExceed',symStart,nPUCCHSym,14);
    end

    % Check nslot
    validateattributes(nslot,{'numeric'},{'scalar','integer','nonnegative','<=',159},fcnName,'NSLOT');

    % Check nid
    validateattributes(nid,{'numeric'},{'scalar','integer','nonnegative','<=',1023},fcnName,'NID');

    % Check groupHopping
    groupHopping = validatestring(groupHopping,{'neither','enable','disable'},fcnName,'GROUPHOPPING');

    % Check initialCS
    validateattributes(initialCS,{'numeric'},{'scalar','integer','nonnegative','<=',11},fcnName,'INITIALCS');

    % Check freqHopping
    freqHopping = validatestring(freqHopping,{'enabled','disabled'},fcnName,'FREQHOPPING');

    % Check Mrb and potential name-value pairs
    Mrb = 1; % Set default single-PRB allocation for PUCCH format 0 and 1
    firstoptarg = 1; % Set name-value pair to be the first optional input by default
    if nargin>10
        if isnumeric(varargin{1})
            % Mrb is an input
            Mrb = varargin{1};
            validateattributes(Mrb,{'numeric'},{'scalar',...
                'real','positive','integer'},fcnName,'MRB');
            firstoptarg = 2;
        end
    end
    optargs = {varargin{firstoptarg:end}};
end