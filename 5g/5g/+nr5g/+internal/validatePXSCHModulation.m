function out = validatePXSCHModulation(fcnName,in,ncw,modlist)
% validatePXSCHModulation validates PDSCH and PUSCH modulation order or orders
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.

%  Copyright 2023 The MathWorks, Inc.

%#codegen

    coder.internal.errorIf(~(ischar(in) || ...
        iscellstr(in) || isstring(in)) || isempty(in), ...
        'nr5g:nrPXSCH:InvalidModType');

    if (isstring(in))
        inChar = convertStringsToChars(in);
    else
        inChar = in;
    end

    if (~iscell(inChar))
        temp = {inChar};
    else
        temp = inChar;
    end

    if (numel(temp)==1 && ncw==2)
        out = repmat(temp,1,2);
    else
        out = temp;
    end

    for q = 1:numel(out)
        validatestring(out{q},modlist,fcnName,'modulation');
    end

end
