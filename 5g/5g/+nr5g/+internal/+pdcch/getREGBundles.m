function [regB, f, coresetIdx] = getREGBundles(carrier,pdcch,opts)
%getREGBundles Resource-element group (REG) bundles (as 1-based linear indices)
%   for the PDCCH configuration
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   See also nrPDCCHResources, nrPDCCHSpace.

%   Copyright 2019-2022 The MathWorks, Inc.

%#codegen

    % Extract CORESET parameter object
    crst = pdcch.CORESET;

    % Get the CCE-to-REG mapping
    [f,L] = getCCEMapping(crst);

    firstSymLoc = pdcch.SearchSpace.StartSymbolWithinSlot;          % 0-based
    symIdx = double(firstSymLoc(1)) + (0:double(crst.Duration-1));  % Set of 0-based symbol indices of the CORESET instance
    
    nStartBWP = double(pdcch.NStartBWP);
    prbIdx = nr5g.internal.pdcch.getCORESETPRB(crst,nStartBWP);

    if strcmpi(opts.IndexOrientation,'carrier')
        rbOffset = nStartBWP - double(carrier.NStartGrid);
        nrb = double(carrier.NSizeGrid);
    else    % BWP
        rbOffset = 0;
        nrb = double(pdcch.NSizeBWP);
    end

    % Get the 1-based RB indices for the CORESET, using implicit expansion
    % addition of a row of OFDM symbol indices with a column of RB indices
    coresetIdx = nrb*symIdx + (1+rbOffset+prbIdx);

    rbIdxTime = reshape(coresetIdx',[],1);
    % rbIdxTime are the PRB indices: sort them for CCEs, in REGBundles.
    regB = reshape(rbIdxTime,L,[]);         % Form REG-bundles

end