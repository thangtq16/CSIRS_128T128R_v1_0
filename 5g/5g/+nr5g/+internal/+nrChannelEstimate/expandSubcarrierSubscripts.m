function subsOut = expandSubcarrierSubscripts(subsIn,prgStarts,cols)
%EXPANDSUBCARRIERSUBSCRIPTS expand the relative subcarrier subscripts in
%one single PRG into absolute subcarrier subscripts in the grid.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen

    % By default, shift all columns
    if nargin<3
        cols = size(subsIn,2);
    end

    % Number of PRGs and number of subcarriers in each PRG
    nPRG = numel(prgStarts);
    nSCPerPRG = size(subsIn,1);

    % Subcarrier subscript shift for multiple PRGs. Note that subsIn is
    % relative to a single PRG, hence in the rage [1,prgSize]
    scShift = prgStarts-1;
    scShift = reshape((repmat(scShift,[1,nSCPerPRG]))',[],1);
    scShift = repmat(scShift,[1,size(subsIn,2)]);

    % Shift the nominated columns of subs
    subsOut = repmat(subsIn,[nPRG,1]);
    ind = zeros(1,size(subsIn,2));
    ind(cols) = 1;
    scShift = scShift*diag(ind);
    subsOut = subsOut+scShift;

end