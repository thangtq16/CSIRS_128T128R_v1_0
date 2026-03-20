function [transmittedCB,cbgtiOut,cb2cbg] = getCodeblockIndices(ncb,cbgti)
% getCodeblockIndices gets the active code block indices associated
% with the input CBGTI bitmap and number of code blocks
%
% Inputs:
%   ncb    - Number of CBs
%   cbgti  - Code block transmission information (CBGTI), a binary column
%            vector
%
% Output:
%   transmittedCB - 1-based CB index of the transmitted CBs
%   cbgtiOut      - CBGTI where bits corresponding to the non-existing CBGs
%                   have been set to 0
%   cb2cbg        - 1-based CBG indices of all CBs
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2024 The MathWorks, Inc.

%#codegen

    % Get number of CBGs
    m = min(ncb,numel(cbgti));

    % If CBGTI is longer than necessary, set the trailing bits to 0 and
    % ignore them
    cbgtiOut = cbgti;
    cbgtiOut(m+1:end) = 0;
    cbgti = cbgti(1:m);

    % Calculate CBG sizes
    m1 = mod(ncb,m);   % Number of larger CBGs (the first m1 CBGs)
    k1 = ceil(ncb/m);  % Size of larger CBGs (if required)
    k2 = floor(ncb/m); % Size of smaller CBGs

    % Map the larger CBGs to CBs
    cbti1 = reshape(repmat(cbgti(1:m1),1,k1)',[],1);

    % Map the smaller CBGs to CBs
    cbti2 = reshape(repmat(cbgti(m1+1:end),1,k2)',[],1);

    % Combine all CBs to find the transmitted CBs
    cbti = [cbti1;cbti2];
    transmittedCB = find(cbti==1);

    % Construct CB to CBG map
    cbmap1 = reshape(repmat((1:m1)',1,k1)',[],1);
    cbmap2 = reshape(repmat((m1+1:m)',1,k2)',[],1);
    cb2cbg = [cbmap1;cbmap2];

end