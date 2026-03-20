function prbIndices = vrbToPRBInterleaver(nrb,rboffset,L)
%vrbToPRBInterleaver VRB to PRB Interleaver
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   PRBINDICES = vrbToPRBInterleaver(NRB,RBOFFSET,L) returns the
%   interleaved order of resource blocks PRBINDICES according to TS 38.211
%   Section 7.3.1.6, given the number of resource blocks NRB, the resource
%   block offset RBOFFSET and the bundle size L.
%
%   Example:
%   % Get the interleaved order of resource blocks for the number of
%   % resource blocks set to 25, resource block offset set to 0 and the
%   % bundle size set to 2.
%
%   nrb = 25;
%   rbOffset = 0;
%   L = 2;
%   prbIndices = nr5g.internal.pdsch.vrbToPRBInterleaver(nrb,rbOffset,L)

%   Copyright 2019-2022 The MathWorks, Inc.

%#codegen

    % RB offset with respect to bundle size
    rboffsetModL = mod(rboffset,L);

    % PRB bundle generation
    Nb = ceil((nrb+rboffsetModL)/L); % Number of PRB bundles

    % If only one bundle, force the indices to be [0:NRB-1]
    if Nb == 1
        prbIndices = (0:nrb-1);
    else
        % Generate the 0-based RB bundle indices
        numRBinBundle = zeros(1,Nb);
        numRBinBundle(1) = L-rboffsetModL;
        numRBinBundle(end) = mod(rboffset+nrb,L);
        if ~numRBinBundle(end)
            numRBinBundle(end) = L;
        end
        numRBinBundle(2:end-1) = L;

        % Generate the 0-based PRB bundles indices for the interleaved
        % mapping
        R = 2;
        C = floor(Nb/R);
        r = (0:R-1)';
        c = 0:C-1;
        prbbInd = repmat(r*C,1,C)+repmat(c,R,1); % Indices of the PRB bundle
        prbbInd = prbbInd(:)';
        if numel(prbbInd) ~= Nb
            prbbInd = [prbbInd Nb-1];
        else
            prbbInd(Nb) = Nb-1; % Last VRB bundle is mapped to the last PRB bundle
        end

        % Note that prbbInd(1) is 0 and prbbInd(end) is Nb-1 always by
        % design. Thus, only prbbInd(2:end-1) are actively considered.
        prbInd = (prbbInd(2:end-1).*numRBinBundle(2:end-1)) - rboffsetModL;

        % Extend the bundle indices to the actual RB indices
        prbIndices = NaN(1,nrb);
        prbIndices(1:numRBinBundle(1)) = (0:numRBinBundle(1)-1); % First bundle
        PRBindices_tmp = repmat(prbInd,L,1)+repmat((0:L-1)',1,numel(prbInd));
        prbIndices(1+numRBinBundle(1):end-numRBinBundle(end)) = PRBindices_tmp(:)'; % Bundles in the middle
        prbIndices(end-numRBinBundle(end)+1:end) = max(prbIndices)+1+(0:numRBinBundle(end)-1); % Last bundle
    end

end