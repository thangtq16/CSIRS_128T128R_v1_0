function [NIRB,NPRB,Mrb] = interlacedResourceBlockSet(carrier,channel)
%interlacedResourceBlockSet interlaced resource block indices
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   The available RBs for PUSCH/PUCCH are determined from the intersection
%   of the RBs defined by the interlace index (interlace0 and interlace1)
%   and the RB set defined by rb-SetIndex (TS 38.213 Section 9.2.1). The RB
%   set pointed by rb-SetIndex is defined indirectly by the intracell
%   guard bands in the carrier.

%   Copyright 2023-2024 The MathWorks, Inc.

%#codegen

    if ~isempty(channel.NStartBWP)
        nStartBWP = channel.NStartBWP(1);
    else
        nStartBWP = carrier.NStartGrid(1);
    end

    if ~isempty(channel.NSizeBWP)
        nSizeBWP = channel.NSizeBWP(1);
    else
        nSizeBWP = carrier.NSizeGrid(1);
    end

    % Guard band sizes
    gb = nr5g.internal.interlacing.guardBandSize(carrier);

    % Number of interlaces
    M = 10*(15/carrier.SubcarrierSpacing);

    % Limit the 0-based RB set index to the maximum number of guard bands
    rbSetIdx = unique(min(channel.RBSetIndex,size(gb,1))+1);

    % Calculate the initial set of RB indices within the interlace (nIRB)
    % from the RB set pointed by RBSetIndex. These nIRB indices may include
    % elements that are not available after intersecting the corresponding
    % CRB set with the available RB set.
    guardband = [0 0; gb; carrier.NSizeGrid 0];
    guardband(:,1) = guardband(:,1)+carrier.NStartGrid; % From CRB0
    prevGuardBand = guardband(rbSetIdx,:);
    nextGuardBand = guardband(rbSetIdx+1,:);
    crbrange = [sum(prevGuardBand,2) nextGuardBand(:,1)-1];
    nIRBRange = floor((crbrange-nStartBWP)/M);
    
    % Initialize outputs and temporary variables
    coder.varsize('nIRB','nCRB','NCRB',[Inf Inf],[1 1]);
    numRBSets = size(nIRBRange,1);
    NCRB = zeros(0,1);

    m = reshape(unique(channel.InterlaceIndex),[],1);

    % For each RB set, find the NIRB and CRB sets of to each interlace
    for i = 1:numRBSets
        % Initial NIRB set
        nIRB = nIRBRange(i,1):nIRBRange(i,2);

        % Calculate the CRB set for the previous nIRB set and the
        % interlaces configured.
        nCRB = reshape(M*nIRB + nStartBWP + mod(m-nStartBWP,M),[],1);

        % Remove RBs out of the range
        nCRB(nCRB < crbrange(i,1) | nCRB > crbrange(i,2)) = [];

        NCRB = [NCRB; nCRB(:)]; %#ok<AGROW>
    end

    % Calculate PRB from CRB and NIRB, limited to the BWP.
    prb = sort(NCRB(:).' - nStartBWP);
    NPRB = prb(0 <= prb & prb < nSizeBWP);
    NIRB = floor(NPRB/M);

    % Number of RB used for low PAPR sequence generation. This might be
    % different from the number of RB used for transmission if interlacing
    % is enabled for PUCCH formats 0 and 1.
    Mrb = length(NPRB);
    formatPUCCH = nr5g.internal.pucch.getPUCCHFormat(channel);
    if Mrb~=0 && ~isempty(formatPUCCH) && any(formatPUCCH==[0 1])
        Mrb = 1;
    end

end