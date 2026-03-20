function sym = lowPAPRSequence(dmrs,nsc,prbset,nslot,nsymbol,ldash,symbperslot)
%lowPAPRSequence Low-PAPR based DM-RS sequences for transform precoding
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%

%   Copyright 2019-2022 The MathWorks, Inc.

%#codegen

    if ~isempty(prbset)
        [minprb,maxprb] = bounds(prbset);
        % Note that in the transform precoding case, DM-RS config type = 1, which is 6 DM-RS SC per 12 PRB SC (ever other subcarrier)
        % so nsc will nominally be 6
        mzc = nsc*(maxprb-minprb+1);      % Required (minimum) length of contiguous sequence to cover the PRB

        % Adjust active symbol number for ldash (second double symbol is the same as the first)
        nsymbol = nsymbol-ldash;

        if ~isfield(dmrs,'Type') || dmrs.Type == 1
            % Get low PAPR sequence u,v parameters for the DM-RS, using the NRSID and sequence/group hopping state contained in 'dmrs'
            [u,v] = nr5g.internal.pusch.getHoppingParameters(dmrs,mzc,nslot,nsymbol,symbperslot);

            alpha = 0;
            sym = reshape(nrLowPAPRS(u(1),v(1),alpha,mzc),nsc,[]);    % Type 1 low PAPR sequence, SEQ = nrLowPAPRS(U,V,ALPHA,M)
        else
            % Select active scrambling ID
            nidselected = min(dmrs.NSCID+1,length(dmrs.NIDNSCID));
            
            % Cache the scrambling IDs
            nidnscid = double(dmrs.NIDNSCID(nidselected));
            nscid = double(dmrs.NSCID);
                        
            % If dmrs.NRSID is empty at this point then use nidnscid, for the hopping parameters (type 2 behaviour, specifically)
            if isempty(dmrs.NRSID)
                dmrs.NRSID = nidnscid;
            end

            % Get low PAPR sequence u parameter for the DM-RS, using the NRSID and sequence/group hopping state contained in 'dmrs'
            u = nr5g.internal.pusch.getHoppingParameters(dmrs,mzc,nslot,nsymbol,symbperslot); 

            % Calculate the cinit for the type 2 sequence
            cinit = mod(2^17*(symbperslot*nslot + nsymbol + 1)*(2*nidnscid + 1) + 2*nidnscid + nscid,2^31);
            sym = reshape(nrLowPAPRS(u(1),cinit,mzc),nsc,[]);    % Type 2 low PAPR sequence, SEQ = nrLowPAPRS(U,CINIT,M) 
        end

        % Extract associated DM-RS symbols for the PRB and turn into a
        % column
        sym = reshape(sym(:,prbset-minprb+1),[],1);
    else
        sym = complex([]);
    end
end
