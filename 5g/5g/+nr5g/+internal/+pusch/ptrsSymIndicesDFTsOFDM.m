function [symIndex,puschSymInd] = ptrsSymIndicesDFTsOFDM(symbolset,dmrssymbolset,ptrssymbolset)
%ptrsSymIndicesDFTsOFDM Provides the PT-RS symbol indices
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [SYMINDEX,PUSCHSYMIND] = ptrsSymIndicesDFTsOFDM(SYMBOLSET,DMRSSYMBOLSET,PTRSSYMBOLSET)
%   returns the PT-RS symbol indices SYMINDEX, relative to the PUSCH symbol
%   indices when transform precoding is enabled. It also provides the PUSCH
%   OFDM symbol indices PUSCHSYMIND, excluding DM-RS symbol set.

%   Copyright 2019 The MathWorks, Inc.

%#codegen

    % Get symbol indices of PUSCH in a slot, excluding PUSCH DM-RS
    logicalMatrix = repmat(symbolset(:),1,numel(dmrssymbolset)) == repmat(reshape(dmrssymbolset,1,[]),numel(symbolset),1);
    puschSymInd = symbolset(~sum(logicalMatrix,2));

    % Get the PT-RS symbol indices such that the indices are used
    % directly in PUSCH transform precoding
    flag = false(1,length(puschSymInd));
    [~,indexVal] = sort(puschSymInd);
    for i = 1:length(ptrssymbolset)
        flag = flag | (puschSymInd==ptrssymbolset(i));
    end
    symIndex = indexVal(flag)-1;

end
