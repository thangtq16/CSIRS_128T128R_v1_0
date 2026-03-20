function Aout = foldMultiplePRG(Ain,nPRG)
%FOLDMULTIPLEPRG folds an array Ain containing date for multiple PRGs in to
%multiple pages (with an extra dimension) where each page corresponds to a
%single PRG.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen

    nCol = size(Ain,2);

    if nCol==1
        Aout = reshape(Ain,[],1,nPRG);
    else
        Aout = permute(reshape(permute(Ain,[2 1]),nCol,[],nPRG),[2 1 3]);
    end

end