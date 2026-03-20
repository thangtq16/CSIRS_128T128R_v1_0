function sym = blockWiseDespread(tfdpcde,Mrb,sf,occi)
%blockwiseDespreading Blockwise despreading for PUCCH formats 3 and 4
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    % Number of subcarriers allocated for PUCCH
    Msc = Mrb * 12;

    % Reshape input symbols into columns of length Msc
    tfdpcde = reshape(tfdpcde,Msc,[]);

    wn = nr5g.internal.pucch.blockWiseSpreadingSequence(sf,occi);
    w = repmat(wn,Msc/sf,1);

    % Multiply tfdpcde by the conjugate of wn to align symbol phases
    numCols = size(tfdpcde,2);
    aligned = tfdpcde.*conj(w(:));

    symbolsT = zeros(Msc/sf,numCols,'like',aligned);
    for i = 1:(Msc/sf)
        % Average symbols representing the same modulation symbol
        symbolsT(i,:) = mean(aligned(i:Msc/sf:Msc,:));
    end

    % Vectorize
    sym = symbolsT(:);

end