function out = averageColumn(in,N)
%AVERAGECOLUMN performs averaging on a column.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen
    
    out = in;

    uN = unique(N);
    
    for i = 1:numel(uN)

        Nthis = uN(i);
        ind = find(N==Nthis);

        % Convolve each channel of 'in' with an N-by-1 vector of ones
        h = ones([Nthis 1]);
        m = convmtx(h,size(in,1));
        outTemp = pagemtimes(m,in(:,:,ind));

        % Retain central rows of output to make the number of rows the same as
        % the input, removing other rows
        M = (Nthis - 1) / 2;
        outTemp = outTemp(M+1:end-M,:,:);

        % Create and apply scaling matrix 'nv' which normalizes by the number
        % of samples averaged in each element of 'out'
        nv = sum(m,2);
        nv = nv(M+1:end-M,:);
        outTemp = outTemp./nv;

        out(:,:,ind) = outTemp(:,:,:);

    end
    
end