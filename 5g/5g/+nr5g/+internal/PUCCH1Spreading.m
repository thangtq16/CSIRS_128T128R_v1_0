function seq = PUCCH1Spreading(nSF,occi)
% PUCCH1Spreading PUCCH format 1 block-wise spread orthogonal sequence
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%    seq = nr5g.internal.PUCCH1Spreading(NSF,OCCI) provides the block-wise
%    spreading orthogonal sequence of the PUCCH format 1 for the inputs,
%    spreading factor NSF and orthogonal cover code index OCCI. The input
%    NSF can be any integer value in between 1 and 7 and OCCI can be a
%    value between 0 and 6, less than NSF.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    % Get the table of phase values based on nSF
    if nSF == 1
        phi = 0;
    elseif nSF == 2
        phi = [0 0;
               0 1];
    elseif nSF == 3
        phi = [0 0 0;
               0 1 2;
               0 2 1];
    elseif nSF == 4
        phi = [0 0 0 0;
               0 2 0 2;
               0 0 2 2;
               0 2 2 0];
    elseif nSF == 5
        phi = [0 0 0 0 0;
               0 1 2 3 4;
               0 2 4 1 3;
               0 3 1 4 2;
               0 4 3 2 1];
    elseif nSF == 6
        phi = [0 0 0 0 0 0;
               0 1 2 3 4 5;
               0 2 4 0 2 4;
               0 3 0 3 0 3;
               0 4 2 0 4 2;
               0 5 4 3 2 1];
    else % nSF is equal to 7
        phi = [0 0 0 0 0 0 0;
               0 1 2 3 4 5 6;
               0 2 4 6 1 3 5;
               0 3 6 2 5 1 4;
               0 4 1 5 2 6 3;
               0 5 3 1 6 4 2;
               0 6 5 4 3 2 1];
    end

    % Get the block-wise spreading orthogonal sequence
    seq = exp(1i*2*pi.*phi(double(occi)+1,:)/double(nSF));

end
