function Ho = staticChannelMatrix(Nt,Nr)
%staticChannelMatrix Channel matrix for static propagation conditions as defined in TS 38.101-4 Annex B.1.

%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023-2024 The MathWorks, Inc.

%#codegen

    H = [];
    switch Nr
        case 1
            switch Nt
                case 1
                    H = 1;
                case 2
                    H = [1 1];
                case 4
                    H = [1 1 1i 1i];
                case 8
                    H = [1 1 1 1 1i 1i 1i 1i];
            end

        case 2
            switch Nt
                case 1
                    H = [1;
                         1];
                case 2
                    H = [1  1i;
                         1 -1i];
                case 4
                    H = [1 1  1i  1i;
                         1 1 -1i -1i];
                case 8
                    H = [1 1 1 1  1i  1i  1i  1i;
                         1 1 1 1 -1i -1i -1i -1i];
            end

        case 4
            switch Nt
                case 1
                    H = ones(4,1);
                case 2
                    H = [1  1i;
                         1 -1i;
                         1  1i;
                         1 -1i];
                case 4
                    H = [1  1  1i  1i;
                         1  1 -1i -1i;
                         1 -1  1i -1i;
                         1 -1 -1i  1i];
                case 8
                    H = [1  1  1  1  1i  1i  1i  1i;
                         1  1  1  1 -1i -1i -1i -1i;
                         1  1 -1 -1  1i  1i -1i -1i;
                         1  1 -1 -1 -1i -1i  1i  1i];
            end
    end

    % If undefined, use all ones.
    if isempty(H)
        Ho = ones(Nr,Nt);
    else
        Ho = H;
    end

end
