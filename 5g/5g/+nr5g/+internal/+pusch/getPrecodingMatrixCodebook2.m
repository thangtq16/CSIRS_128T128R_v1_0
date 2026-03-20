function allW = getPrecodingMatrixCodebook2(nlayers)
%getPrecodingMatrixCodebook2 Compute all precoding matrices for PUSCH
% codebook transmission codebook2, according to Tables 6.3.1.5-25 to
% 6.3.1.5-36 of TS 38.211
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023-2024 The MathWorks, Inc.

%#codegen

    fcnName = "initializeW" + string(nlayers) + "8cb2";
    fcn = str2func(fcnName);
    allW = fcn();

end

% Submatrices Wbar
function Wbar_i = Wbar1_cb2(i)

    % Table 6.3.1.5-25
    %                         i
    Wbar = [1   1   1   1; %  0
            1   1  1j  1j; %  1
            1   1  -1  -1; %  2
            1   1 -1j -1j; %  3
            1  1j   1  1j; %  4
            1  1j  1j  -1; %  5
            1  1j  -1 -1j; %  6
            1  1j -1j   1; %  7
            1  -1   1  -1; %  8
            1  -1  1j -1j; %  9
            1  -1  -1   1; %  10
            1  -1 -1j  1j; %  11
            1 -1j   1 -1j; %  12
            1 -1j  1j   1; %  13
            1 -1j  -1  1j; %  14
            1 -1j -1j  -1; %  15
            ];
    Wbar = Wbar / 2;

    % Return the Wbar matrix associated to TPMI.
    % The output matrix has 8 columns, as described in TS 38.211
    idx = getMatrixIndex(1,i);
    Wbar_i = Wbar(idx,:).';
end

function Wbar_i = Wbar2_cb2(i)

    % Table 6.3.1.5-26
    %                         i
    Wbar = [1   1   1   1; %  0
            1   1  -1  -1;
            1   1  1j  1j; %  1
            1   1 -1j -1j;
            1  1j   1  1j; %  2
            1  1j  -1 -1j;
            1  1j  1j  -1; %  3
            1  1j -1j   1;
            1  -1   1  -1; %  4
            1  -1  -1   1;
            1  -1  1j -1j; %  5
            1  -1 -1j  1j;
            1 -1j   1 -1j; %  6
            1 -1j  -1  1j;
            1 -1j  1j   1; %  7
            1 -1j -1j  -1;
            ];
    Wbar = Wbar / (2*sqrt(2));

    % Return the Wbar matrix associated to TPMI.
    % The output matrix has 8 columns, as described in TS 38.211
    idx = getMatrixIndex(2,i);
    Wbar_i = Wbar(idx,:).';
end

function Wbar_i = Wbar3_cb2(i)

    % Table 6.3.1.5-27
    %                         i
    Wbar = [1   1   1   1; %  0
            1  -1   1  -1;
            1   1  -1  -1;
            1   1  1j  1j; %  1
            1  -1  1j -1j;
            1   1 -1j -1j;
            1  -1   1  -1; %  2
            1   1   1   1;
            1  -1  -1   1;
            1  -1  1j -1j; %  3
            1   1  1j  1j;
            1  -1 -1j  1j;
            ];
    Wbar = Wbar / (2*sqrt(3));

    % Return the Wbar matrix associated to TPMI.
    % The output matrix has 8 columns, as described in TS 38.211
    idx = getMatrixIndex(3,i);
    Wbar_i = Wbar(idx,:).';
end

function Wbar_i = Wbar4_cb2(i)

    % Table 6.3.1.5-28
    %                         i
    Wbar = [1   1   1   1; %  0
            1  -1   1  -1;
            1   1  -1  -1;
            1  -1  -1   1;
            1   1  1j  1j; %  1
            1  -1  1j -1j;
            1   1 -1j -1j;
            1  -1 -1j  1j;
            ];
    Wbar = Wbar / 4;

    % Return the Wbar matrix associated to TPMI.
    % The output matrix has 8 columns, as described in TS 38.211
    idx = getMatrixIndex(4,i);
    Wbar_i = Wbar(idx,:).';
end

% Matrices Wprime
function W18_cb2 = initializeW18cb2()

    % Table 6.3.1.5-29

    nlayers = 1;
    numTPMI = 33;
    W18_cb2 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:15
        W18_cb2(idx(tpmi),:) = [Wbar1_cb2(tpmi)
                                zeros(4,1)].';
    end
    for tpmi = 16:31
        W18_cb2(idx(tpmi),:) = [zeros(4,1)
                                Wbar1_cb2(tpmi-16)].';
    end
    W18_cb2 = W18_cb2 / sqrt(2);
    tpmi = 32;
    W18_cb2(idx(tpmi),:) = [1   1   1   1   1   1   1  1] / (2*sqrt(2));
end

function W28_cb2 = initializeW28cb2()

    % Table 6.3.1.5-30

    nlayers = 2;
    numTPMI = 272;
    W28_cb2 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:7
        W28_cb2(idx(tpmi),:) = [Wbar2_cb2(tpmi)
                                zeros(4,2)].';
    end
    for tpmi = 8:15
        W28_cb2(idx(tpmi),:) = [zeros(4,2)
                                Wbar2_cb2(tpmi-8)].';
    end
    for tpmi = 16:271
        W28_cb2(idx(tpmi),:) = [Wbar1_cb2(floor((tpmi-16)/16)) zeros(4,1)
                                zeros(4,1)                     Wbar1_cb2(mod(tpmi,16))].';
    end
    W28_cb2 = W28_cb2 / sqrt(2);
end

function W38_cb2 = initializeW38cb2()

    % Table 6.3.1.5-31

    nlayers = 3;
    numTPMI = 264;
    W38_cb2 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:3
        W38_cb2(idx(tpmi),:) = [Wbar3_cb2(tpmi)
                                zeros(4,3)].';
    end
    for tpmi = 4:7
        W38_cb2(idx(tpmi),:) = [zeros(4,3)
                                Wbar3_cb2((tpmi-4))].';
    end
    for tpmi = 8:135
        W38_cb2(idx(tpmi),:) = [Wbar1_cb2(floor((tpmi-8)/8)) zeros(4,2)
                                zeros(4,1)                   Wbar2_cb2(mod(tpmi,8))].';
    end
    for tpmi = 136:263
        W38_cb2(idx(tpmi),:) = [Wbar2_cb2(floor((tpmi-136)/16)) zeros(4,1)
                                zeros(4,2)                      Wbar1_cb2(mod(tpmi-136,16))].';
    end
    W38_cb2 = W38_cb2 / sqrt(2);
end

function W48_cb2 = initializeW48cb2()

    % Table 6.3.1.5-32

    nlayers = 4;
    numTPMI = 68;
    W48_cb2 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:1
        W48_cb2(idx(tpmi),:) = [Wbar4_cb2(tpmi)
                                zeros(4,4)].';
    end
    for tpmi = 2:3
        W48_cb2(idx(tpmi),:) = [zeros(4,4)
                                Wbar4_cb2((tpmi- 2) )].';
    end
    for tpmi = 4:67
        W48_cb2(idx(tpmi),:) = [Wbar2_cb2(floor((tpmi-4)/8)) zeros(4,2)
                                zeros(4,2)                   Wbar2_cb2(mod(tpmi-4,8))].';
    end
    W48_cb2 = W48_cb2 / sqrt(2);
end

function W58_cb2 = initializeW58cb2()

    % Table 6.3.1.5-33

    nlayers = 5;
    numTPMI = 32;
    W58_cb2 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:31
        W58_cb2(idx(tpmi),:) = [Wbar2_cb2(floor(tpmi/4)) zeros(4,3)
                                zeros(4,2)               Wbar3_cb2(mod(tpmi,4))].';
    end
    W58_cb2 = W58_cb2 / sqrt(2);
end

function W68_cb2 = initializeW68cb2()

    % Table 6.3.1.5-34

    nlayers = 6;
    numTPMI = 16;
    W68_cb2 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:15
        W68_cb2(idx(tpmi),:) = [Wbar3_cb2(floor(tpmi/4)) zeros(4,3)
                                zeros(4,3)               Wbar3_cb2(mod(tpmi,4))].';
    end
    W68_cb2 = W68_cb2 / sqrt(2);
end

function W78_cb2 = initializeW78cb2()

    % Table 6.3.1.5-35

    nlayers = 7;
    numTPMI = 8;
    W78_cb2 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:7
        W78_cb2(idx(tpmi),:) = [Wbar3_cb2(floor(tpmi/2)) zeros(4,4)
                                zeros(4,3)               Wbar4_cb2(mod(tpmi,2))].';
    end
    W78_cb2 = W78_cb2 / sqrt(2);
end

function W88_cb2 = initializeW88cb2()

    % Table 6.3.1.5-36

    nlayers = 8;
    numTPMI = 4;
    W88_cb2 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:3
        W88_cb2(idx(tpmi),:) = [Wbar4_cb2(floor(tpmi/2)) zeros(4,4)
                                zeros(4,4)               Wbar4_cb2(mod(tpmi,2))].';
    end
    W88_cb2 = W88_cb2 / sqrt(2);
end

function idx = getMatrixIndex(nrows,tpmi)
    % Get the indices of the matrix, given the number of rows of the matrix
    % for each TPMI and the TPMI index. NROWS is the number of layers for
    % TS 38.211 Tables 6.3.1.5-29 to 6.3.1.5-36. NROWS is the number of
    % columns for Tables 6.3.1.5-25 to 6.3.1.5-28.
    idx = tpmi*nrows+(1:nrows);
end