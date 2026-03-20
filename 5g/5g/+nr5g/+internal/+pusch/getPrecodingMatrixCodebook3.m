function allW = getPrecodingMatrixCodebook3(nlayers)
%getPrecodingMatrixCodebook3 Compute all precoding matrices for PUSCH
% codebook transmission codebook3, according to Tables 6.3.1.5-37 to
% 6.3.1.5-46 of TS 38.211
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023-2024 The MathWorks, Inc.

%#codegen

    fcnName = "initializeW" + string(nlayers) + "8cb3";
    fcn = str2func(fcnName);
    allW = fcn();

end

% Submatrices Wbar
function Wbar_i = Wbar1_cb3(i)

    % Table 6.3.1.5-37
    %                 i
    Wbar = [1   1; %  0
            1  -1; %  1
            1  1j; %  2
            1 -1j; %  3
            ];
    Wbar = Wbar / sqrt(2);

    % Return the Wbar matrix associated to TPMI.
    % The output matrix has 8 columns, as described in TS 38.211
    idx = getMatrixIndex(1,i);
    Wbar_i = Wbar(idx,:).';
end

function Wbar_i = Wbar2_cb3(i)

    % Table 6.3.1.5-38
    %                 i
    Wbar = [1   1; %  0
            1  -1;
            1  1j; %  1
            1 -1j;
            ];
    Wbar = Wbar / 2;

    % Return the Wbar matrix associated to TPMI.
    % The output matrix has 8 columns, as described in TS 38.211
    idx = getMatrixIndex(2,i);
    Wbar_i = Wbar(idx,:).';
end

% Matrices Wprime
function W18_cb3 = initializeW18cb3()

    % Table 6.3.1.5-39

    nlayers = 1;
    numTPMI = 17;
    W18_cb3 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:3
        W18_cb3(idx(tpmi),:) = [Wbar1_cb3(tpmi)
                                zeros(2,1)
                                zeros(2,1)
                                zeros(2,1)].';
    end
    for tpmi = 4:7
        W18_cb3(idx(tpmi),:) = [zeros(2,1)
                                Wbar1_cb3((tpmi- 4))
                                zeros(2,1)
                                zeros(2,1)].';
    end
    for tpmi = 8:11
        W18_cb3(idx(tpmi),:) = [zeros(2,1)
                                zeros(2,1)
                                Wbar1_cb3((tpmi-8))
                                zeros(2,1)].';
    end
    for tpmi = 12:15
        W18_cb3(idx(tpmi),:) = [zeros(2,1)
                                zeros(2,1)
                                zeros(2,1)
                                Wbar1_cb3((tpmi- 12))].';
    end
    W18_cb3 = W18_cb3 / 2;
    tpmi = 16;
    W18_cb3(idx(tpmi),:) = [1   1   1   1   1   1   1  1] / (2*sqrt(2));
end

function W28_cb3 = initializeW28cb3()

    % Table 6.3.1.5-40

    nlayers = 2;
    numTPMI = 105;
    W28_cb3 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:1
        W28_cb3(idx(tpmi),:) = [Wbar2_cb3(tpmi)
                                zeros(2,2)
                                zeros(2,2)
                                zeros(2,2)].';
    end
    for tpmi = 2:3
        W28_cb3(idx(tpmi),:) = [zeros(2,2)
                                Wbar2_cb3((tpmi- 2))
                                zeros(2,2)
                                zeros(2,2)].';
    end
    for tpmi = 4:5
        W28_cb3(idx(tpmi),:) = [zeros(2,2)
                                zeros(2,2)
                                Wbar2_cb3((tpmi-4))
                                zeros(2,2)].';
    end
    for tpmi = 6:7
        W28_cb3(idx(tpmi),:) = [zeros(2,2)
                                zeros(2,2)
                                zeros(2,2)
                                Wbar2_cb3((tpmi-6))].';
    end
    for tpmi = 8:23
        W28_cb3(idx(tpmi),:) = [Wbar1_cb3(floor((tpmi-8)/4)) zeros(2,1)
                                zeros(2,1)                   Wbar1_cb3(mod(tpmi,4))
                                zeros(2,1)                   zeros(2,1)
                                zeros(2,1)                   zeros(2,1)].';
    end
    for tpmi = 24:39
        W28_cb3(idx(tpmi),:) = [Wbar1_cb3(floor((tpmi-24)/4)) zeros(2,1)
                                zeros(2,1)                    zeros(2,1)
                                zeros(2,1)                    Wbar1_cb3(mod(tpmi,4) )
                                zeros(2,1)                    zeros(2,1)].';
    end
    for tpmi = 40:55
        W28_cb3(idx(tpmi),:) = [Wbar1_cb3(floor((tpmi-40)/4)) zeros(2,1)
                                zeros(2,1)                    zeros(2,1)
                                zeros(2,1)                    zeros(2,1)
                                zeros(2,1)                    Wbar1_cb3(mod(tpmi,4))].';
    end
    for tpmi = 56:71
        W28_cb3(idx(tpmi),:) = [zeros(2,1)                    zeros(2,1)
                                Wbar1_cb3(floor((tpmi-56)/4)) zeros(2,1)
                                zeros(2,1)                    Wbar1_cb3(mod(tpmi,4))
                                zeros(2,1)                    zeros(2,1)].';
    end
    for tpmi = 72:87
        W28_cb3(idx(tpmi),:) = [zeros(2,1)                    zeros(2,1)
                                Wbar1_cb3(floor((tpmi-72)/4)) zeros(2,1)
                                zeros(2,1)                    zeros(2,1)
                                zeros(2,1)                    Wbar1_cb3(mod(tpmi,4))].';
    end
    for tpmi = 88:103
        W28_cb3(idx(tpmi),:) = [zeros(2,1)                    zeros(2,1)
                                zeros(2,1)                    zeros(2,1)
                                Wbar1_cb3(floor((tpmi-88)/4)) zeros(2,1)
                                zeros(2,1)                    Wbar1_cb3(mod(tpmi,4))].';
    end
    W28_cb3 = W28_cb3 / 2;
    tpmi = 104;
    W28_cb3(idx(tpmi),:) = [1   1   1   1   0   0   0  0
                            0   0   0   0   1   1   1  1] / (2*sqrt(2));
end

function W38_cb3 = initializeW38cb3()

    % Table 6.3.1.5-41

    nlayers = 3;
    numTPMI = 305;
    W38_cb3 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:7
        W38_cb3(idx(tpmi),:) = [Wbar2_cb3(floor(tpmi/4)) zeros(2,1)
                                zeros(2,2)               Wbar1_cb3(mod(tpmi,4))
                                zeros(2,2)               zeros(2,1)
                                zeros(2,2)               zeros(2,1)].';
    end
    for tpmi = 8:15
        W38_cb3(idx(tpmi),:) = [Wbar2_cb3(floor((tpmi-8)/4)) zeros(2,1)
                                zeros(2,2)                   zeros(2,1)
                                zeros(2,2)                   Wbar1_cb3(mod(tpmi,4))
                                zeros(2,2)                   zeros(2,1)].';
    end
    for tpmi = 16:23
        W38_cb3(idx(tpmi),:) = [Wbar2_cb3(floor((tpmi-16)/4)) zeros(2,1)
                                zeros(2,2)                    zeros(2,1)
                                zeros(2,2)                    zeros(2,1)
                                zeros(2,2)                    Wbar1_cb3(mod(tpmi,4))].';
    end
    for tpmi = 24:31
        W38_cb3(idx(tpmi),:) = [zeros(2,2)                    zeros(2,1)
                                Wbar2_cb3(floor((tpmi-24)/4)) zeros(2,1)
                                zeros(2,2)                    Wbar1_cb3(mod(tpmi,4))
                                zeros(2,2)                    zeros(2,1)].';
    end
    for tpmi = 32:39
        W38_cb3(idx(tpmi),:) = [zeros(2,2)                    zeros(2,1)
                                Wbar2_cb3(floor((tpmi-32)/4)) zeros(2,1)
                                zeros(2,2)                    zeros(2,1)
                                zeros(2,2)                    Wbar1_cb3(mod(tpmi,4))].';
    end
    for tpmi = 40:47
        W38_cb3(idx(tpmi),:) = [zeros(2,2)                    zeros(2,1)
                                zeros(2,2)                    zeros(2,1)
                                Wbar2_cb3(floor((tpmi-40)/4)) zeros(2,1)
                                zeros(2,2)                    Wbar1_cb3(mod(tpmi,4))].';
    end
    for tpmi = 48:111
        W38_cb3(idx(tpmi),:) = [Wbar1_cb3(floor((tpmi-48)/16)) zeros(2,1)                       zeros(2,1)
                                zeros(2,1)                     Wbar1_cb3(floor(mod(tpmi,16)/4)) zeros(2,1)
                                zeros(2,1)                     zeros(2,1)                       Wbar1_cb3(mod(tpmi,4))
                                zeros(2,1)                     zeros(2,1)                       zeros(2,1)].';
    end
    for tpmi = 112:175
        W38_cb3(idx(tpmi),:) = [Wbar1_cb3(floor((tpmi-112)/16)) zeros(2,1)                       zeros(2,1)
                                zeros(2,1)                      Wbar1_cb3(floor(mod(tpmi,16)/4)) zeros(2,1)
                                zeros(2,1)                      zeros(2,1)                       zeros(2,1)
                                zeros(2,1)                      zeros(2,1)                       Wbar1_cb3(mod(tpmi,4) )].';
    end
    for tpmi = 176:239
        W38_cb3(idx(tpmi),:) = [Wbar1_cb3(floor((tpmi-176)/16)) zeros(2,1)                       zeros(2,1)
                                zeros(2,1)                      zeros(2,1)                       zeros(2,1)
                                zeros(2,1)                      Wbar1_cb3(floor(mod(tpmi,16)/4)) zeros(2,1)
                                zeros(2,1)                      zeros(2,1)                       Wbar1_cb3(mod(tpmi,4) )].';
    end
    for tpmi = 240:303
        W38_cb3(idx(tpmi),:) = [zeros(2,1)                      zeros(2,1)                       zeros(2,1)
                                Wbar1_cb3(floor((tpmi-240)/16)) zeros(2,1)                       zeros(2,1)
                                zeros(2,1)                      Wbar1_cb3(floor(mod(tpmi,16)/4)) zeros(2,1)
                                zeros(2,1)                      zeros(2,1)                       Wbar1_cb3(mod(tpmi,4))].';
    end
    W38_cb3 = W38_cb3 / 2;
    tpmi = 304;
    W38_cb3(idx(tpmi),:) = [1  1  1  1  0  0  0  0
                            0  0  0  0  1  1  0  0
                            0  0  0  0  0  0  1  1] / (2*sqrt(2));
end

function W48_cb3 = initializeW48cb3()

    % Table 6.3.1.5-42

    nlayers = 4;
    numTPMI = 280;
    W48_cb3 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:255
        W48_cb3(idx(tpmi),:) = [Wbar1_cb3(floor(tpmi/64)) zeros(2,1)                        zeros(2,1)                       zeros(2,1)
                                zeros(2,1)                Wbar1_cb3(floor(mod(tpmi,64)/16)) zeros(2,1)                       zeros(2,1)
                                zeros(2,1)                zeros(2,1)                        Wbar1_cb3(floor(mod(tpmi,16)/4)) zeros(2,1)
                                zeros(2,1)                zeros(2,1)                        zeros(2,1)                       Wbar1_cb3(mod(tpmi,4))].';
    end
    for tpmi = 256:259
        W48_cb3(idx(tpmi),:) = [Wbar2_cb3(floor((tpmi-256)/2)) zeros(2,2)
                                zeros(2,2)                     Wbar2_cb3(mod(tpmi,2))
                                zeros(2,2)                     zeros(2,2)
                                zeros(2,2)                     zeros(2,2)].';
    end
    for tpmi = 260:263
        W48_cb3(idx(tpmi),:) = [Wbar2_cb3(floor((tpmi-260)/2)) zeros(2,2)
                                zeros(2,2)                     zeros(2,2)
                                zeros(2,2)                     Wbar2_cb3(mod(tpmi,2))
                                zeros(2,2)                     zeros(2,2)].';
    end
    for tpmi = 264:267
        W48_cb3(idx(tpmi),:) = [Wbar2_cb3(floor((tpmi-264)/2)) zeros(2,2)
                                zeros(2,2)                     zeros(2,2)
                                zeros(2,2)                     zeros(2,2)
                                zeros(2,2)                     Wbar2_cb3(mod(tpmi,2))].';
    end
    for tpmi = 268:271
        W48_cb3(idx(tpmi),:) = [zeros(2,2)                     zeros(2,2)
                                Wbar2_cb3(floor((tpmi-268)/2)) zeros(2,2)
                                zeros(2,2)                     Wbar2_cb3(mod(tpmi,2))
                                zeros(2,2)                     zeros(2,2)].';
    end
    for tpmi = 272:275
        W48_cb3(idx(tpmi),:) = [zeros(2,2)                     zeros(2,2)
                                Wbar2_cb3(floor((tpmi-272)/2)) zeros(2,2)
                                zeros(2,2)                     zeros(2,2)
                                zeros(2,2)                     Wbar2_cb3(mod(tpmi,2))].';
    end
    for tpmi = 276:279
        W48_cb3(idx(tpmi),:) = [zeros(2,2)                     zeros(2,2)
                                zeros(2,2)                     zeros(2,2)
                                Wbar2_cb3(floor((tpmi-276)/2)) zeros(2,2)
                                zeros(2,2)                     Wbar2_cb3(mod(tpmi,2))].';
    end
    W48_cb3 = W48_cb3 / 2;
end

function W58_cb3 = initializeW58cb3()

    % Table 6.3.1.5-43

    nlayers = 5;
    numTPMI = 160;
    W58_cb3 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:15
        W58_cb3(idx(tpmi),:) = [Wbar2_cb3(floor(tpmi/8)) zeros(2,2)                      zeros(2,1)
                                zeros(2,2)               zeros(2,2)                      zeros(2,1)
                                zeros(2,2)               Wbar2_cb3(floor(mod(tpmi,8)/4)) zeros(2,1)
                                zeros(2,2)               zeros(2,2)                      Wbar1_cb3(mod(tpmi,4))].';
    end
    for tpmi = 16:31
        W58_cb3(idx(tpmi),:) = [zeros(2,2)                    zeros(2,2)                      zeros(2,1)
                                Wbar2_cb3(floor((tpmi-16)/8)) zeros(2,2)                      zeros(2,1)
                                zeros(2,2)                    Wbar2_cb3(floor(mod(tpmi,8)/4)) zeros(2,1)
                                zeros(2,2)                    zeros(2,2)                      Wbar1_cb3(mod(tpmi,4))].';
    end
    for tpmi = 32:159
        W58_cb3(idx(tpmi),:) = [Wbar1_cb3(floor((tpmi-32)/32)) zeros(2,1)                       zeros(2,2)                        zeros(2,1)
                                zeros(2,1)                     Wbar1_cb3(floor(mod(tpmi,32)/8)) zeros(2,2)                        zeros(2,1)
                                zeros(2,1)                     zeros(2,1)                       Wbar2_cb3(floor((mod(tpmi,8))/4)) zeros(2,1)
                                zeros(2,1)                     zeros(2,1)                       zeros(2,2)                        Wbar1_cb3(mod(tpmi,4))].';
    end
    W58_cb3 = W58_cb3 / 2;
end

function W68_cb3 = initializeW68cb3()

    % Table 6.3.1.5-44

    nlayers = 6;
    numTPMI = 80;
    W68_cb3 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:7
        W68_cb3(idx(tpmi),:) = [Wbar2_cb3(floor(tpmi/4)) zeros(2,2)                      zeros(2,2)
                                zeros(2,2)               Wbar2_cb3(floor(mod(tpmi,4)/2)) zeros(2,2)
                                zeros(2,2)               zeros(2,2)                      Wbar2_cb3(mod(tpmi,2))
                                zeros(2,2)               zeros(2,2)                      zeros(2,2)].';
    end
    for tpmi = 8:15
        W68_cb3(idx(tpmi),:) = [Wbar2_cb3(floor((tpmi-8)/4)) zeros(2,2)                      zeros(2,2)
                                zeros(2,2)                   zeros(2,2)                      zeros(2,2)
                                zeros(2,2)                   Wbar2_cb3(floor(mod(tpmi,4)/2)) zeros(2,2)
                                zeros(2,2)                   zeros(2,2)                      Wbar2_cb3(mod(tpmi,2))].';
    end
    for tpmi = 16:79
        W68_cb3(idx(tpmi),:) = [Wbar2_cb3(floor((tpmi-16)/32)) zeros(2,1)                          zeros(2,2)                      zeros(2,1)
                                zeros(2,2)                     Wbar1_cb3(floor(mod(tpmi-16,32)/8)) zeros(2,2)                      zeros(2,1)
                                zeros(2,2)                     zeros(2,1)                          Wbar2_cb3(floor(mod(tpmi,8)/4)) zeros(2,1)
                                zeros(2,2)                     zeros(2,1)                          zeros(2,2)                      Wbar1_cb3(mod(tpmi,4))].';
    end
    W68_cb3 = W68_cb3 / 2;
end

function W78_cb3 = initializeW78cb3()

    % Table 6.3.1.5-45

    nlayers = 7;
    numTPMI = 32;
    W78_cb3 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:31
        W78_cb3(idx(tpmi),:) = [Wbar2_cb3(floor(tpmi/16)) zeros(2,1)                       zeros(2,2)                      zeros(2,2)
                                zeros(2,2)                Wbar1_cb3(floor(mod(tpmi,16)/4)) zeros(2,2)                      zeros(2,2)
                                zeros(2,2)                zeros(2,1)                       Wbar2_cb3(floor(mod(tpmi,4)/2)) zeros(2,2)
                                zeros(2,2)                zeros(2,1)                       zeros(2,2)                      Wbar2_cb3(mod(tpmi,2))].';
    end
    W78_cb3 = W78_cb3 / 2;
end

function W88_cb3 = initializeW88cb3()

    % Table 6.3.1.5-46

    nlayers = 8;
    numTPMI = 16;
    W88_cb3 = coder.nullcopy(zeros(numTPMI*nlayers,8,'like',1i)); % Matrix dimension is (numTPMI*nlayers,nports)
    idx = @(tpmi)getMatrixIndex(nlayers,tpmi);
    for tpmi = 0:15
        W88_cb3(idx(tpmi),:) = [Wbar2_cb3(floor(tpmi/8)) zeros(2,2)                      zeros(2,2)                      zeros(2,2)
                                zeros(2,2)               Wbar2_cb3(floor(mod(tpmi,8)/4)) zeros(2,2)                      zeros(2,2)
                                zeros(2,2)               zeros(2,2)                      Wbar2_cb3(floor(mod(tpmi,4)/2)) zeros(2,2)
                                zeros(2,2)               zeros(2,2)                      zeros(2,2)                      Wbar2_cb3(mod(tpmi,2))].';
    end
    W88_cb3 = W88_cb3 / 2;
end

function idx = getMatrixIndex(nrows,tpmi)
    % Get the indices of the matrix, given the number of rows of the matrix
    % for each TPMI and the TPMI index. NROWS is the number of layers for
    % TS 38.211 Tables 6.3.1.5-39 to 6.3.1.5-46. NROWS is the number of
    % columns for Tables 6.3.1.5-37 to 6.3.1.5-38.
    idx = tpmi*nrows+(1:nrows);
end