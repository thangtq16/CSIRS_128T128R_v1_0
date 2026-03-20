function W = nrPUSCHCodebook(nlayers,nports,TPMI,varargin)
%nrPUSCHCodebook Codebook for PUSCH precoding
%   W = nrPUSCHCodebook(NLAYERS,NPORTS,TPMI) returns the PUSCH precoding
%   matrix W defined for codebook-based transmission in TS 38.211 Section
%   6.3.1.5 for number of layers NLAYERS (1...8), number of antenna ports
%   NPORTS (1,2,4,8) and transmitted precoding matrix indicator TPMI
%   (0...304, valid range depends on NLAYERS, NPORTS, and CODEBOOKTYPE).
%   Transform precoding is disabled, but can be enabled by an optional
%   input described below.
%   
%   W is the NLAYERS-by-NPORTS precoding matrix for codebook-based
%   transmission of the PUSCH. W=1 for NLAYERS=1 and NPORTS=1 otherwise it
%   is selected from TS 38.211 Tables 6.3.1.5-1...24, depending on NLAYERS,
%   NPORTS and TPMI.
%
%   Note that W is the transpose of the precoding matrix defined in the
%   specification. nrLayerMap produces an output of size M-by-NLAYERS, i.e.
%   one column per layer, and the orientation of W here allows the
%   precoding to be performed by matrix multiplication.
%
%   W = nrPUSCHCodebook(NLAYERS,NPORTS,TPMI,TPRECODE) enables or disables
%   transform precoding through the input TPRECODE (false, true), which
%   affects the precoding matrix used in the case of NLAYERS=1 and NPORTS=4
%   (see TS 38.211 Section 6.3.1.4).
%
%   W = nrPUSCHCodebook(...,CODEBOOKTYPE) specifies the codebook type
%   ('codebook1_ng1n4n1', 'codebook1_ng1n2n2', 'codebook2', 'codebook3',
%   'codebook4') in the codebook transmission when NPORTS = 8. Together
%   with NLAYERS and TPMI, CODEBOOKTYPE is used in codebook transmissions
%   with 8 antenna ports to choose the precoding matrix W from TS 38.211
%   Tables 6.3.1.5-9 to 6.3.1.5-47. Use the table below to identify which
%   codebook type to use for a specific number of antenna groups (Ng) and
%   specific table from TS 38.211. For single antenna group (Ng=1), the
%   table also shows the geometrical distribution of the antenna ports in
%   each antenna group [N1 N2]. N1 is the number of antenna ports in the
%   horizontal direction and N2 is the number of antenna ports in the
%   vertical direction.
%
%   CODEBOOKTYPE        |   Ng  |  [N1 N2]  |        Tables
%   --------------------|-------|-----------|--------------------------
%   'codebook1_ng1n4n1' |   1   |   [4 1]   |  6.3.1.5-9  to 6.3.1.5-16
%   'codebook1_ng1n2n2' |   1   |   [2 2]   |  6.3.1.5-17 to 6.3.1.5-24
%   'codebook2'         |   2   |     -     |  6.3.1.5-25 to 6.3.1.5-36
%   'codebook3'         |   4   |     -     |  6.3.1.5-37 to 6.3.1.5-46
%   'codebook4'         |   8   |     -     |  6.3.1.5-47
%    
%   Example:
%   % Create 64QAM modulation symbols, split onto two layers, and precode
%   % using a precoding matrix for four antenna ports.
%
%   modulation = '64QAM';
%   nlayers = 2;
%   nports = 4;
%   TPMI = 7;
%   
%   data = randi([0 1],600,1);
%   d = nrSymbolModulate(data,modulation);
%   y = nrLayerMap(d,nlayers);
%
%   W = nrPUSCHCodebook(nlayers,nports,TPMI);
%   z = y * W;
%
%   See also nrLayerMap, nrLayerDemap.
 
%   Copyright 2018-2025 The MathWorks, Inc.

%#codegen

    % Initialize precoding matrix tables
    persistent matrixTables validTPMIsCodebook4;
    if (isempty(matrixTables))
        [matrixTables, validTPMIsCodebook4] = initializeMatrixTables();
    end

    narginchk(3,5);

    % Parse and validate inputs
    transformPrecode = false;
    codebookTypeIn = 'codebook1_ng1n4n1';
    if (nargin>3)
        transformPrecode = varargin{1};
        if (nargin>4)
            codebookTypeIn = varargin{2};
        end
    end
    fcnName = 'nrPUSCHCodebook';
    validateattributes(nlayers,{'numeric'}, ...
        {'scalar','integer','>=',1,'<=',8},fcnName,'NLAYERS');
    validateattributes(nports,{'numeric'}, ...
        {'scalar','integer'},fcnName,'NPORTS');
    coder.internal.errorIf(~any(nports==[1 2 4 8]), ...
        'nr5g:nrPUSCHCodebook:InvalidNPorts',nports);
    validateattributes(TPMI,{'numeric'}, ...
        {'scalar','integer','>=',0,'<=',304},fcnName,'TPMI');
    validateattributes(transformPrecode,{'numeric','logical'}, ...
        {'scalar'},fcnName,'TPRECODE');
    codebookType = validatestring(codebookTypeIn,...
        {'codebook1_ng1n4n1', 'codebook1_ng1n2n2', 'codebook2', 'codebook3', 'codebook4'},fcnName,'CODEBOOKTYPE');
    coder.internal.errorIf(nlayers>nports, ...
        'nr5g:nrPUSCHCodebook:TooManyLayers',nlayers,nports);

    % Port mapping function for 8 port transmission - Table 6.3.1.5-8.
    %     cb1  cb2  cb3  cb4
    f_i = [0    0    0    0
           1    1    4    1
           2    4    1    2
           3    5    5    3
           4    2    2    4
           5    3    6    5
           6    6    3    6
           7    7    7    7];

    % Initialize for codegen
    allW = coder.nullcopy(nan);
    allWCodebook4 = coder.nullcopy(cell(1));
    W = coder.nullcopy(complex(nan)); %#ok<NASGU>
    coder.varsize("W",[inf 8]);

    if (nports==1) % single antenna port

        % Section 6.3.1.5 - Only single layer is allowed
        allW = 1;

    elseif (nports==2) % two antenna ports

        if (nlayers==1) % single-layer transmission

            % Table 6.3.1.5-1
            allW = matrixTables.W12;

        else % two-layer transmission

            % Table 6.3.1.5-4
            allW = matrixTables.W22;

        end

    elseif (nports==4) % four antenna ports

        if (nlayers==1) % single-layer transmission

            if (transformPrecode) % transform precoding enabled

                % Table 6.3.1.5-2
                allW = matrixTables.W14_tp;

            else % transform precoding disabled

                % Table 6.3.1.5-3
                allW = matrixTables.W14_notp;

            end

        elseif (nlayers==2) % two-layer transmission

            % Table 6.3.1.5-5
            allW = matrixTables.W24;

        elseif (nlayers==3) % three-layer transmission

            % Table 6.3.1.5-6
            allW = matrixTables.W34;

        else % four-layer transmission

            % Table 6.3.1.5-7
            allW = matrixTables.W44;

        end

    else % eight antenna ports

        if codebookType=="codebook1_ng1n4n1"

            if (nlayers==1)

                % Table 6.3.1.5-9
                allW = matrixTables.W18cb1_41;

            elseif (nlayers==2)

                % Table 6.3.1.5-10
                allW = matrixTables.W28cb1_41;

            elseif (nlayers==3)

                % Table 6.3.1.5-11
                allW = matrixTables.W38cb1_41;

            elseif (nlayers==4)

                % Table 6.3.1.5-12
                allW = matrixTables.W48cb1_41;

            elseif (nlayers==5)

                % Table 6.3.1.5-13
                allW = matrixTables.W58cb1_41;

            elseif (nlayers==6)

                % Table 6.3.1.5-14
                allW = matrixTables.W68cb1_41;

            elseif (nlayers==7)

                % Table 6.3.1.5-15
                allW = matrixTables.W78cb1_41;

            else % nlayers==8

                % Table 6.3.1.5-16
                allW = matrixTables.W88cb1_41;

            end

            % Update the codebook matrix using the port mapping function f(i)
            % Note that f(i) represents a 1-1 map for codebook1
            allW(:,f_i(:,1)+1) = allW;

        elseif codebookType=="codebook1_ng1n2n2"

            if (nlayers==1)

                % Table 6.3.1.5-17
                allW = matrixTables.W18cb1_22;

            elseif (nlayers==2)

                % Table 6.3.1.5-18
                allW = matrixTables.W28cb1_22;

            elseif (nlayers==3)

                % Table 6.3.1.5-19
                allW = matrixTables.W38cb1_22;

            elseif (nlayers==4)

                % Table 6.3.1.5-20
                allW = matrixTables.W48cb1_22;

            elseif (nlayers==5)

                % Table 6.3.1.5-21
                allW = matrixTables.W58cb1_22;

            elseif (nlayers==6)

                % Table 6.3.1.5-22
                allW = matrixTables.W68cb1_22;

            elseif (nlayers==7)

                % Table 6.3.1.5-23
                allW = matrixTables.W78cb1_22;

            else % nlayers==8

                % Table 6.3.1.5-24
                allW = matrixTables.W88cb1_22;

            end

            % Update the codebook matrix using the port mapping function f(i)
            % Note that f(i) represents a 1-1 map for codebook1
            allW(:,f_i(:,1)+1) = allW;

        elseif codebookType=="codebook2"

            if (nlayers==1)

                % Table 6.3.1.5-29
                allW = matrixTables.W18cb2;

            elseif (nlayers==2)

                % Table 6.3.1.5-30
                allW = matrixTables.W28cb2;

            elseif (nlayers==3)

                % Table 6.3.1.5-31
                allW = matrixTables.W38cb2;

            elseif (nlayers==4)

                % Table 6.3.1.5-32
                allW = matrixTables.W48cb2;

            elseif (nlayers==5)

                % Table 6.3.1.5-33
                allW = matrixTables.W58cb2;

            elseif (nlayers==6)

                % Table 6.3.1.5-34
                allW = matrixTables.W68cb2;

            elseif (nlayers==7)

                % Table 6.3.1.5-35
                allW = matrixTables.W78cb2;

            else % nlayers==8

                % Table 6.3.1.5-36
                allW = matrixTables.W88cb2;

            end

            % Update the codebook matrix using the port mapping function f(i)
            allW(:,f_i(:,2)+1) = allW;

        elseif codebookType=="codebook3"

            if (nlayers==1)

                % Table 6.3.1.5-39
                allW = matrixTables.W18cb3;

            elseif (nlayers==2)

                % Table 6.3.1.5-40
                allW = matrixTables.W28cb3;

            elseif (nlayers==3)

                % Table 6.3.1.5-41
                allW = matrixTables.W38cb3;

            elseif (nlayers==4)

                % Table 6.3.1.5-42
                allW = matrixTables.W48cb3;

            elseif (nlayers==5)

                % Table 6.3.1.5-43
                allW = matrixTables.W58cb3;

            elseif (nlayers==6)

                % Table 6.3.1.5-44
                allW = matrixTables.W68cb3;

            elseif (nlayers==7)

                % Table 6.3.1.5-45
                allW = matrixTables.W78cb3;

            else % nlayers==8

                % Table 6.3.1.5-46
                allW = matrixTables.W88cb3;

            end

            % Update the codebook matrix using the port mapping function f(i)
            allW(:,f_i(:,3)+1) = allW;

        else % codebook4

            % The matrix computation in Table 6.3.1.5-47 is different and
            % there are not separate tables for different numbers of layers
            allWCodebook4 = matrixTables.W8codebook4;

        end

    end
    
    if (nports==8 && codebookType=="codebook4")
        % Codebook4 defines the valid values for TPMI differently
        validTPMIsCodebook4Txt = getValidTPMITxt(validTPMIsCodebook4{nlayers});
        coder.internal.errorIf(~any(TPMI==validTPMIsCodebook4{nlayers}), ...
            'nr5g:nrPUSCHCodebook:InvalidTPMICodebook4',nlayers,nports,TPMI,validTPMIsCodebook4Txt);
        W = allWCodebook4{TPMI+1}';

        % Update the codebook matrix using the port mapping function f(i)
        % Note that f(i) represents a 1-1 map for codebook4
        W(:,f_i(:,4)+1) = W;
    else
        maxTPMI = (size(allW,1) / nlayers) - 1;
        coder.internal.errorIf(TPMI>maxTPMI, ...
            'nr5g:nrPUSCHCodebook:InvalidTPMI',nlayers,nports,TPMI,maxTPMI);
        W = allW(double(TPMI)*nlayers + (1:nlayers),:);
    end
    
end

function [matrixTables, validTPMIsCodebook4] = initializeMatrixTables()

    % 1-4 ports
    matrixTables.W12 = initializeW12();
    matrixTables.W14_tp = initializeW14_tp();
    matrixTables.W14_notp = initializeW14_notp();
    matrixTables.W22 = initializeW22();
    matrixTables.W24 = initializeW24();
    matrixTables.W34 = initializeW34();
    matrixTables.W44 = initializeW44();

    % 8 ports, codebook1_ng1n4n1
    matrixTables.W18cb1_41 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(1,'ng1n4n1');
    matrixTables.W28cb1_41 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(2,'ng1n4n1');
    matrixTables.W38cb1_41 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(3,'ng1n4n1');
    matrixTables.W48cb1_41 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(4,'ng1n4n1');
    matrixTables.W58cb1_41 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(5,'ng1n4n1');
    matrixTables.W68cb1_41 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(6,'ng1n4n1');
    matrixTables.W78cb1_41 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(7,'ng1n4n1');
    matrixTables.W88cb1_41 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(8,'ng1n4n1');

    % 8 ports, codebook1_ng1n2n2
    matrixTables.W18cb1_22 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(1,'ng1n2n2');
    matrixTables.W28cb1_22 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(2,'ng1n2n2');
    matrixTables.W38cb1_22 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(3,'ng1n2n2');
    matrixTables.W48cb1_22 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(4,'ng1n2n2');
    matrixTables.W58cb1_22 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(5,'ng1n2n2');
    matrixTables.W68cb1_22 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(6,'ng1n2n2');
    matrixTables.W78cb1_22 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(7,'ng1n2n2');
    matrixTables.W88cb1_22 = nr5g.internal.pusch.getPrecodingMatrixCodebook1(8,'ng1n2n2');

    % 8 ports, codebook2
    matrixTables.W18cb2 = nr5g.internal.pusch.getPrecodingMatrixCodebook2(1);
    matrixTables.W28cb2 = nr5g.internal.pusch.getPrecodingMatrixCodebook2(2);
    matrixTables.W38cb2 = nr5g.internal.pusch.getPrecodingMatrixCodebook2(3);
    matrixTables.W48cb2 = nr5g.internal.pusch.getPrecodingMatrixCodebook2(4);
    matrixTables.W58cb2 = nr5g.internal.pusch.getPrecodingMatrixCodebook2(5);
    matrixTables.W68cb2 = nr5g.internal.pusch.getPrecodingMatrixCodebook2(6);
    matrixTables.W78cb2 = nr5g.internal.pusch.getPrecodingMatrixCodebook2(7);
    matrixTables.W88cb2 = nr5g.internal.pusch.getPrecodingMatrixCodebook2(8);

    % 8 ports, codebook3
    matrixTables.W18cb3 = nr5g.internal.pusch.getPrecodingMatrixCodebook3(1);
    matrixTables.W28cb3 = nr5g.internal.pusch.getPrecodingMatrixCodebook3(2);
    matrixTables.W38cb3 = nr5g.internal.pusch.getPrecodingMatrixCodebook3(3);
    matrixTables.W48cb3 = nr5g.internal.pusch.getPrecodingMatrixCodebook3(4);
    matrixTables.W58cb3 = nr5g.internal.pusch.getPrecodingMatrixCodebook3(5);
    matrixTables.W68cb3 = nr5g.internal.pusch.getPrecodingMatrixCodebook3(6);
    matrixTables.W78cb3 = nr5g.internal.pusch.getPrecodingMatrixCodebook3(7);
    matrixTables.W88cb3 = nr5g.internal.pusch.getPrecodingMatrixCodebook3(8);

    % 8 ports, codebook4
    [matrixTables.W8codebook4, validTPMIsCodebook4] = nr5g.internal.pusch.getPrecodingMatrixCodebook4();

end

function W12 = initializeW12()

    % Table 6.3.1.5-1
    %              TPMI
    W12 = [1   0;  %  0
           0   1;  %  1
           1   1;  %  2
           1  -1;  %  3
           1  1j;  %  4
           1 -1j]; %  5
    W12 = W12 / sqrt(2);
    
end

function W14_tp = initializeW14_tp()

    % Table 6.3.1.5-2
    %                         TPMI
    W14_tp = [1   0   0   0;  %  0
              0   1   0   0;  %  1
              0   0   1   0;  %  2
              0   0   0   1;  %  3
              1   0   1   0;  %  4
              1   0  -1   0;  %  5
              1   0  1j   0;  %  6
              1   0 -1j   0;  %  7
              0   1   0   1;  %  8
              0   1   0  -1;  %  9
              0   1   0  1j;  % 10
              0   1   0 -1j;  % 11
              1   1   1  -1;  % 12
              1   1  1j  1j;  % 13
              1   1  -1   1;  % 14
              1   1 -1j -1j;  % 15
              1  1j   1  1j;  % 16
              1  1j  1j   1;  % 17
              1  1j  -1 -1j;  % 18
              1  1j -1j  -1;  % 19
              1  -1   1   1;  % 20
              1  -1  1j -1j;  % 21
              1  -1  -1  -1;  % 22
              1  -1 -1j  1j;  % 23
              1 -1j   1 -1j;  % 24
              1 -1j  1j  -1;  % 25
              1 -1j  -1  1j;  % 26
              1 -1j -1j   1]; % 27
    W14_tp = W14_tp / 2;
    
end

function W14_notp = initializeW14_notp()
    
    % Table 6.3.1.5-3
    %                           TPMI
    W14_notp = [1   0   0   0;  %  0
                0   1   0   0;  %  1
                0   0   1   0;  %  2
                0   0   0   1;  %  3
                1   0   1   0;  %  4
                1   0  -1   0;  %  5
                1   0  1j   0;  %  6
                1   0 -1j   0;  %  7
                0   1   0   1;  %  8
                0   1   0  -1;  %  9
                0   1   0  1j;  % 10
                0   1   0 -1j;  % 11
                1   1   1   1;  % 12
                1   1  1j  1j;  % 13
                1   1  -1  -1;  % 14
                1   1 -1j -1j;  % 15
                1  1j   1  1j;  % 16
                1  1j  1j  -1;  % 17
                1  1j  -1 -1j;  % 18
                1  1j -1j   1;  % 19
                1  -1   1  -1;  % 20
                1  -1  1j -1j;  % 21
                1  -1  -1   1;  % 22
                1  -1 -1j  1j;  % 23
                1 -1j   1 -1j;  % 24
                1 -1j  1j   1;  % 25
                1 -1j  -1  1j;  % 26
                1 -1j -1j  -1]; % 27
    W14_notp = W14_notp / 2;
    
end

function W22 = initializeW22()

    % Table 6.3.1.5-4
    %              TPMI
    W22 = [1   0;  %  0
           0   1;
           1   1;  %  1
           1  -1;
           1  1j;  %  2
           1 -1j];
    W22(1:2,:) = W22(1:2,:) / sqrt(2);
    W22(3:end,:) = W22(3:end,:) / 2;

end

function W24 = initializeW24()

    % Table 6.3.1.5-5
    %                      TPMI
    W24 = [1   0   0   0;  %  0
           0   1   0   0;
           1   0   0   0;  %  1
           0   0   1   0;
           1   0   0   0;  %  2
           0   0   0   1;
           0   1   0   0;  %  3
           0   0   1   0;
           0   1   0   0;  %  4
           0   0   0   1;
           0   0   1   0;  %  5
           0   0   0   1;
           1   0   1   0;  %  6
           0   1   0 -1j;
           1   0   1   0;  %  7
           0   1   0  1j;
           1   0 -1j   0;  %  8
           0   1   0   1;
           1   0 -1j   0;  %  9
           0   1   0  -1;
           1   0  -1   0;  % 10
           0   1   0 -1j;
           1   0  -1   0;  % 11
           0   1   0  1j;
           1   0  1j   0;  % 12
           0   1   0   1;
           1   0  1j   0;  % 13
           0   1   0  -1;
           1   1   1   1;  % 14
           1   1  -1  -1;
           1   1  1j  1j;  % 15
           1   1 -1j -1j;
           1  1j   1  1j;  % 16
           1  1j  -1 -1j;
           1  1j  1j  -1;  % 17
           1  1j -1j   1;
           1  -1   1  -1;  % 18
           1  -1  -1   1;
           1  -1  1j -1j;  % 19
           1  -1 -1j  1j;
           1 -1j   1 -1j;  % 20
           1 -1j  -1  1j;
           1 -1j  1j   1;  % 21
           1 -1j -1j  -1];
    W24(1:28,:) = W24(1:28,:) / 2;
    W24(29:end,:) = W24(29:end,:) / (2*sqrt(2));
    
end

function W34 = initializeW34()

    % Table 6.3.1.5-6
    %                     TPMI
    W34 = [1   0   0   0; %  0
           0   1   0   0;
           0   0   1   0;
           1   0   1   0; %  1
           0   1   0   0;
           0   0   0   1;
           1   0  -1   0; %  2
           0   1   0   0;
           0   0   0   1;
           1   1   1   1; %  3
           1  -1   1  -1;
           1   1  -1  -1;
           1   1  1j  1j; %  4
           1  -1  1j -1j;
           1   1 -1j -1j;
           1  -1   1  -1; %  5
           1   1   1   1;
           1  -1  -1   1;
           1  -1  1j -1j; %  6
           1   1  1j  1j;
           1  -1 -1j  1j];
    W34(1:9,:) = W34(1:9,:) / 2;
    W34(10:end,:) = W34(10:end,:) / (2*sqrt(3));
    
end

function W44 = initializeW44()

    % Table 6.3.1.5-7
    %                     TPMI
    W44 = [1   0   0   0; %  0
           0   1   0   0;
           0   0   1   0;
           0   0   0   1;
           1   0   1   0; %  1
           1   0  -1   0;
           0   1   0   1;
           0   1   0  -1;
           1   0  1j   0; %  2
           1   0 -1j   0;
           0   1   0  1j;
           0   1   0 -1j;
           1   1   1   1; %  3
           1  -1   1  -1;
           1   1  -1  -1;
           1  -1  -1   1;
           1   1  1j  1j; %  4
           1  -1  1j -1j;
           1   1 -1j -1j;
           1  -1 -1j  1j];
    W44(1:4,:) = W44(1:4,:) / 2;
    W44(5:12,:) = W44(5:12,:) / (2*sqrt(2));
    W44(13:end,:) = W44(13:end,:) / 4;
    
end


%% Additional local functions

% Create the string that contains the valid range of TPMI values for 8
% ports and codebook4
function validTPMIsCodebook4Txt = getValidTPMITxt(validTPMIs)

    if validTPMIs(end)>254
        % 1, 2, 3, and 4 layers have a disjoint set of valid TPMI values
        validTPMIsCodebook4Txt = sprintf('[%.0f:%.0f, %.0f]',validTPMIs(1),validTPMIs(end-1),validTPMIs(end));
    else
        validTPMIsCodebook4Txt = sprintf('[%.0f:%.0f]',validTPMIs(1),validTPMIs(end));
    end

end