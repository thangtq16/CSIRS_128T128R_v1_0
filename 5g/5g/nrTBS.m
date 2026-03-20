function tbs = nrTBS(modulation,nlayers,nPRB,NREPerPRB,tcr,varargin)
%nrTBS Transport block size determination
%   TBS = nrTBS(MODULATION,NLAYERS,NPRB,NREPERPRB,TCR) returns the
%   transport block size, TBS, associated with each codeword for a shared
%   channel transmission, as defined in TS 38.214 Sections 5.1.3.2 and
%   6.1.4.2, with the inputs:
%   MODULATION - Modulation scheme for each codeword. It must be specified
%                as one of {'pi/2-BPSK', 'QPSK', '16QAM', '64QAM', 
%                '256QAM','1024QAM'}. The modulation scheme can be 
%                specified as a character array or string scalar for one
%                codeword or two codewords. In case of two codewords, the
%                same modulation scheme is applied to both codewords. 
%                Alternatively, a cell array of character vectors or string
%                array can be used to specify different modulation schemes
%                for each codeword.
%   NLAYERS    - Number of transmission layers (1...4 for one codeword,
%                5...8 for two codewords).
%   NPRB       - Number of physical resource blocks (PRBs) allocated for
%                the physical shared channel. The value must be a scalar
%                nonnegative integer. The nominal value of NPRB is in the
%                range of 0...275.
%   NREPERPRB  - Number of resource elements (REs) allocated for the data
%                transmission in the physical shared channel, within one
%                PRB for one slot. The value must be a scalar nonnegative
%                integer. Note that this value does not include any
%                additional overhead.
%   TCR        - Target code rate for each codeword. It is a scalar for one
%                codeword or a two-element vector for two codewords, with
%                each value between 0 and 1. Alternatively, two codewords
%                can also be configured with single target code rate.
%   Note that the additional overhead and the scaling factor, used for TBS
%   calculation are considered as 0 and 1, respectively.
%
%   TBS = nrTBS(MODULATION,NLAYERS,NPRB,NREPERPRB,TCR,XOH) specifies the
%   additional overhead, which controls the number of REs available for the
%   data transmission in the shared channel, within one PRB for one slot.
%   XOH must be a scalar nonnegative integer. The nominal value of XOH is
%   one of {0, 6, 12, 18}, provided by the higher-layer parameter xOverhead
%   in PDSCH-ServingCellConfig IE or PUSCH-ServingCellConfig IE. Note that
%   the scaling factor is considered as 1.
%
%   TBS = nrTBS(MODULATION,NLAYERS,NPRB,NREPERPRB,TCR,XOH,TBSCALING) also
%   specifies the scaling factor(s) used in the calculation of intermediate
%   number of information bits, N_info, as defined in TS 38.214
%   Section 5.1.3.2. TBSCALING is a scalar for one codeword or a
%   two-element vector for two codewords, with each value must be greater
%   than 0 and less than or equal to 1. Alternatively, two codewords can
%   also be configured with single scaling factor. The nominal value of
%   TBSCALING is one of {0.25, 0.5, 1}, as defined in TS 38.214
%   Table 5.1.3.2-2.
%
%   Example 1:
%   % Determine the transport block size associated with a data
%   % transmission in a shared channel for one codeword. Specify the
%   % modulation scheme as '16QAM', number of transmission layers as 4,
%   % number of resource blocks allocated for the shared channel as 52,
%   % number of REs allocated for the shared channel within one PRB for one
%   % slot without accounting for the additional overhead as 120. Set the
%   % target code rate as 0.48, additional overhead as 6, and scaling
%   % factor as 0.25.
%
%   modulation = '16QAM';
%   nlayers = 4;
%   nPRB = 52;
%   NREPerPRB = 120;
%   tcr = 0.48;
%   xOh = 6;
%   tbScaling = 0.25;
%
%   % Calculate TBS
%   tbs = nrTBS(modulation,nlayers,nPRB,NREPerPRB,tcr,xOh,tbScaling)
%
%   Example 2:
%   % Determine the payload size of each transport block for a shared
%   % channel transmission with two codewords. Specify the modulation
%   % schemes as 'QPSK' and '64QAM', number of transmission layers as 8,
%   % number of resource blocks allocated for the shared channel as 106,
%   % number of REs allocated for shared channel within one PRB for one
%   % slot having no additional overhead as 100. Set the target code rates
%   % for two codewords as 0.3701 and 0.4277.
%
%   modulation = {'QPSK','64QAM'};
%   nlayers = 8;
%   nPRB = 106;
%   NREPerPRB = 100;
%   tcr = [0.3701 0.4277];
%
%   % Calculate TBS
%   tbs = nrTBS(modulation,nlayers,nPRB,NREPerPRB,tcr)
%
%   See also nrDLSCH, nrULSCH.

%   Copyright 2020-2024 The MathWorks, Inc.

%#codegen

    % Pre-process the input arguments
    narginchk(5,7);
    Qm = nr5g.internal.TBSDetermination.physicalChParamsLinear(modulation,nlayers,nPRB,NREPerPRB);
    [tcr,xOh,tbScaling] = nr5g.internal.TBSDetermination.transportChParamsLinear(tcr, 1, varargin{:}); % Include TB scaling look-up
    
    % Supply all inputs to the full linear function
    tbs = nr5g.internal.TBSDetermination.calculateTBS(nlayers,nPRB,NREPerPRB,Qm,tcr,xOh,tbScaling);

    return;
