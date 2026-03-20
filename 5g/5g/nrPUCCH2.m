function sym = nrPUCCH2(uciCW,nid,rnti,varargin)
%nrPUCCH2 Physical uplink control channel format 2
%   SYM = nrPUCCH2(UCICW,NID,RNTI) returns a complex column vector SYM
%   containing physical uplink control channel format 2 encoded symbols as
%   per TS 38.211 Section 6.3.2.5, by considering the following inputs:
%   UCICW - Encoded UCI codeword as per TS 38.212 Section 6.3.1. It must be
%           a column vector.
%   NID   - Scrambling identity. It is equal to the higher-layer parameter
%           dataScramblingIdentityPUSCH (0...1023), if configured, else, it
%           is equal to the physical layer cell identity, NCellID
%           (0...1007).
%   RNTI  - Radio Network Temporary Identifier (0...65535).
%
%   The encoding process involves scrambling followed by QPSK modulation.
%
%   SYM = nrPUCCH2(UCICW,NID,RNTI,NAME,VALUE) specifies an additional
%   option as a NAME,VALUE pair to allow control over the data type of the
%   output symbols:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
%
%   Example 1:
%   % Generate PUCCH format 2 symbols with nid as 148 and rnti as 160.
%
%   uciCW = randi([0 1],100,1);
%   nid = 148;
%   rnti = 160;
%   sym = nrPUCCH2(uciCW,nid,rnti);
%
%   Example 2:
%   % Generate PUCCH format 2 symbols of single data type with nid as 512
%   % and rnti as 2563.
%
%   uciCW = randi([0 1],100,1);
%   nid = 512;
%   rnti = 2563;
%   sym = nrPUCCH2(uciCW,nid,rnti,'OutputDataType','single');
%
%   See also nrPUCCH0, nrPUCCH1, nrPUCCH3, nrPUCCH4, nrPUCCHPRBS,
%   nrSymbolModulate.

%   Copyright 2018-2023 The MathWorks, Inc.

%#codegen

    narginchk(3,5);

    validateInputs(uciCW,nid,rnti);

    % Set interlacing specific parameters to empty as they are not
    % supported in this function.
    nIRB = [];
    sf = [];
    occi = [];
    sym = nr5g.internal.pucch.hPUCCH2(uciCW,nid,rnti,nIRB,sf,occi,varargin{:});

end

function validateInputs(uciCW,nid,rnti)

    % Validate mandatory inputs
    fcnName = 'nrPUCCH2';
    validateattributes(uciCW,{'double','int8','logical'},{'real'},...
        fcnName,'UCICW');
    coder.internal.errorIf(~(iscolumn(uciCW) || isempty(uciCW)),...
        'nr5g:nrPUCCH:InvalidInputDim');
    validateattributes(nid,{'numeric'},{'scalar',...
        'real','integer','>=',0,'<=',1023},fcnName,'NID');
    validateattributes(rnti,{'numeric'},{'scalar',...
        'real','integer','>=',0,'<=',65535},fcnName,'RNTI');

end