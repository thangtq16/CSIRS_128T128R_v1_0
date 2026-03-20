function sym = nrPDCCH(dciCW,nID,nRNTI,varargin)
%nrPDCCH Physical downlink control channel
%   SYM = nrPDCCH(DCICW,NID,NRNTI) returns a complex column vector SYM
%   containing the physical downlink control channel (PDCCH) modulation
%   symbols as defined in TS 38.211 Section 7.3.2.
%   DCICW is the encoded DCI codeword as per TS 38.212 Section 7.3.
%   NID is the pdcch-DMRS-ScramblingID (0...65535), if configured, for a
%   UE-specific search space, else, it is the physical layer cell identity,
%   NCellID (0...1007).
%   NRNTI is the C-RNTI (1...65519) for a PDCCH in a UE specific search
%   space or 0 otherwise.
%   The two supported sets for {NID,NRNTI} are {NCellID,0} or
%   {pdcch-DMRS-ScramblingID,C-RNTI}.
%
%   SYM = nrPDCCH(...,NAME,VALUE) specifies an additional option as a
%   NAME,VALUE pair to allow control over the format of the output symbols:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
%
%   % Example 1:
%   % Generate PDCCH symbols configured with pdcch-DMRS-ScramblingID
%
%   nID = 2^11;                     % pdcch-DMRS-ScramblingID
%   nRNTI = 123;                    % C-RNTI
%   dciCW = randi([0 1],560,1);     % DCI codeword
% 
%   sym = nrPDCCH(dciCW,nID,nRNTI);
%
%   % Example 2:
%   % Generate PDCCH symbols configured with NCellID
%
%   nID = 123;                  % NCellID (0...1007)
%   nRNTI = 0;
%   dciCW = randi([0 1],560,1); % DCI codeword
% 
%   sym = nrPDCCH(dciCW,nID,nRNTI);
%
%   See also nrPDCCHDecode, nrPDCCHPRBS.

%   Copyright 2018-2020 The MathWorks, Inc.

%#codegen

%   Reference:
%   [1] 3GPP TS 38.211, "3rd Generation Partnership Project; Technical 
%   Specification Group Radio Access Network; NR; Physical Channel and 
%   Modulation (Release 15). Section 7.3.2.

    narginchk(3,5);

    % Validate mandatory inputs
    validateInputs(dciCW,nID,nRNTI);

    % Section 7.3.2.3 Scrambling
    cSeq = nrPDCCHPRBS(nID,nRNTI,length(dciCW));
    scrambled = xor(dciCW,cSeq);

    % Section 7.3.2.4 Modulation
    sym = nrSymbolModulate(scrambled,'QPSK',varargin{:});

end


function validateInputs(dciCW,nID,nRNTI)
% Check inputs

    fcnName = 'nrPDCCH';

    % Validate encoded DCI codeword
    validateattributes(dciCW,{'int8','double'},{'binary','column'}, ...
        fcnName,'DCICW');
    coder.internal.errorIf(mod(length(dciCW),2)~=0, ...
        'nr5g:nrPDCCH:InvalidDCICW');

    % Validate scrambling identity (0...65535 or 0...1007)
    validateattributes(nID, {'numeric'}, ...
        {'scalar','nonnegative','integer','<=',65535},fcnName,'NID');

    % Validate radio network temporary identifier (0...65519)
    validateattributes(nRNTI, {'numeric'}, ...
        {'scalar','nonnegative','integer','<=',65519},fcnName,'NRNTI');

end
