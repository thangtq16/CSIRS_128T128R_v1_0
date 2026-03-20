function [dciCW,crc] = nrDCIEncode(dciBits,rnti,Edci)
%nrDCIEncode Downlink control information encoding
%   [DCICW,MCRC] = nrDCIEncode(DCIBITS,RNTI,EDCI) encodes the input DCI bits,
%   DCIBITS, as per TS 38.212 Sections 7.3.2, 7.3.3 and 7.3.4 to output the
%   rate-matched coded block, DCICW, of specified length EDCI. The processing
%   includes CRC attachment, polar coding and rate matching. RNTI input 
%   specifies the Radio Network Temporary Identifier used to mask the 
%   last 16 bits of the 24 appended CRC bits. The RNTI masked CRC bits,
%   p0,p1...p23, are returned in the decimal integer, MCRC, calculated 
%   MSB first, i.e. MCRC = p0*2^23 + p1*2^22 + .... + p23*2^0.
%   The input DCIBITS must be a binary column vector corresponding to the
%   DCI bits and the output is a binary column vector of length EDCI.
%
%   % Example:
%   % Perform DCI encoding for RNTI as 100 and output length Edci as 240.
%
%   RNTI = 100;
%   Edci = 240;
%   dcicw = nrDCIEncode(randi([0 1],32,1),RNTI,Edci);
%
%   See also nrDCIDecode, nrPDCCH, nrPDCCHDecode.

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen

%   Reference:
%   [1] 3GPP TS 38.212, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Multiplexing and channel
%   coding. Section 7.3.

    % Validate inputs
    validateInputs(dciBits,rnti,Edci);

    % CRC attachment, Section 7.3.2, [1]
    bitscrcPad = nrCRCEncode([ones(24,1,class(dciBits));dciBits],'24C',rnti);  % First prepend 1s
    cVec = bitscrcPad(25:end,1); % Then, after calculating the CRC, remove the 1s

    % Turn the 24 CRC bits into a MSB first decimal representation for the output
    crc = sum((2.^(23:-1:0)').*logical(cVec(end-23:end,1)));

    % Channel coding, Section 7.3.3, [1]
    encOut = nrPolarEncode(cVec,Edci);

    % Rate matching, Section 7.3.4, [1]
    K = length(cVec);
    dciCW = nrRateMatchPolar(encOut,K,Edci);

end


function validateInputs(dciBits,rnti,Edci)
% Check inputs

    fcnName = 'nrDCIEncode';

    % Validate input DCI message bits, length must be greater than or equal
    % to 12 and less than or equal to 140
    validateattributes(dciBits,{'int8','double'},{'binary','column'}, ...
        fcnName,'DCIBITS');
    Kin = length(dciBits);
    coder.internal.errorIf( Kin<12 || Kin>140, ...
        'nr5g:nrDCIEncode:InvalidInputLength',Kin);

    % Validate radio network temporary identifier RNTI (0...65535)
    validateattributes(rnti,{'numeric'}, ...
        {'scalar','nonnegative','integer','<=',2^16-1},fcnName,'RNTI');

    % Validate rate matched output length which must be greater than or
    % equal to K+24 (i.e., K+CRC)
    validateattributes(Edci,{'numeric'}, ...
        {'scalar','integer','>=',Kin+24},fcnName,'EDCI');

end
