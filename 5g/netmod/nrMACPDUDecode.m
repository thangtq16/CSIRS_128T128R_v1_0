function [msgIndexList,payloadList,varargout] = nrMACPDUDecode(macPDU,linkDir,softErrorFlag)
%nrMACPDUDecode Decode NR MAC PDU
%   [MSGINDEXLIST,PAYLOADLIST] = nrMACPDUDecode(MACPDU,LINKDIR) decodes the
%   NR medium access control (MAC) protocol data unit (PDU), MACPDU, as per
%   3GPP TS 38.321 Release 16. The function returns the message indices,
%   MSGINDEXLIST, that represent logical channel IDs (LCIDs) or extended
%   logical channel IDs (eLCIDs) and payloads of corresponding MAC subPDUs,
%   PAYLOADLIST.
%
%   [...,HEADERLENGTHList] = nrMACPDUDecode(...) also returns the header lengths
%   of the  MAC subPDUs in addition to any of the output arguments in previous
%   syntaxes.
%
%   MACPDU is a vector of octets in decimal format.
%
%   LINKDIR represents the transmission direction (uplink/downlink) of MAC
%   PDU.
%       LINKDIR = 0 represents downlink
%       LINKDIR = 1 represents uplink
%
%   MSGINDEXLIST is a vector of message indices where each message index
%   represents either the LCID or the eLCID field of the subheader in a
%   subPDU. For downlink shared channel (DL-SCH) subPDUs, the MSGINDEXLIST
%   contains the index values of LCIDs or eLCIDs as per 3GPP TS 38.321
%   Release 16 Table 6.2.1-1, Table 6.2.1-1a, or Table 6.2.1-1b. For uplink
%   shared channel (UL-SCH) subPDUs, the MSGINDEXLIST contains the index
%   values of LCIDs or eLCIDs as per 3GPP TS 38.321 Release 16 Table
%   6.2.1-2, Table 6.2.1-2a, or Table 6.2.1-2b.
%
%   PAYLOADLIST is a cell array that consists of either MAC service data
%   units (SDUs), MAC control elements (CEs), or padding payloads, present
%   in MAC subPDUs.
%
%   HEADERLENGTHLIST is an array containing the header lengths of MAC subPDUs.
%
%   [MSGINDEXLIST,PAYLOADLIST] = nrMACPDUDecode(MACPDU,LINKDIR,SOFTERRORFLAG)
%   also specifies how the errors in decoding are to be handled.
%
%   SOFTERRORFLAG represents how the PDU decoding errors are to be handled
%       SOFTERRORFLAG = true   When the function encounters a corrupted PDU,
%                              the error message is not displayed, and the 
%                              function returns empty values.
%       SOFTERRORFLAG = false  When the function encounters a corrupted PDU,
%                              the corresponding error message is displayed.
%                              This is the default value.
%
%   Example 1:
%   % Decode a MAC PDU that contains five MAC subPDUs in the downlink
%   % direction with LCID values 61, 53, 20, 30, and 63, defined in the
%   % subheaders. These subPDUs carry a fixed size CE (1 byte), variable
%   % size CE (289 bytes), SDU (4 bytes and 512 bytes), and padding (4 bytes),
%   % payloads respectively.
%   macPDU = [61;17; ...
%             117;1;33;ones(289,1); ...
%             20;4;64;21;202;238; ...
%             94;2;0;ones(512,1); ...
%             63;0;0;0;0];
%   linkDir = 0;
%   [msgIndexList,payloadList] = nrMACPDUDecode(macPDU,linkDir);
%
%   Example 2:
%   % Decode a MAC PDU that contains five MAC subPDUs in the uplink
%   % direction with LCID values 20, 17, 55, 54, and 63, defined in the
%   % subheaders. These subPDUs carry SDU (344 bytes and 24 bytes), fixed
%   % size CE (0 bytes), variable size CE (4 bytes), and padding (0 bytes),
%   % payloads respectively.
%   macPDU = [84;1;88;ones(344,1); ...
%             17;24;ones(24,1); ...
%             55; ...
%             54;4;ones(4,1); ...
%             63];
%   linkDir = 1;
%   [msgIndexList,payloadList] = nrMACPDUDecode(macPDU,linkDir);
%
%   Example 3:
%   % Decode a MAC PDU that contains four MAC subPDUs in the uplink
%   % direction with LCID and eLCID values 20, 2487, 58, and 319, defined in
%   % the subheaders. These subPDUs carry SDU (4 bytes and 166 bytes), fixed
%   % size CE (2 bytes), and variable size CE (349 bytes), payloads respectively.
%   macPDU = [20;4;64;21;202;238; ...
%             33;8;119;166;ones(166,1); ...
%             58;20;154; ...
%             98;255;1;93;ones(349,1)];
%   linkDir = 1;
%   softErrorFlag = false;
%   [msgIndexList,payloadList] = nrMACPDUDecode(macPDU,linkDir,softErrorFlag);
%
%   Example 4:
%   % Decode a MAC PDU that contains one MAC subPDU in the downlink direction
%   % with LCID value 38 defined in the subheader and carrying a payload
%   % (7 bytes).
%   macPDU = [38;ones(7,1)];
%   linkDir = 0;
%   softErrorFlag = true;
%   [msgIndexList,payloadList] = nrMACPDUDecode(macPDU,linkDir,softErrorFlag);
%
%   Example 5:
%   % Decode a MAC PDU that contains one MAC subPDU in the uplink direction
%   % with eLCID value 68 defined in the subheader and carrying a payload
%   % (122 bytes).
%   macPDU = [34;68;ones(122,1)];
%   linkDir = 1;
%   softErrorFlag = true;
%   [msgIndexList,payloadList] = nrMACPDUDecode(macPDU,linkDir,softErrorFlag);
%
%   Example 6:
%   % Decode a MAC PDU that contains one MAC subPDU in the downlink direction
%   % with LCID value 63 and carrying a padding payload (7 bytes).
%   macPDU = [63;0;0;0;0;0;0;0];
%   linkDir = 0;
%   softErrorFlag = false;
%   [msgIndexList,payloadList] = nrMACPDUDecode(macPDU,linkDir,softErrorFlag);
%
%   See also nrMACSubPDU.

%   Copyright 2022-2024 The MathWorks, Inc.

%#codegen

narginchk(2,3);
nargoutchk(0,3);

% Get the error handling option
errorHandlingFlag = false;
if nargin == 3
    errorHandlingFlag = softErrorFlag;
end

% Validate the input arguments
validateInputs(macPDU,linkDir,errorHandlingFlag);

macPDULength = numel(macPDU);
macPDU = double(macPDU);

% Initialize the arrays for codegen support
maxSubPDUs = max(16, ceil(macPDULength/10)); % Maximum subPDUs realistically possible
msgIndices = zeros(maxSubPDUs,1);
payloads = cell(maxSubPDUs,1);
headerLengths = zeros(maxSubPDUs,1);
if nargout == 3
    varargout{1} = []; % Header length list of MAC subPDUs
end
for i = 1:maxSubPDUs
    payloads{i,1} = 0;
end

% Initialize the list of invalid message indices based on the link
% direction
if linkDir % Uplink
    invalidMsgIndices = [35:44 47 64:313];
else
    invalidMsgIndices = [35:46 64:308];
end

numSubPDUs = 0;
% Starting byte index of a subPDU
subPDUStartIndex = 1;

% Read the subPDUs from a MAC PDU
while subPDUStartIndex <= macPDULength
    % Extract the subPDU information starting at a particular byte index in
    % a MAC PDU
    if linkDir % Uplink
        [msgIndex,headerLength,payloadLength] = getULMACSubPDU(macPDU,subPDUStartIndex);
    else % Downlink
        [msgIndex,headerLength,payloadLength] = getDLMACSubPDU(macPDU,subPDUStartIndex);
    end

    % Check if a variable length MAC subPDU has length field defined in the
    % subheader
    if payloadLength < 0
        coder.internal.errorIf(errorHandlingFlag == false,'nr5g:nrMACPDUDecode:InsufficientMACPDULength',macPDULength,payloadLength);
        payloadList = {};
        msgIndexList = [];
        return;
    end

    % Validate payloadLength
    if payloadLength > (macPDULength - subPDUStartIndex)
        coder.internal.errorIf(errorHandlingFlag == false,'nr5g:nrMACPDUDecode:InvalidSubPDUPayloadLength',payloadLength);
        payloadList = {};
        msgIndexList = [];
        return;
    end

    % Validate msgIndex output
    if (msgIndex < 0 || msgIndex > 65855) || any(msgIndex == invalidMsgIndices)
        % Reserved/Invalid index fields
        if linkDir % Uplink
            coder.internal.errorIf(errorHandlingFlag == false,'nr5g:nrMACPDUDecode:InvalidULMessageIndex',msgIndex);
        else % Downlink
            coder.internal.errorIf(errorHandlingFlag == false,'nr5g:nrMACPDUDecode:InvalidDLMessageIndex',msgIndex);
        end
        payloadList = {};
        msgIndexList = [];
        return;
    end

    if msgIndex == 63 % Padding
        subPDULength = (macPDULength - subPDUStartIndex) + 1;
        headerLength = 1;
        payloadLength = subPDULength - 1;
    else
        subPDULength = headerLength + payloadLength;
    end

    if payloadLength ~= 0
        subPDUPayload = macPDU(subPDUStartIndex+headerLength : subPDUStartIndex+subPDULength-1);
    else
        subPDUPayload = [];
    end

    subPDUStartIndex = subPDUStartIndex + subPDULength;
    numSubPDUs = numSubPDUs + 1;
    msgIndices(numSubPDUs,1) = msgIndex;
    payloads{numSubPDUs,1} = subPDUPayload;
    headerLengths(numSubPDUs,1) = headerLength;
end

% Initialize the output list to support codegen
msgIndexList = msgIndices(1:numSubPDUs,1);
headerLengthList = headerLengths(1:numSubPDUs,1);
truncatedPayloadList = cell(numSubPDUs,1);
for i = 1:numSubPDUs
    truncatedPayloadList{i,1} = payloads{i,1};
end
payloadList = truncatedPayloadList;
if nargout == 3
    varargout{1} = headerLengthList; % Header length list of MAC subPDUs
end
end

function validateInputs(macPDU,linkDir,errorHandlingFlag)
% Validates the given input arguments

% MAC PDU must be nonempty, vector of octets in decimal format
validateattributes(macPDU,{'numeric'},{'nonempty','vector','>=',0,'<=',255,'integer'},'nrMACPDUDecode','macPDU');

% Link direction must be either 0 or 1
validateattributes(linkDir,{'numeric','logical'},{'nonempty','scalar','binary'},'nrMACPDUDecode','linkDir');

% Validate soft error flag
validateattributes(errorHandlingFlag,{'numeric','logical'},{'nonempty','scalar','binary'},'nrMACPDUDecode','errorHandlingFlag');
end

function [msgIndex,headerLength,payloadLength] = getULMACSubPDU(macPDU,byteIndex)
% getULMACSubPDU returns the information of the uplink MAC subPDU
%   [msgIndex,headerLength,payloadLength] = getULMACSubPDU(macPDU,byteIndex)
%   returns the information of MAC subPDU, given its starting byte index in
%   the uplink MAC PDU.
%
%   BYTEINDEX is the starting index value of MAC subPDU in the uplink MAC
%   PDU.
%
%   HEADERLENGTH is the length of the header field in a MAC subPDU in bytes.
%
%   PAYLOADLENGTH is the length of the payload in a MAC subPDU in bytes.

% Initial header length and payload length. For reserved/unknown LCID,
% output these as 0
headerLength = 0;
payloadLength = 0;

% Read the first octet. Its 6 bits (bit 3:8) indicate LCID
firstByte = macPDU(byteIndex);
lcid = bitand(firstByte, 63);

msgIndex = lcid;
macPDULength = numel(macPDU);
if (lcid >= 1 && lcid <= 32) || any(lcid == [45 46 50 51 54 56 60 62])
    % Variable size MAC SDU or MAC CE payload as per the CodePoint value in
    % Table 6.2.1-2 of 3GPP TS 38.321 Release 16. 
    % R/F/LCID/L(length) MAC subheader with 1-byte or 2-byte L field
    F = bitand(bitshift(firstByte, -6), 1); % Get 2nd bit of first byte
    if F == 0
        % R/F/LCID/L MAC subheader with 1-byte L field
        headerLength = 2;
        % Read length of the payload from 1st byte
        if (macPDULength - byteIndex) < 1
            payloadLength = -1;
            return;
        end
        payloadLength = macPDU(byteIndex + 1);
    else
        % R/F/LCID/L MAC subheader with 2-byte L field
        headerLength = 3;
        % Read length of the payload from 2nd and 3rd bytes
        if (macPDULength - byteIndex) < 2
            payloadLength = -1;
            return;
        end
        payloadLength = bitshift(macPDU(byteIndex + 1), 8) + macPDU(byteIndex + 2);
    end
elseif lcid == 0 || (lcid >= 48 && lcid <= 61)
    % Fixed size MAC SDU or MAC CE payload as per the CodePoint value in
    % Table 6.2.1-2 of 3GPP TS 38.321 Release 16. 
    % R/LCID MAC subheader
    headerLength = 1;
    if lcid == 0
        payloadLength = 8;
    elseif lcid == 48
        payloadLength = 4;
    elseif lcid == 52
        payloadLength = 6;
    elseif lcid == 53 || lcid == 57 || lcid == 58
        payloadLength = 2;
    elseif lcid == 55
        payloadLength = 0;
    else
        payloadLength = 1;
    end
elseif lcid == 34
    if (macPDULength - byteIndex) < 1
        msgIndex = -1;
        return;
    end
    elcid = macPDU(byteIndex + 1);
    if elcid == 252 || elcid == 253 || elcid == 254
        % Fixed size MAC CE payload as per the CodePoint value in Table
        % 6.2.1-2b of 3GPP TS 38.321 Release 16. 
        % R/LCID/eLCID MAC subheader with 1-byte eLCID field
        headerLength = 2;
        if elcid == 253
            payloadLength = 1;
        else
            payloadLength = 4;
        end
    elseif elcid == 250 || elcid == 251 || elcid == 255
        % Variable size MAC CE payload as per the CodePoint value in Table
        % 6.2.1-2b of 3GPP TS 38.321 Release 16.
        F = bitand(bitshift(firstByte, -6), 1); % Get 2nd bit of first byte
        if F == 0
            % R/F/LCID/eLCID/L MAC subheader with 1-byte eLCID and 1-byte L
            % field
            headerLength = 3;
            % Read length of the payload from 2nd byte
            if (macPDULength - byteIndex) < 2
                payloadLength = -1;
                return;
            end
            payloadLength = macPDU(byteIndex + 2);
        else
            % R/F/LCID/eLCID/L MAC subheader with 1-byte eLCID and 2-byte L
            % field
            headerLength = 4;
            % Read length of the payload from 3rd and 4th bytes
            if (macPDULength - byteIndex) < 3
                payloadLength = -1;
                return;
            end
            payloadLength = bitshift(macPDU(byteIndex + 2), 8) + macPDU(byteIndex + 3);
        end
    end
    msgIndex = elcid + 64;
elseif lcid == 33
    % Variable size MAC SDU payload as per the CodePoint value in Table
    % 6.2.1-2a of 3GPP TS 38.321 Release 16.
    if (macPDULength - byteIndex) < 2
        msgIndex = -1;
        return;
    end
    elcid = bitshift(macPDU(byteIndex + 1), 8) + macPDU(byteIndex + 2);
    F = bitand(bitshift(firstByte, -6), 1); % Get 2nd bit of first byte
    if F == 0
        % R/F/LCID/eLCID/L MAC subheader with 2-byte eLCID and 1-byte L
        % field
        headerLength = 4;
        % Read length of the payload from 3rd byte
        if (macPDULength - byteIndex) < 3
            payloadLength = -1;
            return;
        end
        payloadLength = macPDU(byteIndex + 3);
    else
        % R/F/LCID/eLCID/L MAC subheader with 2-byte eLCID and 2-byte L
        % field
        headerLength = 5;
        % Read length of the payload from 4th and 5th bytes
        if (macPDULength - byteIndex) < 4
            payloadLength = -1;
            return;
        end
        payloadLength = bitshift(macPDU(byteIndex + 3), 8) + macPDU(byteIndex + 4);
    end
    msgIndex = elcid + 320;
end
end

function [msgIndex,headerLength,payloadLength] = getDLMACSubPDU(macPDU,byteIndex)
% getDLMACSubPDU returns the information of the downlink MAC subPDU
%   [msgIndex,headerLength,payloadLength] =
%   getDLMACSubPDU(macPDU,byteIndex) returns the information of MAC subPDU,
%   given its starting byte index in the downlink MAC PDU.

% Initial header length and payload length. For reserved/unknown LCID, output these as 0
headerLength = 0;
payloadLength = 0;

% Read the first octet. Its 6 bits (bit 3:8) indicate LCID
firstByte = macPDU(byteIndex);
lcid = bitand(firstByte, bitshift(255, -2));

msgIndex = lcid;
macPDULength = numel(macPDU);
if (lcid >= 0 && lcid <= 32) || lcid == 50 || (lcid >= 53 && lcid <= 55)
    % Variable size MAC SDU or MAC CE payload as per the CodePoint value in
    % Table 6.2.1-1 of 3GPP TS 38.321 Release 16. 
    % R/F/LCID/L(length) MAC subheader with 1-byte or 2-byte L field
    F = bitand(bitshift(firstByte, -6), 1); % Get 2nd bit of first byte
    if F == 0
        % R/F/LCID/L MAC subheader with 1-byte L field
        headerLength = 2;
        % Reade length of the payload from 1st byte
        if (macPDULength - byteIndex) < 1
            payloadLength = -1;
            return;
        end
        payloadLength = macPDU(byteIndex + 1);
    else
        % R/F/LCID/L MAC subheader with 2-byte L field
        headerLength = 3;
        % Read length of the payload from 2nd and 3rd byte
        if (macPDULength - byteIndex) < 2
            payloadLength = -1;
            return;
        end
        payloadLength = bitshift(macPDU(byteIndex + 1), 8) + macPDU(byteIndex + 2);
    end
elseif (lcid >= 47 && lcid <= 62)
    % Fixed size MAC CE payload as per the CodePoint value in Table 6.2.1-1
    % of 3GPP TS 38.321 Release 16.
    % R/LCID MAC subheader
    headerLength = 1;
    if lcid == 47 || lcid == 48 || lcid == 51 || lcid == 52
        payloadLength = 2;
    elseif lcid == 49
        payloadLength = 3;
    elseif lcid == 56 || lcid == 58 || lcid == 61
        payloadLength = 1;
    elseif lcid == 57
        payloadLength = 4;
    elseif lcid == 62
        payloadLength = 6;
    else
        payloadLength = 0;
    end
elseif lcid == 34
    if (macPDULength - byteIndex) < 1
        msgIndex = -1;
        return;
    end
    elcid = macPDU(byteIndex + 1);
    if elcid == 247 || elcid == 251 || elcid == 252 || elcid == 254 || elcid == 255
        % Fixed size MAC CE payload as per the CodePoint value in Table
        % 6.2.1-1b of 3GPP TS 38.321 Release 16. 
        % R/LCID/eLCID MAC subheader with 1-byte eLCID field
        headerLength = 2;
        if elcid == 247
            payloadLength = 3;
        elseif elcid == 251
            payloadLength = 1;
        elseif elcid == 252 || elcid == 255
            payloadLength = 2;
        else
            payloadLength = 4;
        end
    elseif (elcid >= 245) && (elcid <= 253)
        % Variable size MAC CE payload as per the CodePoint value in Table
        % 6.2.1-1b of 3GPP TS 38.321 Release 16.
        F = bitand(bitshift(firstByte, -6), 1); % Get 2nd bit of first byte
        if F == 0
            % R/F/LCID/eLCID/L MAC subheader with 1-byte eLCID and 1-byte L
            % field
            headerLength = 3;
            % Read length of the payload from 2nd byte
            if (macPDULength - byteIndex) < 2
                payloadLength = -1;
                return;
            end
            payloadLength = macPDU(byteIndex + 2);
        else
            % R/F/LCID/eLCID/L MAC subheader with 1-byte eLCID and 2-byte L
            % field
            headerLength = 4;
            % Read length of the payload from 3rd and 4th bytes
            if (macPDULength - byteIndex) < 3
                payloadLength = -1;
                return;
            end
            payloadLength = bitshift(macPDU(byteIndex + 2), 8) + macPDU(byteIndex + 3);
        end
    end
    msgIndex = elcid + 64;
elseif lcid == 33
    % Variable size MAC SDU payload as per the CodePoint value in Table
    % 6.2.1-1a of 3GPP TS 38.321 Release 16.
    if (macPDULength - byteIndex) < 2
        msgIndex = -1;
        return;
    end
    elcid = bitshift(macPDU(byteIndex + 1), 8) + macPDU(byteIndex + 2);
    F = bitand(bitshift(firstByte, -6), 1); % Get 2nd bit of first byte
    if F == 0
        % R/F/LCID/eLCID/L MAC subheader with 2-byte eLCID and 1-byte L
        % field
        headerLength = 4;
        % Read length of the payload from 3rd byte
        if (macPDULength - byteIndex) < 3
            payloadLength = -1;
            return;
        end
        payloadLength = macPDU(byteIndex + 3);
    else
        % R/F/LCID/eLCID/L MAC subheader with 2-byte eLCID and 2-byte L
        % field
        headerLength = 5;
        % Read length of the payload from 4th and 5th bytes
        if (macPDULength - byteIndex) < 4
            payloadLength = -1;
            return;
        end
        payloadLength = bitshift(macPDU(byteIndex + 3), 8) + macPDU(byteIndex + 4);
    end
    msgIndex = elcid + 320;
end
end
