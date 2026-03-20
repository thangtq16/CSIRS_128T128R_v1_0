function macSubPDU = nrMACSubPDU(linkDirOrPadding, msgIndex, payload)
%nrMACSubPDU Generates NR MAC subPDU
%   MACSUBPDU = nrMACSubPDU(LINKDIR, MSGINDEX, PAYLOAD) generates a medium
%   access control (MAC) sub protocol data unit (subPDU) as per 3GPP TS
%   38.321 Release 16 Section 6.1.2.
%
%   LINKDIR represents the transmission direction (uplink/downlink) of MAC
%   subPDU.
%   LINKDIR = 0 represents downlink
%   LINKDIR = 1 represents uplink
%
%   MSGINDEX is the message index which represents either the logical
%   channel ID (LCID) or the extended logical channel ID (eLCID) field of
%   subheader in a subPDU. It represents eLCID for a subPDU only if eLCID
%   is applicable to it and its corresponding LCID value is set based on
%   the index range of MSGINDEX. For downlink shared channel (DL-SCH)
%   subPDU, the MSGINDEX is the index value in Table 6.2.1-1, Table
%   6.2.1-1a, or Table 6.2.1-1b of 3GPP TS 38.321 Release 16. For uplink
%   shared channel (UL-SCH) subPDU, the MSGINDEX is the index value in
%   Table 6.2.1-2, Table 6.2.1-2a, or Table 6.2.1-2b of 3GPP TS 38.321
%   Release 16.
%   For DL-SCH/UL-SCH, LCIDs are defined for the index range [0, 62].
%   One-octet eLCIDs are defined for the index range [64, 319] with LCID
%   value set to 34.
%   Two-octet eLCIDs are defined for the index range [320, 65855] with LCID
%   value set to 33.
%
%   PAYLOAD is either a MAC service data unit (SDU) or MAC control element
%   (CE) represented as a vector of octets in decimal format. Set it to []
%   for MAC CE with an empty payload.
%
%   MACSUBPDU = nrMACSubPDU(PADDINGLENGTH) generates a padding MAC subPDU
%   for uplink or downlink direction. LCID is set to 63 in the subheader.
%
%   PADDINGLENGTH is the required MAC padding size in bytes.
%
%   MACSUBPDU is the generated MAC subPDU represented as a column vector of
%   octets in decimal format.
%
%   Example 1:
%   % Generate a MAC subPDU in uplink direction with LCID index value
%   % equal to 20 and SDU payload vector of 4 bytes.
%   linkDir = 1;
%   msgIndex = 20;
%   payload = [64;21;202;238];
%   macSubPDU = nrMACSubPDU(linkDir, msgIndex, payload);
%
%   Example 2:
%   % Generate a MAC subPDU in uplink direction with eLCID index value
%   % equal to 380 and SDU payload vector of 588 bytes.
%   linkDir = 1;
%   msgIndex = 380;
%   payload = ones(588,1);
%   macSubPDU = nrMACSubPDU(linkDir, msgIndex, payload);
%
%   Example 3:
%   % Generate a MAC subPDU in downlink direction with LCID index value
%   % equal to 62 and CE payload vector of 6 bytes.
%   linkDir = 0;
%   msgIndex = 62;
%   payload = [64;21;202;238;244;251];
%   macSubPDU = nrMACSubPDU(linkDir, msgIndex, payload);
%
%   Example 4:
%   % Generate a MAC subPDU in uplink direction with MAC BSR as a payload.
%   linkDir = 1;
%   lcgBufferSize = [0 2000 3000 4000];
%   [msgIndex, bsr] = nrMACBSR(lcgBufferSize);
%   macSubPDU = nrMACSubPDU(linkDir, msgIndex, bsr);
%
%   Example 5:
%   % Generate a MAC subPDU in uplink direction with eLCID index value
%   % equal to 319 and CE payload vector of 2 bytes.
%   linkDir = 1;
%   msgIndex = 319;
%   payload = [64;21];
%   macSubPDU = nrMACSubPDU(linkDir, msgIndex, payload);
%
%   Example 6:
%   % Generate a MAC subPDU in downlink direction with LCID index value
%   % equal to 60 and an empty CE payload.
%   linkDir = 0;
%   msgIndex = 60;
%   payload = [ ];
%   macSubPDU = nrMACSubPDU(linkDir, msgIndex, payload);
%
%   Example 7:
%   % Generate a MAC subPDU with padding payload
%   paddingLength = 5;
%   macSubPDU = nrMACSubPDU(paddingLength);
%
%   See also nrMACBSR.

%   Copyright 2021-2024 The MathWorks, Inc.

%#codegen

% Validate the number of input arguments
narginchk(1, 3);
coder.internal.errorIf(nargin == 2, 'nr5g:nrMACSubPDU:InvalidNumInputArgs');

if nargin == 3
    linkDir = linkDirOrPadding;
    % Validate link direction input (linkDir)
    validateattributes(linkDir, {'numeric'}, {'binary','scalar'}, 'nrMACSubPDU', 'linkDir');

    % Validate message index input (msgIndex)
    validateattributes(msgIndex, {'numeric'}, {'integer', 'nonnegative',...
        'scalar', 'finite', '<=', 65855}, 'nrMACSubPDU', 'msgIndex');

    % Validate payload input (payload)
    if ~isempty(payload)
        validateattributes(payload, {'numeric'}, {'integer', 'nonnegative',...
            'vector', 'finite', '<=', 255}, 'nrMACSubPDU', 'payload');
    end

    if isrow(payload)
        payload = payload';
    end

    % Determine the payload length
    payloadLength = size(payload,1);

    % Initialize subheader
    subheader = 0;

    if linkDir % uplink
        subHeaderFormat = getULSubHeaderInfo(msgIndex, payloadLength);
    else % downlink
        subHeaderFormat = getDLSubHeaderInfo(msgIndex, payloadLength);
    end

    switch subHeaderFormat % Construct subheader
        case 1
            % R/LCID MAC subheader
            % R1    - Value 0 (1 bit)
            % R2    - Value 0 (1 bit)
            % LCID  - (6 bits)
            subheader = msgIndex;
        case 2
            % R/LCID/eLCID MAC subheader
            % R1    - Value 0 (1 bit)
            % R2    - Value 0 (1 bit)
            % LCID  - Value 34 (6 bits)
            % eLCID - (8 bits)

            % LCID is set to 34 for one-octet eLCID index
            % Convert eLCID index to codepoint value for eLCID as per Table
            % 6.2.1-1b, Table 6.2.1-2b of 3GPP TS 38.321 Release 16. eLCID
            % indices ranging from 64 to 319 map to codepoint values from 0
            % to 255
            subheader = [34; msgIndex - 64];
        case 3
            % R/F/LCID/L MAC subheader with 1-byte L field
            % R     - Value 0 (1 bit)
            % F     - Value 0 (1 bit)
            % LCID  - (6 bits)
            % L     - (8 bits)
            subheader = [msgIndex; payloadLength];
        case 4
            % R/F/LCID/eLCID/L MAC subheader with 1-byte L field
            % R     - Value 0 (1 bit)
            % F     - Value 0 (1 bit)
            % LCID  - Value 34 (6 bits)
            % eLCID - (8 bits)
            % L     - (8 bits)
            subheader = [34; msgIndex - 64; payloadLength];
        case 5
            % R/F/LCID/eLCID/L MAC subheader with 1-byte L field
            % R     - Value 0 (1 bit)
            % F     - Value 0 (1 bit)
            % LCID  - Value 33 (6 bits)
            % eLCID - (16 bits)
            % L     - (8 bits)

            % LCID is set to 33 for two-octet eLCID index
            % Convert eLCID index to codepoint value for eLCID as per Table
            % 6.2.1-1a, Table 6.2.1-2a of 3GPP TS 38.321 Release 16. eLCID
            % indices ranging from 320 to 655855 map to codepoint values
            % from 0 to 65535
            subheader = [33; ...
                bitshift(msgIndex - 320, -8); ...
                bitand(msgIndex - 320, 255); ...
                payloadLength];
        case 6
            % R/F/LCID/L MAC subheader with 2-byte L field
            % R     - Value 0 (1 bit)
            % F     - Value 1 (1 bit)
            % LCID  - (6 bits)
            % L     - (16 bits)
            subheader = [msgIndex + 64; ...
                bitshift(payloadLength, -8); ...
                bitand(payloadLength, 255)];
        case 7
            % R/F/LCID/eLCID/L MAC subheader with 2-byte L field
            % R     - Value 0 (1 bit)
            % F     - Value 1 (1 bit)
            % LCID  - Value 34 (6 bits)
            % eLCID - (8 bits)
            % L     - (16 bits)
            subheader = [98; ...
                msgIndex - 64; ...
                bitshift(payloadLength, -8); ...
                bitand(payloadLength, 255)];
        case 8
            % R/F/LCID/eLCID/L MAC subheader with 2-byte L field
            % R     - Value 0 (1 bit)
            % F     - Value 1 (1 bit)
            % LCID  - Value 33 (6 bits)
            % eLCID - (16 bits)
            % L     - (16 bits)
            subheader = [97; ...
                bitshift(msgIndex - 320, -8); ...
                bitand(msgIndex - 320, 255); ...
                bitshift(payloadLength, -8); ...
                bitand(payloadLength, 255)];
    end
else
    paddingLength = linkDirOrPadding;
    % Validate padding length input (paddingLength)
    validateattributes(paddingLength, {'numeric'},{'nonempty', 'scalar', '>=', 1, 'finite', 'integer'}, 'paddingLength');   
    
    % MAC subheader value for padding as per 3GPP TS 38.321 Release 16
    % Table 6.2.1-1 and Table 6.2.1-2.
    subheader = 63;
    % Padding payload
    payload = zeros(paddingLength-1,1);
end
% Concatenate subheader and payload
macSubPDU = [subheader; payload];
end

function subHeaderFormat = getULSubHeaderInfo(msgIndex, payloadLength)
%getULSubHeaderInfo Returns the subheader format of the uplink MAC subPDU
%   subHeaderFormat = getULSubHeaderInfo(MSGINDEX, PAYLOADLENGTH) returns
%   the subheader format of the uplink MAC subPDU
%
%   PAYLOADLENGTH is the length of the payload vector
%
%   SUBHEADERFORMAT is the numeric value that defines the subheader type of
%   a subPDU based on MSGINDEX and PAYLOADLENGTH.
%   SUBHEADERFORMAT = 1 R/LCID MAC subheader
%   SUBHEADERFORMAT = 2 R/LCID/eLCID MAC subheader with 1-byte eLCID field
%   SUBHEADERFORMAT = 3 R/F/LCID/L MAC subheader with 1-byte L field
%   SUBHEADERFORMAT = 4 R/F/LCID/eLCID/L MAC subheader with 1-byte eLCID and 1-byte L field
%   SUBHEADERFORMAT = 5 R/F/LCID/eLCID/L MAC subheader with 2-byte eLCID and 1-byte L field
%   SUBHEADERFORMAT = 6 R/F/LCID/L MAC subheader with 2-byte L field
%   SUBHEADERFORMAT = 7 R/F/LCID/eLCID/L MAC subheader with 1-byte eLCID and 2-byte L field
%   SUBHEADERFORMAT = 8 R/F/LCID/eLCID/L MAC subheader with 2-byte eLCID and 2-byte L field

% Initialize subheader format
subHeaderFormat = 0;

if msgIndex == 0 || msgIndex == 48 || msgIndex == 49 || msgIndex == 52 || msgIndex == 53 || msgIndex == 55 || msgIndex == 57 || msgIndex == 58 || msgIndex == 59 || msgIndex == 61
    % Fixed size MAC SDU or MAC CE payload as per the index value in Table
    % 6.2.1-2 of 3GPP TS 38.321 Release 16.
    % R/LCID MAC subheader
    subHeaderFormat = 1;
    if msgIndex == 0
        coder.internal.errorIf(payloadLength ~= 8, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,8);
    elseif msgIndex == 48
        coder.internal.errorIf(payloadLength ~= 4, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,4);
    elseif msgIndex == 52
        coder.internal.errorIf(payloadLength ~= 6, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,6);
    elseif msgIndex == 53 || msgIndex == 57 || msgIndex == 58
        coder.internal.errorIf(payloadLength ~= 2, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,2);
    elseif msgIndex == 55
        coder.internal.errorIf(payloadLength ~= 0, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,0);
    else
        coder.internal.errorIf(payloadLength ~= 1, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,1);
    end
elseif (msgIndex >= 1 && msgIndex <= 32) || msgIndex == 45 || msgIndex == 46 || msgIndex == 50 || msgIndex == 51 || msgIndex == 54 || msgIndex == 56 || msgIndex == 60 || msgIndex == 62
    % Variable size MAC SDU or MAC CE payload as per the index value in
    % Table 6.2.1-2 of 3GPP TS 38.321 Release 16.
    if payloadLength <= 255
        % R/F/LCID/L MAC subheader with 1-byte L field
        subHeaderFormat = 3;
    elseif payloadLength <= 65535
        % R/F/LCID/L MAC subheader with 2-byte L field
        subHeaderFormat = 6;
    else
        coder.internal.error('nr5g:nrMACSubPDU:InvalidVarSizeSubPDUPayloadLength',payloadLength);
    end
elseif  msgIndex >= 314 && msgIndex <= 319
    % Fixed size MAC CE payload as per the index value in Table
    % 6.2.1-2b of 3GPP TS 38.321 Release 16.
    % R/LCID/eLCID MAC subheader with 1-byte eLCID field
    if msgIndex == 317
        subHeaderFormat = 2;
        coder.internal.errorIf(payloadLength ~= 1, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,1);
    elseif msgIndex == 316 || msgIndex == 318
        subHeaderFormat = 2;
        coder.internal.errorIf(payloadLength ~= 4, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,4);
    else
        % Variable size MAC CE payload as per the index value in Table
        % 6.2.1-2b of 3GPP TS 38.321 Release 16.
        if payloadLength <= 255
            % R/F/LCID/eLCID/L MAC subheader with 1-byte eLCID and 1-byte L field
            subHeaderFormat = 4;
        elseif payloadLength <= 65535
            % R/F/LCID/eLCID/L MAC subheader with 1-byte eLCID and 2-byte L field
            subHeaderFormat = 7;
        else
            coder.internal.error('nr5g:nrMACSubPDU:InvalidVarSizeSubPDUPayloadLength',payloadLength);
        end
    end
elseif msgIndex >= 320 && msgIndex <= 65855
    % Variable size MAC SDU payload as per the index value in Table
    % 6.2.1-2a of 3GPP TS 38.321 Release 16.
    if payloadLength <= 255
        % R/F/LCID/eLCID/L MAC subheader with 2-byte eLCID and 1-byte L field
        subHeaderFormat = 5;
    elseif payloadLength <= 65535
        % R/F/LCID/eLCID/L MAC subheader with 2-byte eLCID and 2-byte L field
        subHeaderFormat = 8;
    else
        coder.internal.error('nr5g:nrMACSubPDU:InvalidVarSizeSubPDUPayloadLength',payloadLength);
    end
else
    % Reserved/Invalid index fields
    coder.internal.error('nr5g:nrMACSubPDU:InvalidMessageIndex');
end
end

function subHeaderFormat = getDLSubHeaderInfo(msgIndex, payloadLength)
%getDLSubHeaderInfo Returns the subheader format of the downlink MAC subPDU
%   subHeaderFormat = getDLSubHeaderInfo(MSGINDEX, PAYLOADLENGTH) returns
%   the subheader format of the downlink MAC subPDU
%
%   PAYLOADLENGTH is the length of the payload vector
%
%   SUBHEADERFORMAT is the numeric value that defines the subheader type of
%   a subPDU based on MSGINDEX and PAYLOADLENGTH.
%   SUBHEADERFORMAT = 1 R/LCID MAC subheader
%   SUBHEADERFORMAT = 2 R/LCID/eLCID MAC subheader with 1-byte eLCID field
%   SUBHEADERFORMAT = 3 R/F/LCID/L MAC subheader with 1-byte L field
%   SUBHEADERFORMAT = 4 R/F/LCID/eLCID/L MAC subheader with 1-byte eLCID and 1-byte L field
%   SUBHEADERFORMAT = 5 R/F/LCID/eLCID/L MAC subheader with 2-byte eLCID and 1-byte L field
%   SUBHEADERFORMAT = 6 R/F/LCID/L MAC subheader with 2-byte L field
%   SUBHEADERFORMAT = 7 R/F/LCID/eLCID/L MAC subheader with 1-byte eLCID and 2-byte L field
%   SUBHEADERFORMAT = 8 R/F/LCID/eLCID/L MAC subheader with 2-byte eLCID and 2-byte L field

% Initialize subheader format
subHeaderFormat = 0;

if msgIndex >= 47 && msgIndex <= 49 || msgIndex == 51 || msgIndex == 52 || (msgIndex >= 56 && msgIndex <= 62)
    % Fixed size MAC CE payload as per the index value in Table 6.2.1-1 of
    % 3GPP TS 38.321 Release 16.
    % R/LCID MAC subheader
    subHeaderFormat = 1;
    if msgIndex == 47 || msgIndex == 48 || msgIndex == 51 || msgIndex == 52
        coder.internal.errorIf(payloadLength ~= 2, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,2);
    elseif msgIndex == 49
        coder.internal.errorIf(payloadLength ~= 3, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,3);
    elseif msgIndex == 56 || msgIndex == 58 || msgIndex == 61
        coder.internal.errorIf(payloadLength ~= 1, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,1);
    elseif msgIndex == 57
        coder.internal.errorIf(payloadLength ~= 4, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,4);
    elseif msgIndex == 62
        coder.internal.errorIf(payloadLength ~= 6, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,6);
    else
        coder.internal.errorIf(payloadLength ~= 0, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,0);
    end
elseif (msgIndex >= 0 && msgIndex <= 32) || msgIndex == 50 || (msgIndex >= 53 && msgIndex <= 55)
    % Variable size MAC SDU or MAC CE payload as per the index value in
    % Table 6.2.1-1 of 3GPP TS 38.321 Release 16.
    if payloadLength <= 255
        % R/F/LCID/L MAC subheader with 1-byte L field
        subHeaderFormat = 3;
    elseif payloadLength <= 65535
        % R/F/LCID/L MAC subheader with 2-byte L field
        subHeaderFormat = 6;
    else
        coder.internal.error('nr5g:nrMACSubPDU:InvalidVarSizeSubPDUPayloadLength',payloadLength);
    end
elseif  msgIndex >=309 && msgIndex <=319
    % Fixed size MAC CE payload as per the index value in Table
    % 6.2.1-1b of 3GPP TS 38.321 Release 16.
    % R/LCID/eLCID MAC subheader with 1-byte eLCID field
    if msgIndex == 311
        subHeaderFormat = 2;
        coder.internal.errorIf(payloadLength ~= 3, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,3);
    elseif msgIndex == 315
        subHeaderFormat = 2;
        coder.internal.errorIf(payloadLength ~= 1, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,1);
    elseif msgIndex == 316 || msgIndex == 319
        subHeaderFormat = 2;
        coder.internal.errorIf(payloadLength ~= 2, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,2);
    elseif msgIndex == 318
        subHeaderFormat = 2;
        coder.internal.errorIf(payloadLength ~= 4, 'nr5g:nrMACSubPDU:InvalidFixedSizeSubPDUPayloadLength',payloadLength,msgIndex,4);
    else
        % Variable size MAC CE payload as per the index value in Table
        % 6.2.1-1b of 3GPP TS 38.321 Release 16.
        if payloadLength <= 255
            % R/F/LCID/eLCID/L MAC subheader with 1-byte eLCID and 1-byte L field
            subHeaderFormat = 4;
        elseif payloadLength <= 65535
            % R/F/LCID/eLCID/L MAC subheader with 1-byte eLCID and 2-byte L field
            subHeaderFormat = 7;
        else
            coder.internal.error('nr5g:nrMACSubPDU:InvalidVarSizeSubPDUPayloadLength',payloadLength);
        end
    end
elseif msgIndex >= 320 && msgIndex <= 65855
    % Variable size MAC SDU payload as per the index value in Table
    % 6.2.1-1a of 3GPP TS 38.321 Release 16.
    if payloadLength <= 255
        % R/F/LCID/eLCID/L MAC subheader with 2-byte eLCID and 1-byte L field
        subHeaderFormat = 5;
    elseif payloadLength <= 65535
        % R/F/LCID/eLCID/L MAC subheader with 2-byte eLCID and 2-byte L field
        subHeaderFormat = 8;
    else
        coder.internal.error('nr5g:nrMACSubPDU:InvalidVarSizeSubPDUPayloadLength',payloadLength);
    end
else
    % Reserved/Invalid index fields
    coder.internal.error('nr5g:nrMACSubPDU:InvalidMessageIndex');
end
end

