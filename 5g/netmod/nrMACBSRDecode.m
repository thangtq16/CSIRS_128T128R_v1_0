function [lcgID, lcgBufferRange] = nrMACBSRDecode(lcid, bsr)
%nrMACBSRDecode NR buffer status report (BSR) control element (CE) decoder
%   [LCGID, LCGBUFFERRANGE] = nrMACBSRDecode(LCID, BSR) decodes the
%   received BSR medium access control (MAC) CE, as per 3GPP TS 38.321
%   Section 6.1.3.1 and returns the logical channel group (LCG) IDs and
%   the buffer size range.
%
%   LCID is an integer scalar containing the logical channel ID (LCID) of
%   the BSR. Specify LCID as 59, 60, 61, or 62. The LCID corresponding to
%   each BSR format is as follows:
%   LCID = 59, Short Truncated
%   LCID = 60, Long Truncated
%   LCID = 61, Short
%   LCID = 62, Long
%
%   BSR is a column vector of octets in decimal format containing the
%   BSR. The maximum length of BSR is 9 bytes.
%
%   LCGID is a column vector, containing the LCG IDs reported in the BSR.
%
%   LCGBUFFERRANGE is a row vector of length 2 for short or short truncated
%   BSR. For long BSR, LCGBUFFERRANGE is a matrix of size N-by-2, where N
%   denotes the number of LCGs (as per the LCG bitmap) containing data for
%   transmission. For long truncated BSR, the function returns
%   LCGBUFFERRANGE as a matrix of size M-by-2 where M (M<N) denotes the
%   number of buffer size values reported in BSR CE. The matrix contains
%   the current buffer size values of the corresponding LCGs. The first and
%   second column of the matrix specifies the lower and upper range of the
%   buffer size, respectively. For a buffer size index of 31 (for short or
%   short truncated BSR) and 254 (for long or long truncated BSR), this
%   value contains only the lower bound and the upper bound is set to the
%   largest value of the 32-bit signed integer type. For long BSR, each row
%   of LCGBUFFERRANGE is mapped to the corresponding row in LCGID. For long
%   truncated BSR, the function does not map this value to LCGIDs. The
%   buffer information of one or more LCGIDs are lost in the truncation
%   during a long truncated BSR generation.
%
%   Example 1:
%   % Decode the LCG ID and the buffer size range of a short truncated BSR
%   % format with a BSR value 23.
%   lcid = 59;
%   bsr = 23;
%   [lcgID, lcgBufferRange] = nrMACBSRDecode(lcid, bsr)
%
%   Example 2:
%   % Decode the LCG ID and the buffer size range of a short BSR with
%   % buffer size value as 66.
%   lcid = 61;
%   bsr = 66;
%   [lcgID, lcgBufferRange] = nrMACBSRDecode(lcid, bsr)
%
%   Example 3:
%   % Decode the LCG IDs and the buffer size ranges of a long truncated BSR
%   % with buffer size values reported.
%   lcid = 60;
%   bsr = [145;58;65];
%   [lcgID, lcgBufferRange] = nrMACBSRDecode(lcid, bsr)
%
%   Example 4:
%   % Decode the LCG IDs and the buffer size ranges of a long BSR with
%   % buffer size values reported.
%   lcid = 62;
%   bsr = [36;66;74];
%   [lcgID, lcgBufferRange] = nrMACBSRDecode(lcid, bsr)
%
%   Example 5:
%   % Decode the LCG IDs of a long truncated BSR containing only one octet
%   % representing the LCG bitmap.
%   lcid = 60;
%   bsr = 25;
%   [lcgID, lcgBufferRange] = nrMACBSRDecode(lcid, bsr)
%
%   Example 6:
%   % Decode a short BSR with the buffer size value 0.
%   lcid = 61;
%   bsr = 0;
%   [lcgID, lcgBufferRange] = nrMACBSRDecode(lcid, bsr)
%
%   See also nrMACBSR, nrMACSubPDU.

%   Copyright 2021-2024 The MathWorks, Inc.

%#codegen

% Validate LCID
validateattributes(lcid,{'numeric'}, {'scalar','integer', '>=', 59, ...
    '<=', 62}, 'nrMACBSRDecode', 'lcid');

% Validate MAC BSR
validateattributes(bsr, {'numeric'},{'vector','integer', '>=',0, ...
    '<=',255},'nrMACBSRDecode', 'bsr');

% Short truncated BSR or short BSR
if  lcid == 59 || lcid == 61
    %   Short BSR and short truncated BSR MAC CE contains the following
    %   fields.
    %       LCGID            - Logical channel group id (3 bits).
    %       BufferSizeIndex  - Buffer size level index (5 bits) as per
    %                          3GPP TS 38.321 Table 6.1.3.1-1.

    % Validate the length of BSR
    coder.internal.errorIf(numel(bsr) ~= 1, 'nr5g:nrMACBSRDecode:InvalidBSRLength');

    lcgID = bitshift(bsr, -5); % Logical channel group id
    % Read the buffer size index
    bufferSizeIndex = bitand(bsr, 31);
    lcgBufferRange = zeros(1,2);

    if (bufferSizeIndex(1) ~= 0)&&(bufferSizeIndex(1) ~= 31)
        % Read the buffer size range for buffer size index from 1 till 30
        lcgBufferRange(1) = nr5g.internal.MACConstants.BufferSizeLevelFiveBit(bufferSizeIndex)+1;
        lcgBufferRange(2) = nr5g.internal.MACConstants.BufferSizeLevelFiveBit(bufferSizeIndex+1);
    elseif(bufferSizeIndex(1) == 31)
        % Read the buffer size range for buffer size index 31
        lcgBufferRange(1) = nr5g.internal.MACConstants.BufferSizeLevelFiveBit(bufferSizeIndex)+1;
        lcgBufferRange(2) = 2147483647; % Largest value of the 32-bit signed integer type
    end
else
    %   Long BSR (LCID = 62) and long truncated BSR (LCID = 60) MAC CE
    %   contains the following fields.
    %       LCGBITMAP        - Represents which LCG buffer status is
    %                          reported (8 bits).
    %       BufferSizeIndex  - Buffer size level index (8 bits) as per
    %                          3GPP TS 38.321 Table 6.1.3.1-2.

    % Logical channel group bitmap
    lcgBitmap = bitget(bsr(1),1:8);
    lcgBitIndex = find(lcgBitmap)';
    % Number of LCGs set
    numLCGs = size(lcgBitIndex,1);
    % Length of BSR
    lenBSR = numel(bsr);

    % Validate the length of BSR
    if lcid==62
        coder.internal.errorIf(lenBSR ~= numLCGs+1, 'nr5g:nrMACBSRDecode:InvalidLongBSRLength', numLCGs);
    else
        coder.internal.errorIf(lenBSR >= numLCGs+1, 'nr5g:nrMACBSRDecode:InvalidLongTruncatedBSRLength', numLCGs);
    end

    % Logical channel group id
    lcgID = lcgBitIndex-1;
    lcgBufferRange = zeros(lenBSR-1, 2);

    for byteIndex = 2:lenBSR
        if (bsr(byteIndex) >= 1)&&(bsr(byteIndex) <= 253)
            % Read the buffer size range for buffer size index from 1 till
            % 253
            lcgBufferRange(byteIndex-1,1) = nr5g.internal.MACConstants.BufferSizeLevelEightBit(bsr(byteIndex))+1;
            lcgBufferRange(byteIndex-1,2) = nr5g.internal.MACConstants.BufferSizeLevelEightBit(bsr(byteIndex)+1);
        elseif bsr(byteIndex) == 254
            % Read the buffer size range for buffer size index 254
            lcgBufferRange(byteIndex-1,1) = nr5g.internal.MACConstants.BufferSizeLevelEightBit(bsr(byteIndex))+1;
            lcgBufferRange(byteIndex-1,2) = 2147483647; % Largest value of the 32-bit signed integer type
        elseif bsr(byteIndex) == 255
            % Index 255 is reserved
            coder.internal.error('nr5g:nrMACBSRDecode:ReservedBufferSizeIndex');
        end
    end
end
end