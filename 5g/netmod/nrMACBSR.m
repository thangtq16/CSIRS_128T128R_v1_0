function [lcid, bsr] = nrMACBSR(lcgBufferSize, lcgPriority, paddingBytes)
%nrMACBSR NR buffer status report (BSR) control element (CE) generator
%   [LCID, BSR] = nrMACBSR(LCGBUFFERSIZE) generates a regular or periodic
%   BSR. The function returns a scalar LCID containing the logical channel
%   ID corresponding to the generated BSR. BSR is a vector containing the
%   buffer status report as per 3GPP TS 38.321 Section 6.1.3.1. The BSR
%   format (short, long) is determined by the logical channel group (LCG)
%   buffer size values.
%
%   LCGBUFFERSIZE is a vector containing the list of LCGs buffer size
%   values, arranged in the increasing order of logical channel group IDs
%   (LCGID). The maximum number of LCGs is 8 and the LCG IDs are indexed
%   from 0 to 7. The length of the LCGBUFFERSIZE vector can be limited to
%   the highest LCGID having nonzero data available for transmission.
%
%   LCID is an integer scalar containing the logical channel ID of the BSR
%   generated. LCID is one of the four values 59, 60, 61, and 62. The LCID
%   corresponding to each BSR format is as follows:
%   LCID = 59, Short Truncated
%   LCID = 60, Long Truncated
%   LCID = 61, Short
%   LCID = 62, Long
%
%   BSR is a vector of octets in decimal format containing the buffer
%   status report. For short and short truncated formats, BSR is a
%   nonnegative integer scalar. For long and long truncated formats, BSR is
%   a column vector whose length is determined by the number of LCGs being
%   reported.
%
%   [LCID, BSR] = nrMACBSR(LCGBUFFERSIZE, LCGPRIORITY, PADDINGBYTES) generates
%   a padding BSR CE. The BSR format (short, long, short truncated,
%   long truncated) is determined by the LCG buffer size values, LCG
%   priority, and the number of padding bytes.
%
%   LCGPRIORITY is a vector containing the priority of LCGs in the
%   increasing order of LCG IDs. LCG priority is derived from the highest
%   priority logical channel mapped to it. The priority is specified in
%   the range [1, 16], with a lower value indicating a higher priority. For
%   long truncated BSR format, the LCGs with equal priority are reported in
%   the order of increasing LCGID.
%
%   PADDINGBYTES is an integer scalar with value greater than 1, representing
%   the number of padding bytes available for generating a padding BSR. This
%   parameter limits the number of LCG buffer size values that can be
%   reported in a long truncated BSR.
%
%   Example 1:
%   % Generate a regular or periodic BSR with a single logical
%   % channel group (LCG 2) having buffer size value of 2000 bytes.
%   lcgBufferSize = [0 0 2000];
%   [lcid, bsr] = nrMACBSR(lcgBufferSize);
%
%   Example 2:
%   % Generate a regular or periodic BSR with all the eight LCGs having 
%   % data available for transmission.
%   lcgBufferSize = [20000 700 624 3030 125 1020 3500 2100];
%   [lcid, bsr] = nrMACBSR(lcgBufferSize);
%
%   Example 3:
%   % Generate a padding BSR with all the eight LCGs having data available
%   % for transmission and the padding size is 2 bytes.
%   lcgBufferSize = [1200 3450 7000 4500 5250 6000 2100 9000];
%   lcgPriority = [6 5 3 1 7 8 2 4];
%   paddingBytes = 2;
%   [lcid, bsr] = nrMACBSR(lcgBufferSize, lcgPriority, paddingBytes);
%
%   Example 4:
%   % Generate a padding BSR with all the eight LCGs having data available
%   % for transmission and the padding size is 10 bytes.
%   lcgBufferSize = [1200 3450 7000 4500 5250 6000 2100 9000];
%   lcgPriority = [6 5 3 1 7 8 2 8];
%   paddingBytes = 10;
%   [lcid, bsr] = nrMACBSR(lcgBufferSize, lcgPriority, paddingBytes);
%
%   Example 5:
%   % Generate a periodic BSR of zero bytes when no LCGs have data available
%   % for transmission.
%   [lcid, bsr] = nrMACBSR([0 0 0 0 0 0 0 0]);
%
%   See also nrULSCH, nrPUSCH, nrPUSCHConfig.

%   Copyright 2021-2024 The MathWorks, Inc.

%#codegen

% Validate the number of input arguments
narginchk(1, 3);

% Validate the LCG buffer size values
validateattributes(lcgBufferSize, {'numeric'}, {'integer', 'nonnegative',...
    'vector', 'finite'}, 'nrMACBSR', 'lcgBufferSize');
% Validate the number of LCGs being reported
coder.internal.errorIf(size(lcgBufferSize,2) > 8, 'nr5g:nrMACBSR:InvalidNumLCGs'); 
coder.internal.errorIf(nargin == 2, 'nr5g:nrMACBSR:InvalidNumInputArgs');

if nargin == 3 % Padding BSR
    % Validate the LCG priorities
    validateattributes(lcgPriority, {'numeric'}, {'integer', 'vector',...
        'numel', size(lcgBufferSize,2), '>=', 1, '<=', 16}, 'nrMACBSR', 'lcgPriority')
    % Validate the padding size
    validateattributes(paddingBytes, {'numeric'}, {'scalar', 'integer',...
        '>', 1, 'finite'}, 'nrMACBSR', 'paddingBytes')
else % Regular/Periodic BSR
    paddingBytes = 0;
end

% Determine the index of LCGs with data available
lcgWithDataIdx = find(lcgBufferSize > 0);
% Select the buffer sizes corresponding to the LCGs with nonzero data
lcgBufferSizeWithData = lcgBufferSize(lcgWithDataIdx);
numLCGsWithData = numel(lcgWithDataIdx);

% Create BSR of zero bytes if no LCG has data available for transmission.
% In case of padding BSR, LCID is determined by the padding bytes available
if numLCGsWithData == 0
    bsr = 0;
    if paddingBytes <= 2
        lcid = 61;
    else
        lcid = 62;
    end
elseif numLCGsWithData == 1
    % Generate padding long BSR if padding bytes are greater than or
    % equal to 4 (length of long BSR is 2 bytes + 2 bytes subheader).
    if paddingBytes >= 4
        lcid = 62;
        % Number of bits to represent buffer size field for long BSR format
        % as per Table 6.1.3.1-2 of 3GPP TS 38.321
        bufferSizeFieldLength = 8;
        bufferSizeIndex = getBufferSizeIndex(lcgBufferSizeWithData(1), bufferSizeFieldLength);
        bsr = [bitshift(1, lcgWithDataIdx-1); bufferSizeIndex];
    else
        % Generate a padding short BSR if padding bytes is less than 4
        % bytes or a regular/periodic short BSR if only a single LCG has data available
        lcid = 61;
        % Number of bits to represent buffer size field for short BSR or short truncated
        % BSR formats as per Table 6.1.3.1-1 of 3GPP TS 38.321
        bufferSizeFieldLength = 5;
        bufferSizeIndex = getBufferSizeIndex(lcgBufferSizeWithData(1), bufferSizeFieldLength);
        bsr = bitor(bitshift(lcgWithDataIdx-1, 5), bufferSizeIndex);
    end
else
    % Generate long BSR if multiple LCGs have data available for
    % transmission in case of regular/periodic BSRs. For padding BSR, long
    % BSR is generated if the padding bytes are enough to accommodate a long
    % BSR and its subheader. Size of long BSR is (numLCGWithData + 1) bytes
    % and subheader is 2 bytes long.
    if paddingBytes == 0 || paddingBytes >= (3 + numLCGsWithData)
        lcid = 62;
        % Bitmap to indicate the LCGs being reported in the BSR
        lcgBitmap = 0;
        % Number of bits to represent buffer size field for long/long truncated
        % BSR formats as per Table 6.1.3.1-2 of 3GPP TS 38.321
        bufferSizeFieldLength = 8;
        bufferSizeIndexList = zeros(numLCGsWithData, 1);
        for i = 1 : numLCGsWithData
            lcgBitmap = bitset(lcgBitmap, lcgWithDataIdx(i));
            bufferSizeIndexList(i) = getBufferSizeIndex(lcgBufferSizeWithData(i), bufferSizeFieldLength);
        end
        bsr = [lcgBitmap; bufferSizeIndexList];

        % Generate short truncated BSR if the padding bytes are equal to the
        % size of short BSR plus subheader (2 bytes)
    elseif paddingBytes == 2
        lcid = 59;
        bufferSizeFieldLength = 5;
        % Determine the LCG with highest priority among LCGs with data
        % available
        [~, idx] = min(lcgPriority(lcgWithDataIdx));
        bufferSizeIndex = getBufferSizeIndex(lcgBufferSizeWithData(idx), bufferSizeFieldLength);
        bsr = bitor(bitshift(lcgWithDataIdx(idx)-1, 5), bufferSizeIndex);

        % Generate long truncated BSR if the number of padding bytes is
        % greater than short BSR plus subheader and less than long BSR plus
        % subheader. For long BSR format, BSR CE is of size (1 + number of
        % LCGs being reported) and subheader size is 2 bytes
    else
        lcid = 60;
        % Bitmap to indicate the LCGs with data
        lcgBitmap = 0;
        bufferSizeFieldLength = 8;
        for idx = 1:numLCGsWithData
            lcgBitmap = bitset(lcgBitmap, lcgWithDataIdx(idx));
        end
        % Calculate the number of LCGs which can be included in the BSR
        % after accommodating subheader (2 bytes) and LCG bitmap (1 byte)
        numLCGReported = paddingBytes - 3;
        % If the padding size is such that only the bitmap can be
        % included in the BSR
        if numLCGReported == 0
            bsr = lcgBitmap;
            return;
        end
        % Select the priorities for LCGs with data available for
        % transmission. Sort in the increasing order of priority.
        [~, priorityOrder] = sort(lcgPriority(lcgWithDataIdx));
        % Select the priority order for the LCGs being reported. Sort in
        % the increasing order of LCG ID
        priorityOrder = sort(priorityOrder(1:numLCGReported));
        bufferSizeIndexList = zeros(numLCGReported, 1);
        for i = 1:numLCGReported
            bufferSizeIndexList(i) = getBufferSizeIndex(lcgBufferSizeWithData(priorityOrder(i)), bufferSizeFieldLength);
        end
        bsr = [lcgBitmap; bufferSizeIndexList];
    end
end
end

function bufferSizeIndex = getBufferSizeIndex(bufferSize, bufferSizeFieldLength)
% Performs buffer size row-index calculation
% bufferSize            - Buffer size value in bytes
% bufferSizeFieldLength - Number of bits required to represent the buffer 
%                         size index. For short and short truncated BSR formats, 
%                         5 bits are used. For long and long truncated BSR
%                         formats, 8 bits are used

if bufferSizeFieldLength == 5 % bufferSizeIndex is represented in 5 bits(0 - 31).
    if bufferSize > 150000
        rowIndex = 32;
    else
        % Get the first row index of the table where value in column 1 >
        % bufferSize.
        rowIndex = find((nr5g.internal.MACConstants.BufferSizeLevelFiveBit(:) >= bufferSize), 1);
    end
else % bufferSizeIndex is represented in 8 bits(0 - 255).
    if bufferSize > 81338368
        rowIndex = 255;
    else
        % Get the first row index of the table where value in column 1 >
        % bufferSize.
        rowIndex = find((nr5g.internal.MACConstants.BufferSizeLevelEightBit(1:254) >= bufferSize), 1);
    end
end

% Return zero based index
bufferSizeIndex = rowIndex - 1;
end