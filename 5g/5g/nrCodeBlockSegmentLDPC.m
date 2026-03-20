function cbs = nrCodeBlockSegmentLDPC(blk,bgn)
%nrCodeBlockSegmentLDPC LDPC code block segmentation and CRC attachment
%   CBS = nrCodeBlockSegmentLDPC(BLK,BGN) splits the input data bit vector
%   BLK into a matrix CBS of code block segments (with filler bits and
%   type-24B CRC appended as appropriate) based on the input base graph
%   number BGN, as defined in TS 38.212 5.2.2. The input BLK can be double,
%   int8 or logical data type. The BGN can be either 1 or 2. LDPC specific
%   code block segmentation occurs in transport blocks (after CRC
%   appending) for LDPC encoded transport channels (DL-SCH, UL-SCH, PCH).
%
%   The segmentation and padding operation ensures that code blocks
%   entering the LDPC coder are no larger than the maximum code block size,
%   Kcb (8448 when BGN is 1 or 3840 when BGN is 2) in length. The
%   segmentation is also to ensure the code blocks are a certain multiple
%   of the LDPC lifting sizes. If the input block length is greater than
%   Kcb, the input block is split into a multiple of smaller code blocks
%   where each individual block also has a type-24B CRC appended to it. The
%   <NULL> filler bits (represented by -1 at the output) are appended to
%   the block so that all blocks in the set have valid lengths. If the
%   input block length is less than or equal to Kcb, no segmentation occurs
%   and no CRC is appended but the single output code block may have <NULL>
%   filler bits appended. The dimensions of output matrix CBS is K-by-C,
%   where K denotes the length of all code blocks and C the number of code
%   blocks. The output matrix CBS is of double or int8 data type, based on
%   the data type of input BLK. If the input BLK is either logical or int8,
%   the output CBS is of int8 data type to allow for the -1 filler bits at
%   the output, else it is of double.
%
%   Example 1:
%   % Code block segmentation occurs if the input length is greater than
%   % the maximum code block size, Kcb, which is 8448 when base graph
%   % number is 1 and 3840 when base graph number is 2.
%   % This example shows the segmentation process.
%
%   cbs1 = nrCodeBlockSegmentLDPC(randi([0,1],4000,1),1); %  No segmentation
%   cbs2 = nrCodeBlockSegmentLDPC(randi([0,1],4000,1),2); %  With segmentation
%   size(cbs1)
%   size(cbs2)
%
%   Example 2:
%   % Use a ramp input to observe how the input data element positions map
%   % onto the element positions in the code blocks.
%
%   cbs = nrCodeBlockSegmentLDPC((1:4000)',2); %  With segmentation
%
%   % Plot the input data indices relative to the code block segment
%   % indices
%   plot(cbs)
%   legend('CBS1','CBS2')
%   xlabel('Code block bit indices');
%   ylabel('Input data bit indices + CRC/filler');
%   title('Code block segmentation operation');
%
%   See also nrCodeBlockDesegmentLDPC, nrCRCEncode, nrLDPCEncode,
%   nrRateMatchLDPC.

%   Copyright 2018-2024 The MathWorks, Inc.

%#codegen

    narginchk(2,2);

    fcnName = 'nrCodeBlockSegmentLDPC';
    typeFlag = islogical(blk) || isUnderlyingType(blk, 'int8');
    % Check empty input with valid type and return typed empty output
    if isempty(blk) && (size(blk,2) < 2)
        validateattributes(blk,{'double','int8','logical'}, ...
            {'real'},fcnName,'BLK');
        if typeFlag
            cbs = cast(zeros(0,1,"like",blk),'int8');
        else
            cbs = zeros(0,1,"like",blk);
        end
        return;
    end

    % Validate inputs
    validateattributes(blk,{'double','int8','logical'},{'column', ...
        'real','nonnan'},fcnName,'BLK');
    validateattributes(bgn,{'numeric'},{'scalar','integer', ...
        '>=',1,'<=',2},fcnName,'BGN');

    % Cast the input to double
    blkd = double(blk);
    blkLen = length(blkd);

    % Get information of code block segments
    chsinfo = nr5g.internal.getCBSInfo(blkLen,bgn);

    % Perform code block segmentation and CRC encoding
    if chsinfo.C == 1
        cbCRC = blkd;
    else
        cb = reshape([blkd; zeros(chsinfo.CBZ*chsinfo.C-blkLen,1)], ...
            chsinfo.CBZ,chsinfo.C);
        cbCRC = nrCRCEncode(cb,'24B');
    end

    % Append filler bits
    cbsd = [cbCRC; -1*ones(chsinfo.F,chsinfo.C)];

    % Cast the output data type based on the input data type
    if typeFlag
        cbs = cast(cbsd,'int8');
    else
        cbs = cbsd;
    end

end
