function [blk, err] = nrCodeBlockDesegmentLDPC(cbs,bgn,blklen)
%nrCodeBlockDesegmentLDPC LDPC code block desegmentation and CRC decoding
%   [BLK,ERR] = nrCodeBlockDesegmentLDPC(...) performs the inverse of the
%   code block segmentation and CRC appending (see <a href="matlab:
%   help('nrCodeBlockSegmentLDPC')">nrCodeBlockSegmentLDPC</a>).
%   It concatenates the input code block segments into a single output data
%   block BLK, removing any filler and type 24-B CRC bits that may be
%   present in the process. The results of code block CRC decoding (if
%   applicable) are available in vector ERR.
%
%   [BLK,ERR] = nrCodeBlockDesegmentLDPC(CBS,BGN,BLKLEN) concatenates the
%   input code blocks contained in matrix CBS into an output vector BLK of
%   length BLKLEN, based on selected base graph number BGN. BLKLEN is also
%   used to validate the dimensions of the data in CBS and to calculate the
%   amount of filler to be removed. The data type of CBS can be double or
%   int8. The dimensions of CBS is K-by-C, where K denotes the length of
%   all code blocks and C the number of code blocks. If C is greater than
%   1, each code block is assumed to have a type-24B CRC attached. This CRC
%   is decoded and stripped from each code block prior to output
%   concatenation and the CRC error result is placed in the associated
%   element of vector ERR (the length of ERR is equal to the number of code
%   blocks). If C is 1, no CRC decoding is performed and ERR will be empty.
%   In all cases the number of filler bits removed from the end of a code
%   block is calculated from BLKLEN. If BLKLEN is 0, both BLK and ERR will
%   be empty. The data type of BLK is same as that of CBS.
%
%   Example 1:
%   % Code block segmentation occurs if the input length is greater than
%   % 8448 (base graph number is 1). The example shows how segmentation
%   % and desegmentation happens. The input data of length 10000 gets
%   % segmented into two code blocks of length 5280 given in matrix cbs of
%   % size 5280-by-2. This matrix cbs undergoes filler bits, CRC removal
%   % and concatenation resulting in blk of length 10000.
%
%   bgn = 1;
%   blklen = 10000;
%   cbs = nrCodeBlockSegmentLDPC(randi([0 1],blklen,1),bgn);
%   [blk,err] = nrCodeBlockDesegmentLDPC(cbs,bgn,blklen);
%   blkSize = size(blk)
%   err
%
%   Example 2:
%   % Apply the desegmentation function to a matrix of indices and observe
%   % how the code block bit positions are mapped back into a single
%   % output vector.
%
%   % Create a matrix representing a code block set where each element
%   % contains the linear index of that element within the matrix
%
%   cbs = reshape((1:10560)',[],2);
%   blk = nrCodeBlockDesegmentLDPC(cbs,1,10000);
%   plot(blk);
%   xlabel('Code block bit indices');
%   ylabel('Recovered data bit indices');
%   title('Code block desegmentation operation');
%
%   See also nrCodeBlockSegmentLDPC, nrLDPCDecode, nrRateRecoverLDPC,
%   nrCRCDecode.

%   Copyright 2018-2021 The MathWorks, Inc.

%#codegen

    narginchk(3,3);

    % Validate inputs
    fcnName = 'nrCodeBlockDesegmentLDPC';
    validateattributes(cbs,{'double','int8'},{'2d','real','nonnan'},fcnName,'CBS');
    validateattributes(bgn,{'numeric'},{'scalar','integer','>=',1,'<=',2},fcnName,'BGN');
    validateattributes(blklen,{'numeric'},{'scalar','integer','nonnegative'},fcnName,'BLKLEN');

    % Check for empty cbs or zero value blklen
    if isempty(cbs) || ~blklen
        blk = zeros(0,1,"like",cbs);
        err = zeros(0,1,'uint32');
        return;
    end

    % Get information of code block segments
    chsinfo = nr5g.internal.getCBSInfo(blklen,bgn);

    % Validate dimensions of cbs if there is input for block length
    [K,C] = size(cbs);
    coder.internal.errorIf((C ~= chsinfo.C) || (K ~= chsinfo.K),'nr5g:nrCodeBlockDesegment:InvalidCBSize',K,C,chsinfo.K,chsinfo.C);

    % Remove filler bits
    cbi = cbs(1:end-chsinfo.F,:);

    % Perform code block desegmentation and CRC decoding
    if C == 1
        blk = cbi(:);
        err = zeros(0,1,'uint32');
    else
        [cb,err] = nrCRCDecode(cbi,'24B');
        blk = cb(:);
    end
    blk = blk(1:blklen);

end
