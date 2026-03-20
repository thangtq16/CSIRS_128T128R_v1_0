function [out,csi] = nrEqualizeMMSE(rxSym,Hest,nVar)
%nrEqualizeMMSE MMSE Equalization
%   [OUT,CSI] = nrEqualizeMMSE(RXSYM,HEST,NVAR) returns equalized symbols
%   OUT by performing MMSE equalization on extracted resource elements of a
%   physical channel RXSYM using the estimated channel information HEST and
%   estimated noise variance NVAR. The function also returns soft channel
%   state information CSI.
% 
%   Both OUT and CSI are of same size NRE-by-P. RXSYM is of size NRE-by-R,
%   HEST is of size NRE-by-R-by-P and NVAR is a real nonnegative
%   scalar value.
%   Where,
%   NRE - Number of resource elements of a physical channel whose values
%         are extracted from each K-by-L plane of received grid, where K
%         represents the number of subcarriers and L represents the number
%         of OFDM symbols
%   P   - Number of layers.
%   R   - Number of receive antennas
%
%   Example:
%   % Perform MMSE equalization on extracted resource elements of PBCH
%   % using nrEqualizeMMSE. 
%
%   % Create symbols and indices for a PBCH transmission
%   ncellid = 146;
%   v = 0;
%   E = 864;
%   cw = randi([0 1],E,1);
%   pbchTxSym = nrPBCH(cw,ncellid,v);
%   pbchInd = nrPBCHIndices(ncellid);
%
%   % Generate an empty resource array for one transmitting antenna and
%   % populate it with PBCH symbols using generated PBCH indices
%   carrier = nrCarrierConfig;
%   carrier.NSizeGrid = 20;
%   P = 1;
%   txGrid = nrResourceGrid(carrier,P);
%   txGrid(pbchInd) = pbchTxSym;
%
%   % Perform OFDM modulation
%   txWaveform = nrOFDMModulate(carrier,txGrid);
%
%   % Create channel matrix and apply channel to transmitted waveform
%   R = 4;
%   H = fft(eye(max([P R])));
%   H = H(1:P,1:R);
%   H = H / norm(H);
%   rxWaveform = txWaveform * H;
%
%   % Permute the channel matrix to 1-by-1-by-R-by-P and use it to create
%   % the channel estimation grid of size 240-by-4-by-R-by-P
%   hEstGrid = repmat(permute(H.',[3 4 1 2]),[240 4]);
%   nEst = 0.1;
%
%   % Perform OFDM demodulation
%   rxGrid = nrOFDMDemodulate(carrier,rxWaveform);
%
%   % Use nrExtractResources to extract symbols from received and channel
%   % estimate grids in preparation for PBCH decoding
%   [pbchRxSym,pbchHestSym] = nrExtractResources(pbchInd,rxGrid,hEstGrid);
%   scatterplot(pbchRxSym(:),[],[],'y+');
%   title('Received PBCH constellation');
%
%   % Decode PBCH with extracted resource elements
%   [pbchEqSym,csi] = nrEqualizeMMSE(pbchRxSym,pbchHestSym,nEst);
%   pbchBits = nrPBCHDecode(pbchEqSym,ncellid,v);
%   scatterplot(pbchEqSym(:),[],[],'y+');
%   title('Equalized PBCH constellation');
%
%   See also nrExtractResources, nrPerfectChannelEstimate,
%   nrPerfectTimingEstimate.

%   Copyright 2018-2024 The MathWorks, Inc.

%#codegen

    narginchk(3,3);

    % Gather non-data inputs from remote
    nVar = gather(nVar);

    % Validate received symbols
    fcnName = 'nrEqualizeMMSE';
    validateattributes(rxSym,{'double','single'},{'2d'},fcnName,'RXSYM');

    % Validate estimated channel information
    validateattributes(Hest,{'double','single'},{'3d'},fcnName,'HEST');

    % Validate noise variance
    validateattributes(nVar,{'double','single'},{'scalar','real','nonnegative','finite'},fcnName,'NVAR');

    % Validate the dimensions of rxSym and Hest
    coder.internal.errorIf(size(rxSym,1) ~= size(Hest,1),'nr5g:nrEqualizeMMSE:UnequalNumOfREs',size(rxSym,1),size(Hest,1));
    coder.internal.errorIf(size(rxSym,2) ~= size(Hest,2),'nr5g:nrEqualizeMMSE:UnequalNumOfRxAnts',size(rxSym,2),size(Hest,2));

    if isempty(Hest)
        if isa(rxSym,"gpuArray")
            Hest = gpuArray(Hest);
        end
        out = zeros(0,size(Hest,3),'like',Hest);
        csi = zeros(0,size(Hest,3),'like',real(Hest));
    else
        % Select MMSE algorithm
        algorithm = 0;
        [out,csi] = comm.internal.ofdm.equalizeCore2D(rxSym,permute(Hest,[1 3 2]),nVar,algorithm);
    end

end
