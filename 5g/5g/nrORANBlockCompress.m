function [cGrid,cParam] = nrORANBlockCompress(grid,method,cIQWidth,varargin)
%nrORANBlockCompress O-RAN block compression
%   [CGRID,CPARAM] = nrORANBlockCompress(GRID,METHOD,CIQWIDTH,IQWIDTH)
%   performs open radio access network (O-RAN) block compression for
%   U-Plane IQ data, returning the compressed grid CGRID and the
%   compression parameter CPARAM. This function implements the block
%   floating point (BFP), block scaling, and mu-law compression methods, as
%   defined in TS O-RAN.WG4.CUS Annex A.1, A.2, and A.3, respectively.
%
%   [CGRID,CPARAM] = nrORANBlockCompress(GRID,METHOD,CIQWIDTH) performs
%   only BFP or mu-law compression.
%
%   GRID is a K-by-L-by-P complex array, where K is the number of
%   subcarriers, L is the number of OFDM symbols, and P is the number of
%   antennas. K must be a multiple of 12, which corresponds to the number
%   of REs in a PRB. The IQ samples in the array are singles or doubles
%   with a bit width specified by IQWIDTH.
%
%   METHOD is the block compression method. The options are: 
%       - 'BFP' for block floating point compression
%       - 'muLaw' for mu-law compression
%       - 'blockScaling' for block scaling compression
%
%   CIQWIDTH is an integer (1...16) or a K/12-by-L-by-P integer array with
%   values in the range 1...16. The CIQWIDTH parameter specifies the
%   compressed IQ samples bit width (including the sign bit). If CIQWIDTH
%   is an integer, the function uses the same bit width for all IQ samples
%   in GRID. If CIQWIDTH is an array, then each element of the array
%   specifies a bit width per PRB in CGRID.
%
%   IQWIDTH is an integer (1...32) that specifies the IQ samples bit width
%   in GRID before compression. For mu-law compression, this input must be
%   16. For BFP compression, the function ignores this input.
%
%   CGRID is a K-by-L-by-P complex array. The IQ samples in the array can
%   be singles or doubles.
% 
%   CPARAM is a K/12-by-L-by-P array. This output is a computational
%   parameter specific to the compression method: the common exponent
%   applied per compressed PRB in block floating point compression, the
%   common scale factor applied per compressed PRB in block scaling
%   compression, or the common shift applied per compressed PRB in mu-Law
%   compression.
%
%   Example: 
%   % Generate the grid to compress, scale the IQ samples in the grid
%   % to the specified bit width, and apply the specified compression 
%   % method.
%
%   % Generate resource grid
%   cfg = nrDLCarrierConfig;
%   [waveform,info] = nrWaveformGenerator(cfg);
%   grid = info.ResourceGrids.ResourceGridBWP;
%
%   % Set the IQ samples bit width before compression
%   IQWidth = 16;
%
%   % Scale the IQ samples using IQWidth
%   peak = max(abs([real(grid(:)); imag(grid(:))]));
%   scaleFactor = peak / (2^(IQWidth-1)-1);
%   scaledGrid = round(grid / scaleFactor);
%
%   % Set the compressed IQ samples bit width
%   cIQWidth = 9;
%
%   % Select the compression method
%   method = 'BFP';
%
%   % Apply compression 
%   [cGrid,cParam] = nrORANBlockCompress(scaledGrid,method,cIQWidth);
%
%   See also nrORANBlockDecompress.

%   Copyright 2022-2025 The MathWorks, Inc.

%#codegen

    narginchk(3,4);

    % Validate input arguments
    [method,cIQWidth,IQWidth] = validateInputs(grid,method,cIQWidth,varargin{:});
  
    % Reshape grid into a matrix where each column is a PRB and reshape
    % cIQWidth into a vector
    uPRBs = reshape(grid,12,[]);
    cIQWidthSize = size(cIQWidth);
    cIQWidth = reshape(cIQWidth,1,[]);

    % Compress each PRB (column) available in grid
    switch method 
        case 'BFP'
            [cPRBs,cParam] = BFPCompress(uPRBs,cIQWidth); 
        case 'blockScaling' 
            [cPRBs,cParam] = blockScalingCompress(uPRBs,cIQWidth,IQWidth);
        otherwise  % 'muLaw'
            [cPRBs,cParam] = muLawCompress(uPRBs,cIQWidth);
    end 

    % Reshape back to the original size
    cGrid = reshape(cPRBs,size(grid));
    cParam = reshape(cParam,cIQWidthSize);
end

function [cPRB,exponent] = BFPCompress(uPRB,cIQWidth)
% Block floating point compression method as per Annex A in TS O-RAN.WG4.CUS
    
    % Find the maximum and minimum values within the real and imaginary
    % parts of the resource elements (REs) available per PRB
    maxV = max([real(uPRB); imag(uPRB)]);
    minV = min([real(uPRB); imag(uPRB)]);
    
    % Determine the maximum absolute value per PRB considering that the
    % negative value should be offset by 1 to fit the same number of IQ
    % bits
    maxValue = max(maxV, abs(minV)-1);

    % Calculate the exponent per PRB and limit that to be positive
    rawExp = floor(log2(maxValue)+1);
    exponent = max(rawExp - cIQWidth + 1, 0);
    
    % Determine the right shift value per PRB
    scaleFactor = 2.^(-exponent);
    
    % Scale and truncate each RE in every PRB
    cPRB = fix(scaleFactor .* uPRB); 
end

function [cPRB,sblockScaler] = blockScalingCompress(uPRB,cIQWidth,IQWidth)
% Block scaling compression method as per Annex A in TS O-RAN.WG4.CUS

    % Find the maximum and minimum values within the real and imaginary
    % parts of the REs available per PRB
    maxV = max([real(uPRB); imag(uPRB)]);
    minV = min([real(uPRB); imag(uPRB)]);
    
    % Determine the maximum absolute value per PRB considering that the
    % negative value should be offset by 1 to fit the same number of IQ
    % bits
    maxValue = max(maxV, abs(minV)-1);

    % Map each sBlockScaler (one per PRB) to 8 bits
    sblockScaler = ceil(maxValue/2^(IQWidth-8));

    % Calculate the inverse of each sBlockScaler
    sblockScaler(sblockScaler==0) = ones(1,1,'like',sblockScaler);
    inverseBlockScaler = 2^7./sblockScaler;
    
    % Calculate the maximum output value per PRB, used for saturating
    % compressed samples to iqWidth
    qs = 2.^(cIQWidth-1);
    
    % Scale and round each RE in every PRB
    cRE = round(inverseBlockScaler .* uPRB ./ 2.^(IQWidth-cIQWidth)); 
    cPRB = complex(min(max(real(cRE),-qs),qs-1), min(max(imag(cRE),-qs),qs-1));
end

function [cPRB,compShift] = muLawCompress(uPRB,cIQWidth)
% Mu-law compression method as per Annex A in TS O-RAN.WG4.CUS

    % Extract the sign bits and absolute values from the REs available per PRB
    signI = sign(real(uPRB));
    signQ = sign(imag(uPRB));
    absI = abs(real(uPRB));
    absQ = abs(imag(uPRB));

    % Find the maximum and minimum values within the absolute values of the
    % REs available in each PRB
    maxVal = max([absI;absQ]);

    % Determine the shift to be applied to each PRB, compShift
    compShift = zeros(size(maxVal),'like',maxVal);
    compShiftTable = [2^8, 2^9, 2^10, 2^11, 2^12, 2^13, 2^14, inf;...
                         7,   6,    5,    4,    3,    2,    1,  0];
    for i = 1:length(maxVal)
        compShift(i) = compShiftTable(2,find(maxVal(i)<compShiftTable(1,:),1,'first'));
    end

    % Apply compression to all REs available in each PRB
    compI = muLawCompression(absI,compShift,cIQWidth,signI);
    compQ = muLawCompression(absQ,compShift,cIQWidth,signQ);
    cPRB = complex(compI,compQ);
end

function comp = muLawCompression(absValues,compShift,cIQWidth,signValues)
% Mu-law bit shift, compression and sign assignment

    % Apply round and shift left every RE in each PRB
    absValues = round(absValues.*2.^compShift);

    % Saturate absValues with values higher than 2^absBitWidth-1 
    absBitWidth = 15;  
    idx = absValues > (2^absBitWidth-1);
    absValues(idx) = 2^absBitWidth-1;

    % Compress absValues with values lower or equal to 2^(absBitWidth-2)
    cIQWidth = repmat(cIQWidth,size(absValues,1),1);
    comp = zeros(size(absValues));
    idx1 = absValues <= 2^(absBitWidth-2);
    comp(idx1) = absValues(idx1)./2.^(absBitWidth-cIQWidth(idx1));

    % Compress absValues with values lower or equal to 2^(absBitWidth-1)
    idx2 = absValues <= 2^(absBitWidth-1);
    idx2(idx1==1) = 0;
    comp(idx2) = absValues(idx2)./2.^(absBitWidth-cIQWidth(idx2)+1) + 2.^(cIQWidth(idx2)-3);

    % Compress the remaining absValues
    idx3 = ~(idx1==1 | idx2==1);
    comp(idx3) = absValues(idx3)./2.^(absBitWidth-cIQWidth(idx3)+2) + 2.^(cIQWidth(idx3)-2);

    % Insert previously extracted sign to every RE in each PRB
    comp = fix(signValues .* comp);
end

function [method,cIQWidth,IQWidth] = validateInputs(grid,methodIn,cIQWidthIn,varargin)
% Check inputs    

    fcnName = 'nrORANBlockCompress';

    % Validate type of grid and number of subcarriers in grid
    validateattributes(grid,{'double','single'},{'3d'},fcnName,'GRID');
    coder.internal.errorIf(any(grid ~= floor(grid), 'all'), ...
        'nr5g:nrORANBlockCompressDecompress:mustBeInteger','GRID');
    [K,L,P] = size(grid);
    coder.internal.errorIf(mod(K,12)~=0, ...
        'nr5g:nrORANBlockCompressDecompress:InvalidGridSubcarriers',K);

    % Validate compression method
    method = validatestring(methodIn,{'BFP' 'blockScaling' 'muLaw'},'',fcnName);

    % Validate bit width for compressed IQ samples and expand if scalar
    validateattributes(cIQWidthIn,{'numeric'},{'real','integer','3d',...
        'nonempty','>=',1,'<=',16},fcnName,'CIQWIDTH');
    if isscalar(cIQWidthIn)
        cIQWidth = repmat(double(cIQWidthIn(1)),[K/12 L P]);
    else
        validateattributes(cIQWidthIn,{'numeric'},{'size',[K/12,L,P]},...
            fcnName,'CIQWIDTH');
        cIQWidth = double(cIQWidthIn);
    end

    % Validate bit width for IQ samples before compression
    IQWidth = 16; % Default value assigned due to codegen
    if nargin == 4
        if strcmp(method,'blockScaling')
            validateattributes(varargin{1},{'numeric'},{'real','integer',...
                'scalar','nonempty','>=',1,'<=',32},fcnName,'IQWIDTH');
        elseif strcmp(method,'muLaw') && ~isempty(varargin{1})
            validateattributes(varargin{1},{'numeric'},{'scalar','integer'},fcnName,'IQWIDTH');
            coder.internal.errorIf(varargin{1}~=16, ...
            'nr5g:nrORANBlockCompressDecompress:InvalidIQWidthValue',varargin{1});
        end 
        IQWidth = double(varargin{1});
    else
        coder.internal.errorIf(strcmp(method,'blockScaling'), ...
        'nr5g:nrORANBlockCompressDecompress:MissingIQWidth');
    end
end
