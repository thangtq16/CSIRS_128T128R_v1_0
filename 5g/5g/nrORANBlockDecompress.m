function dGrid = nrORANBlockDecompress(cGrid,cParam,method,varargin)
%nrORANBlockDecompress O-RAN block decompression
%   DGRID = nrORANBlockDecompress(CGRID,CPARAM,METHOD,CIQWIDTH,IQWIDTH)
%   performs open radio access network (O-RAN) block decompression for
%   U-Plane IQ data, returning the decompressed grid DGRID. This function
%   implements the block floating point (BFP), block scaling, and mu-law
%   decompression methods, as defined in TS O-RAN.WG4.CUS Annex A.1, A.2,
%   and A.3, respectively.
%   
%   DGRID = nrORANBlockDecompress(CGRID,CPARAM,METHOD,CIQWIDTH) performs
%   only BFP or mu-law decompression.
%   
%   DGRID = nrORANBlockDecompress(CGRID,CPARAM,METHOD) performs only BFP
%   decompression.
%
%   CGRID is a K-by-L-by-P complex array, where K is the number of
%   subcarriers, L is the number of OFDM symbols, and P is the number of
%   antennas. K must be a multiple of 12, which corresponds to the number
%   of REs in a PRB. The IQ samples in the array are singles or doubles
%   with a bit width specified by CIQWIDTH.
% 
%   CPARAM is a K/12-by-L-by-P array. This output is a computational
%   parameter specific to the compression method: the common exponent
%   applied per compressed PRB in block floating point compression, the
%   common scale factor applied per compressed PRB in block scaling
%   compression, or the common shift applied per compressed PRB in mu-Law
%   compression.
%
%   METHOD is the block decompression method. The options are: 
%       - 'BFP' for block floating point decompression
%       - 'blockScaling' for block scaling decompression
%       - 'muLaw' for mu-law decompression
%   
%   CIQWIDTH is an integer (1...16) or a K/12-by-L-by-P integer array with
%   values in the range 1...16. The CIQWIDTH parameter specifies the
%   compressed IQ samples bit width (including the sign bit). If CIQWIDTH
%   is an integer, the same bit width is assumed for all IQ samples in
%   CGRID. If CIQWIDTH is an array, then each element of the array
%   specifies a bit width per PRB in CGRID. For BFP decompression, the
%   function ignores this input.
%
%   IQWIDTH is an integer (1...32) that specifies the IQ samples bit width
%   before compression. For mu-law decompression, this input must be 16.
%   For BFP decompression, the function ignores this input.
% 
%   DGRID is a K-by-L-by-P complex array. The IQ samples in the array can
%   be singles or doubles.
%
%   Example: 
%   % Generate the grid to compress, scale the IQ samples in the grid
%   % to the specified bit width, apply the specified compression and 
%   % decompression, and scale back the decompressed grid to compare the 
%   % IQ samples against the ones in the original grid.
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
%   % Apply decompression
%   dGrid = nrORANBlockDecompress(cGrid,cParam,method);
%
%   % Scale back the decompressed grid to compare the IQ samples against
%   % the ones in the original grid 
%   descaledGrid = dGrid * scaleFactor;
%
%   See also nrORANBlockCompress.

%   Copyright 2022-2025 The MathWorks, Inc.

%#codegen

    narginchk(3,5);

    % Validate input arguments
    [method,cIQWidth,IQWidth] = validateInputs(cGrid,cParam,method,varargin{:});
    
    % Reshape cGrid into a matrix where each column is a PRB and reshape
    % cParam and cIQWidth into vectors
    cPRBs = reshape(cGrid,12,[]);
    cParam = reshape(double(cParam),1,[]);
    cIQWidth = reshape(cIQWidth,1,[]);

    % Decompress each PRB (column) available in the compressed grid, cGrid
    switch method
        case 'BFP'
            dPRBs = BFPDecompress(cPRBs,cParam);
        case 'blockScaling' 
            dPRBs = blockScalingDecompress(cPRBs,cParam,cIQWidth,IQWidth);
        otherwise  % 'muLaw'
            decompI = muLawDecompress(real(cPRBs),cParam,cIQWidth);
            decompQ = muLawDecompress(imag(cPRBs),cParam,cIQWidth);
            dPRBs = complex(decompI,decompQ);     
    end

    % Reshape the decompressed grid back to the original size
    dGrid = reshape(dPRBs,size(cGrid)); 
end

function dPRB = BFPDecompress(cPRB,exponent)
% Block floating point decompression method as per Annex A in TS O-RAN.WG4.CUS
    
    % Determine the scale factor per PRB
    scaleFactor = 2.^exponent;
    
    % Scale every resource element (RE) within each PRB back to its
    % bit width before compression
    dPRB = scaleFactor .* cPRB;
end

function dPRB = blockScalingDecompress(cPRB,sblockScaler,cIQWidth,IQWidth)
% Block scaling decompression method as per Annex A in TS O-RAN.WG4.CUS
    
    % Scale every RE within each PRB back to its bit width before compression
    dPRB = round(sblockScaler .* cPRB ./ 2.^(cIQWidth-(IQWidth-8+1)));
end

function decomp = muLawDecompress(cValue,compShift,cIQWidth)
% Mu-law decompression method as per Annex A in TS O-RAN.WG4.CUS
    
    % Extract the sign bits and absolute values from the REs available per PRB
    signValues = sign(cValue);
    absValues = abs(cValue);
    
    % Saturate absValues with values higher than 2^(cIQWidth-1)-1
    cIQWidth = repmat(cIQWidth,size(absValues,1),1);
    idx = absValues > 2.^(cIQWidth-1)-1;
    absValues(idx) = 2.^(cIQWidth(idx)-1)-1; 

    % Decompress absValues with values lower or equal to 2^(cIQWidth-2)
    absBitWidth = 15;  
    decomp = zeros(size(cValue));
    idx1 = absValues <= 2.^(cIQWidth-2);
    decomp(idx1) = absValues(idx1).*2.^(absBitWidth-cIQWidth(idx1));

    % Decompress absValues with values lower or equal to (2^(cIQWidth-2) + 2^(cIQWidth-3))
    idx2 = absValues <= (2.^(cIQWidth-2) + 2.^(cIQWidth-3));
    idx2(idx1==1) = 0;
    decomp(idx2) = absValues(idx2).*2.^(absBitWidth-cIQWidth(idx2)+1)-2^13;

    % Decompress the remaining absValues
    idx3 = ~(idx1==1 | idx2==1);
    decomp(idx3) = absValues(idx3).*2.^(absBitWidth-cIQWidth(idx3)+2)-2^15;

    % Insert previously extracted sign to every RE in each PRB
    decomp = signValues .* decomp;
    decomp = decomp./2.^compShift;
end

function [method,cIQWidth,IQWidth] = validateInputs(cGrid,cParam,methodIn,varargin)
% Check inputs

    fcnName = 'nrORANBlockDecompress';

    % Validate grid input
    validateattributes(cGrid,{'double','single'},{'3d'},fcnName,'CGRID');
    coder.internal.errorIf(any(cGrid ~= floor(cGrid), 'all'), ...
        'nr5g:nrORANBlockCompressDecompress:mustBeInteger','CGRID');
    [K,L,P] = size(cGrid);
    coder.internal.errorIf(mod(K,12)~=0, ...
        'nr5g:nrORANBlockCompressDecompress:InvalidCGridSubcarriers',K);

    % Validate compression parameter array
    validateattributes(cParam,{'numeric'},{'real','integer','3d','nonempty',...
        'size',[K/12,L,P]},fcnName,'CPARAM');     

    % Validate compression method
    method = validatestring(methodIn,{'BFP' 'blockScaling' 'muLaw'},'',fcnName);

    % Validate bit width for compressed IQ samples and expand if scalar
    cIQWidth = zeros(1,0); % Assignment needed due to codegen
    if nargin > 3 && (strcmp(method,'blockScaling') || strcmp(method,'muLaw'))
        validateattributes(varargin{1},{'numeric'},{'real','integer','3d',...
            'nonempty','>=',1,'<=',16},fcnName,'CIQWIDTH');
        if isscalar(varargin{1})
            cIQWidth = double(repmat(varargin{1}(1),[K/12 L P]));
        else
            validateattributes(varargin{1},{'numeric'},{'size',[K/12,L,P]},...
                fcnName,'CIQWIDTH');
            cIQWidth = double(varargin{1});
        end
    else
        coder.internal.errorIf(strcmp(method,'blockScaling') || ...
            strcmp(method,'muLaw'), ...
            'nr5g:nrORANBlockCompressDecompress:MissingCIQWidth');
    end
    
    % Validate bit width for IQ samples before compression
    IQWidth = 16; % Default value assigned due to codegen
    if nargin == 5
        if strcmp(method,'blockScaling')
            validateattributes(varargin{2},{'numeric'},{'real','integer',...
                'scalar','nonempty','>=',1,'<=',32},fcnName,'IQWIDTH');
        elseif strcmp(method,'muLaw') && ~isempty(varargin{2})
            validateattributes(varargin{2},{'numeric'},{'scalar','integer'},fcnName,'IQWIDTH');
            coder.internal.errorIf(varargin{2}~=16, ...
            'nr5g:nrORANBlockCompressDecompress:InvalidIQWidthValue',...
            varargin{2});
        end 
        IQWidth = double(varargin{2});
    else
        coder.internal.errorIf(strcmp(method,'blockScaling'), ...
        'nr5g:nrORANBlockCompressDecompress:MissingIQWidth');
    end
end
