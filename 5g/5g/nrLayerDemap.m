function out = nrLayerDemap(in)
%nrLayerDemap Layer demapping onto scrambled and modulated codewords
%   OUT = nrLayerDemap(IN) performs layer demapping of received layered
%   symbols IN. IN is a matrix of size M-by-NLAYERS, where M represents the
%   number of modulation symbols per transmission layer and NLAYERS
%   represents the number of transmission layers. Using NLAYERS the number
%   of codewords is determined as per TS 38.211 table 7.3.1.3-1. Based on
%   the number of codewords, the function returns OUT which is a cell array
%   of 1 or 2 column vectors, one for each codeword.
%
%   Example:
%   % Map a single codeword onto 4 layers and then recover it using
%   % layer demapper.
%
%   codeword = ones(20,1); % Input codeword
%   nlayers = 4; % Number of transmission layers
%
%   % Perform layer mapping followed by layer demapping and then check for
%   % equality
%   layeredOut = nrLayerMap(codeword,nlayers);
%   out = nrLayerDemap(layeredOut);
%   isequal(codeword,out{1})
%
%   See also nrLayerMap, nrSymbolDemodulate, nrPDSCHDecode.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    % Validate the number of input arguments
    narginchk(1,1);

    % Output empty if the input is empty
    if (isempty(in) && isnumeric(in))
        out = {};
        return;
    end

    % Validate received layered symbols
    fcnName = 'nrLayerDemap';
    validateattributes(in,{'numeric'},{'2d','finite'},fcnName,'IN');

    % Get the number of transmission layers
    nlayers = size(in,2);

    % Validate the number of transmission layers (1...8)
    coder.internal.errorIf(nlayers < 1 || nlayers > 8,'nr5g:nrLayerDemap:InvalidNumOfLayers',nlayers);

    % Get the number of codewords as per TS 38.211 table 7.3.1.3-1
    ncw = ceil(nlayers/4);

    % Initialize output
    out = coder.nullcopy(cell(1,ncw));

    % Perform layer demapping
    if ncw == 1
        temp = in.';
        out{1} = temp(:);
    else
        nbrOfLayersCW1 = floor(nlayers/2);
        temp1 = in(:,1:nbrOfLayersCW1).';
        out{1} = temp1(:);
        temp2 = in(:,nbrOfLayersCW1 + 1:end).';
        out{2} = temp2(:);
    end

end
