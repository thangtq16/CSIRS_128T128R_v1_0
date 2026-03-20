function out = nrLayerMap(in,nlayers)
%nrLayerMap Layer mapping of modulated and scrambled codewords
%   OUT = nrLayerMap(IN,NLAYERS) performs layer mapping of input codeword
%   or codewords IN according to TS 38.211 Section 6.3.1.3/7.3.1.3. OUT is
%   an M-by-NLAYERS matrix, where M represents the number of modulation
%   symbols per transmission layer and NLAYERS is the number of
%   transmission layers. These transmission layers are formed by
%   multiplexing the modulation symbols from either one or two codewords.
%   IN can be either a numeric column vector or cell array of 1 or 2
%   numeric column vectors. NLAYERS must be a scalar integer from 1 to 8.
%
%   Note that the overall operation of the layer mapper is the transpose of
%   that defined in the specification i.e. the symbols of each layer form
%   the columns rather than rows.
%
%   Example 1:
%   % Map one codeword onto four layers.
%
%   out = nrLayerMap(ones(40,1),4);
%   sizeOut = size(out)
%
%   Example 2:
%   % Map two codewords onto five layers.
%
%   out = nrLayerMap({ones(20,1),ones(30,1)},5);
%   sizeOut = size(out)
%
%   See also nrLayerDemap, nrSymbolModulate, nrPDSCH.

%   Copyright 2018-2020 The MathWorks, Inc.

%#codegen

    narginchk(2,2);

    % Validate whether the input is either numeric/cell
    fcnName = 'nrLayerMap';
    validateattributes(in,{'numeric','cell'},{},fcnName,'IN');

    % Establish if the input codeword is in a cell array, and if not, place
    % it in a cell array for uniform processing
    if ~iscell(in)
        cws = {in};
    else
        cws = in;
    end

    % Get the number of codewords
    ncw = numel(cws);

    % Validate the number of codewords
    coder.internal.errorIf(~(ncw == 1 || ncw == 2),'nr5g:nrLayerMap:InvalidNumOfCWs',ncw);

    % Check whether the input is empty
    if ncw == 1
        emptyInp = isempty(cws{1});
    else
        emptyInp = (isempty(cws{1}) && isempty(cws{2}));
    end
    if emptyInp
        % Validate the number of transmission layers (1...8)
        validateattributes(nlayers,{'numeric'},{'nonempty','real','scalar',...
            'finite','integer','>=',1,'<=',8},fcnName,'NLAYERS');

        % Output empty
        out = zeros(0,nlayers,'like',cws{1});
        return;
    end

    % Validate input codewords
    for cwIdx = 1:ncw
        validateattributes(cws{cwIdx},{'numeric'},{'column','finite'},...
            fcnName,'IN');
    end

    % Validate the number of transmission layers
    validateattributes(nlayers,{'numeric'},{'nonempty','real','scalar',...
        'finite','integer','>=',1,'<=',8},fcnName,'NLAYERS');

    % Validate the combination of number of transmission layers and number
    % of codewords as per TS 38.211 table 7.3.1.3
    coder.internal.errorIf((ncw == 1 && (nlayers >= 5)) || (ncw == 2 && (nlayers < 5)),...
        'nr5g:nrLayerMap:InvalidCWsAndLayersComb',nlayers,(nlayers>4)+1,ncw);

    % Split the number of layers for each codeword
    nLayers = floor((double(nlayers) + (0:ncw-1))/ncw);

    % Validate the length of each codeword
    for cwIdx = 1:ncw
        coder.internal.errorIf(rem(length(cws{cwIdx}),nLayers(cwIdx)) ~= 0,...
            'nr5g:nrLayerMap:InvalidCWLen',cwIdx,nLayers(cwIdx),length(cws{cwIdx}));
    end

    % Validate the ratio of 1st codeword length to 2nd codeword length
    if ncw == 2 && (length(cws{1}) ~= (nLayers(1)/nLayers(2))*length(cws{2}))
        if rem(sum(nLayers),2) == 0
            coder.internal.error('nr5g:nrLayerMap:UnequalCWsLen',sum(nLayers));
        else
            coder.internal.error('nr5g:nrLayerMap:InvalidOfCWsLenRatio',...
                sum(nLayers),nLayers(1),nLayers(2));
        end
    end

    % Get the number of modulation symbols per layer
    mSymbLayer = uint32(length(cws{1})/nLayers(1));

    % Initialize output
    out = zeros(mSymbLayer,sum(nLayers),'like',(cws{1}));

    % Perform layer mapping
    for idx = 1:nLayers(1)
        out(:,idx) = cws{1}(idx:nLayers(1):end);
    end
    if ncw == 2
        for idx2 = 1:nLayers(2)
            out(:,nLayers(1) + idx2) = cws{2}(idx2:nLayers(2):end);
        end
    end

end
