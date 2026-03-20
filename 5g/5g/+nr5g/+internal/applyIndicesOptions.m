function out = applyIndicesOptions(siz,opts,varargin)
%applyIndicesOptions apply indices options (input is 0-based subscripts or indices)
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   OUT = applyIndicesOptions(SIZ,OPTS,...) applies indices options OPTS 
%   given resource array size SIZ and further inputs which specify the
%   indices or subscripts to which the options should be applied.
%
%   SIZ is a three-element vector [K L P] specifying the number of
%   subcarriers, OFDM symbols and antennas in the resource array.
%
%   OPTS is a structure containing the following fields:
%   'IndexStyle'       - 'index' for linear indices
%                        'subscript' for [subcarrier, symbol, antenna] 
%                         subscript row form
%   'IndexBase'        - '1based' for 1-based indices
%                        '0based' for 0-based indices
%   'MultiColumnIndex' - true for multi-column indices
%                        false for single-column indices
%
%   OUT = applyIndicesOptions(SIZ,OPTS,IND) applies indices options to
%   0-based indices vector IND of size NRE-by-1. 
%
%   OUT = applyIndicesOptions(SIZE,OPTS,K,L,P) applies indices options to
%   NRE-by-1 vectors of 0-based frequency subscripts K, OFDM symbol
%   subscripts L and antenna subscripts P. P is optional, defaulting to
%   subscript 0. If L or P are scalar they are expanded to the same size as
%   K.

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen
 
    narginchk(3,5);
    
    if (nargin==3)
        ind = double(varargin{1});
        inputIndexStyle = "index";
    else % nargin == 4 or 5
        k = double(varargin{1});
        l = reprows(double(varargin{2}),k);
        if (nargin==5)
            p = reprows(double(varargin{3}),k);
        else
            p = zeros(size(k));
        end
        inputIndexStyle = "subscript";
    end

    coder.varsize('outTmp',[Inf Inf],[1 1]);

    K = siz(1);
    L = siz(2);
    if (numel(siz)==2)    
        P = 1;
    else
        P = siz(3);
    end
    
    if (strcmpi(opts.IndexStyle,"index"))
        
        if (strcmpi(inputIndexStyle,"subscript"))
        
            outTmp = sub2ind([K L P],k(:)+1,l(:)+1,p(:)+1);
           
            if (strcmpi(opts.IndexBase,"0based"))
                outTmp = outTmp - 1;
            end
        
        else
            
            outTmp = ind;
            
            if (strcmpi(opts.IndexBase,"1based"))
                outTmp = outTmp + 1;
            end
            
        end

        if isfield(opts,'MultiColumnIndex') && opts.MultiColumnIndex
            % Codegen limitation to expand empty arrays
            if ~isempty(outTmp) 
                outD = reshape(outTmp,[],P);
            else
                outD = zeros(0,P);
            end
        else
            outD = outTmp;
        end
    
    else % IndexStyle = "subscript"
    
        if (strcmpi(inputIndexStyle,"subscript"))
            
            outTmp = [k(:) l(:) p(:)];
           
            if (strcmpi(opts.IndexBase,"1based"))
                outTmp = outTmp + 1;
            end
        
        else
            
            [k,l,p] = ind2sub([K L P],ind+1);
            outTmp = [k l p];
            
            if (strcmpi(opts.IndexBase,"0based"))
                outTmp = outTmp - 1;
            end
            
        end
        
        % Codegen limitation to expand empty arrays
        if isempty(outTmp)
            outD = zeros(0,3);
        else
            outD = outTmp;
        end
        
    end
    
    out = cast(outD,'uint32');

end

function out = reprows(in,ref)
    
    if (size(in,1)~=size(ref,1))
        out = repmat(in,size(ref,1),1);
    else
        out = in;
    end
    
end
