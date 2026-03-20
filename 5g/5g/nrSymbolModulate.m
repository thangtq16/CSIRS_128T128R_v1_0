function out = nrSymbolModulate(in,modulation,varargin)
%nrSymbolModulate Symbol modulation
%   OUT = nrSymbolModulate(IN,MODULATION) maps the bits in codeword IN to
%   complex modulation symbols as per TS 38.211 section 5.1. The modulation
%   scheme, MODULATION must be one of 'pi/2-BPSK', 'BPSK', 'QPSK', '16QAM',
%   '64QAM', '256QAM', '1024QAM'. IN must be a column vector.
%
%   OUT = nrSymbolModulate(IN,MODULATION,NAME,VALUE) specifies an
%   additional option as a NAME,VALUE pair to allow control over the
%   datatype of the output symbols:
%
%   'OutputDataType'  -  Specified as 'double' or 'single'. OutputDataType
%                        determines the data type of the output modulated
%                        symbols and the data type used for intermediate
%                        computations. The default value is 'double'.
%
%   Example 1:
%   % Generate 16-QAM modulated symbols.
%
%   data = randi([0 1],40,1);
%   sym = nrSymbolModulate(data,'16QAM');
%
%   Example 2:
%   % Generate QPSK modulated symbols of single datatype.
%
%   data = randi([0 1],20,1,'int8');
%   sym = nrSymbolModulate(data,'QPSK','OutputDataType','single');
%
%   See also nrSymbolDemodulate, nrPRBS, nrLayerMap, nrPDCCH, nrPDSCH,
%   nrPBCH.

%   Copyright 2018-2023 The MathWorks, Inc.

%#codegen

    narginchk(2,4);

    % List of modulation schemes
    modlist = {'pi/2-BPSK','BPSK','QPSK','16QAM','64QAM','256QAM','1024QAM'};

    % Modulation scheme check
    fcnName = 'nrSymbolModulate';
    modscheme = validatestring(modulation,modlist,fcnName,'MODULATION');

    % PV pair check
    if nargin == 2
        outDataType = 'double';
    else
        pvstruct  = nr5g.internal.parseOptions( ...
            fcnName,{'OutputDataType'},varargin{:});
        outDataType = pvstruct.OutputDataType;
    end

    bpsList = [1 1 2 4 6 8 10];

    % Input codeword validation for datatype, size and value check
    validateattributes(in,{'double','int8','logical'},{'real','binary'}, ...
        fcnName,'IN');
    coder.internal.errorIf(~(iscolumn(in) || isempty(in)), ...
        'nr5g:nrSymbolModDemod:InvalidInputDim');

    ind = strcmpi(modlist,modscheme);
    tmp = bpsList(ind);
    bps = tmp(1);
    modOrder = 2^bps;

    % Input vector length check
    coder.internal.errorIf(mod(numel(in),bps) ~= 0, ...
        'nr5g:nrSymbolModDemod:InvalidInputLength',numel(in),bps);

    % Input codeword processing
    if isempty(in)
        out = cast(zeros(size(in),'like',in),outDataType);
        return;
    end

    intmp = cast(in,outDataType);

    % Generate symbol order vector
    symbolOrdVector = nr5g.internal.generateSymbolOrderVector(bps);

    % Modulate the bits
    symb = comm.internal.qam.modulate(intmp,modOrder,'custom',symbolOrdVector, ...
        1,1,[]);

    if bps == 1 % BPSK and pi/2-BPSK
        rotsymb  = symb.*exp(-1j*3*pi/4);
        if ind(1) % pi/2-BPSK
            out = rotsymb;
            out(2:2:end) = 1j*rotsymb(2:2:end);
        else % BPSK
            out = rotsymb;
        end
    else % QPSK, 16QAM, 64QAM, 256QAM, 1024QAM
        out = symb;
    end

end
