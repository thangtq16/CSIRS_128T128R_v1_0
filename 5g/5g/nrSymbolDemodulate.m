function out = nrSymbolDemodulate(in,modulation,varargin)
%nrSymbolDemodulate Demodulation and symbol to bit conversion
%   OUT = nrSymbolDemodulate(IN,MODULATION) demodulates the complex symbols
%   IN using soft decision. The modulation scheme, MODULATION must be one
%   of 'pi/2-BPSK', 'BPSK', 'QPSK', '16QAM', '64QAM', '256QAM', '1024QAM'.
%   IN must be a column vector.
%
%   OUT = nrSymbolDemodulate(IN,MODULATION,NVAR) additionally specifies
%   the noise variance scaling factor for the soft bits. NVAR must be a
%   real scalar nonnegative value. When not specified, it defaults to 1e-10
%   corresponding to an SNR of 100dB (assuming unit signal power).
%
%   OUT = nrSymbolDemodulate(IN,MODULATION,...,NAME,VALUE) specifies an
%   additional option as a NAME,VALUE pair to allow control over the type
%   of the output symbols:
%
%   'DecisionType'   -   Specified as 'hard' or 'soft'. If 'hard', output
%                        consists of groups of bits. If 'soft', output
%                        consists of bit-wise approximate log-likelihood
%                        ratios. Each demodulated symbol is mapped to a
%                        group of log2(modulation order) bits, with first
%                        bit representing the MSB and the last bit
%                        representing the LSB. The default value is 'soft'.
%
%   Demodulation is performed according to the constellations given in
%   TS 38.211 section 5.1 including the power normalization factors
%   specified. The factors: 1/sqrt(2) for BPSK, pi/2-BPSK and QPSK,
%   1/sqrt(10) for 16QAM, 1/sqrt(42) for 64QAM, 1/sqrt(170) for 256QAM and
%   1/sqrt(682) for 1024QAM. Bits per symbol: 2 for QPSK, 4 for 16QAM,
%   6 for 64QAM, 8 for 256QAM and 10 for 1024QAM.
%
%   Example 1:
%   % Demonstrate QPSK demodulation in soft decision mode for a noise
%   % variance of 0.1.
%
%   data = randi([0 1],40,1);
%   modsymb = nrSymbolModulate(data,'QPSK');
%   nVar = 0.1;
%   recsymb = awgn(modsymb,1/nVar,1,'linear');
%   out = nrSymbolDemodulate(recsymb,'QPSK',0.1);
%
%   Example 2:
%   % Demonstrate 16-QAM hard demodulation at an SNR of 15dB.
%
%   data = randi([0 1],100,1,'int8');
%   modsymb = nrSymbolModulate(data,'16QAM');
%   recsymb = awgn(modsymb,15);
%   demodbits = nrSymbolDemodulate(recsymb,'16QAM','DecisionType', ...
%      'hard');
%   numErr = biterr(data,demodbits)
%
%   See also nrSymbolModulate, nrLayerDemap, nrPRBS, nrPDCCHDecode,
%   nrPDSCHDecode, nrPBCHDecode.

%   Copyright 2018-2024 The MathWorks, Inc.

%#codegen

    narginchk(2,5)

    % List of modulation schemes
    modlist = {'pi/2-BPSK', 'BPSK', 'QPSK', '16QAM', '64QAM', '256QAM', '1024QAM'};

    % Modulation scheme check
    fcnName = 'nrSymbolDemodulate';
    modscheme = validatestring(modulation,modlist,fcnName,'MODULATION');

    nInArgs = nargin;
    if nInArgs == 2
        % nrSymbolDemodulate(cw,mod)
        decType = 'soft';
        nVar = 1e-10;
    elseif nInArgs == 3
        % nrSymbolDemodulate(cw,mod,nVar)
        decType = 'soft';
        nVar = varargin{1};
        validateattributes(nVar,{'double','single'},{'scalar','real',...
            'nonnegative','nonnan','finite'},fcnName,'NVAR');
    elseif nInArgs == 4
        % nrSymbolDemodulate(cw,mod,P1,V1)
        pvstruct = nr5g.internal.parseOptions( ...
            fcnName,{'DecisionType'},varargin{:});
        decType = pvstruct.DecisionType;
        nVar = 1e-10;
    else
        % nrSymbolDemodulate(cw,mod,nVar,P1,V1)
        if isnumeric(varargin{1})
            nVar = varargin{1};
            pvstruct = nr5g.internal.parseOptions(...
                fcnName,{'DecisionType'},varargin{2:end});
            decType = pvstruct.DecisionType;
        else
            % nrSymbolDemodulate(cw,mod,P1,V1,nVar)
            pvstruct = nr5g.internal.parseOptions(...
                fcnName,{'DecisionType'},varargin{1:2});
            nVar = varargin{3};
            decType = pvstruct.DecisionType;
        end
        validateattributes(nVar,{'double','single'},{'scalar','real',...
            'nonnegative','nonnan','finite'},fcnName,'NVAR');
    end

    % Clip nVar to allowable value to avoid +/-Inf outputs
    if nVar < 1e-10
        nVar = cast(1e-10,'like',nVar);
    end

    bpsList = [1 1 2 4 6 8 10];

    % Received codeword validation for data type, size and value check
    validateattributes(in,{'double','single'},{'finite','nonnan'}, ...
        fcnName,'IN');
    coder.internal.errorIf(~(iscolumn(in) || isempty(in)), ...
        'nr5g:nrSymbolModDemod:InvalidInputDim');

    ind = strcmpi(modlist,modscheme);
    tmp = bpsList(ind);
    bps = tmp(1);
    modOrder = 2^bps;

    % Received codeword processing
    if strcmpi(decType,'soft')
        outDType = underlyingType(in);
    else
        outDType = 'int8';
    end
    if isempty(in)
        % Removing the complexity of the input is required to support
        % codegen. Without this, the output of the function will be
        % real for MATLAB and complex for codegen.
        out = cast(zeros(size(in),'like',real(in)),outDType);
        return;
    end

    if strcmpi(decType,'Hard')
        outType = 'bit';
    else
        outType = 'approxLLR';
    end

    % Generate symbol order vector
    symbolOrdVector = nr5g.internal.generateSymbolOrderVector(bps);

    if bps >= 2 % QPSK,16QAM,64QAM,256QAM or 1024QAM
        outTmp = comm.internal.qam.demodulate(in,modOrder,'custom', ...
            symbolOrdVector,1,outType,nVar,false);
    else % BPSK or pi/2-BPSK
        if ind(1) % pi/2-BPSK
            inoddrot = complex(in);
            inoddrot(2:2:end) = -1j*in(2:2:end);
            derotsymb = inoddrot*exp(1j*3*pi/4);
        else % BPSK
            derotsymb = in*exp(1j*3*pi/4);
        end
        outTmp = comm.internal.qam.demodulate(derotsymb,modOrder,'binary',...
            [0 1],1,outType,nVar,true);
    end

    if strcmpi(decType,'soft')
        out = outTmp;
    else % Hard decision
        out = cast(outTmp,outDType);
    end

end
