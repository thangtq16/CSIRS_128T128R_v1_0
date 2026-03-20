function out = nrTransformPrecode(in,Mrb)
%nrTransformPrecode Transform precoding
%   OUT = nrTransformPrecode(IN,MRB) returns transform precoded symbols OUT
%   given the modulation symbols IN, as defined in TS 38.211 Section
%   6.3.1.4/6.3.2.6.4. MRB represents the number of resource blocks
%   associated with the PUSCH or PUCCH format 3 or format 4 transmission.
%   It defines the length of sub-blocks of IN that are transform precoded
%   separately. In the NR uplink, transform precoding is used in
%   conjunction with CP-OFDM modulation, for single layer transmissions, to
%   create a SC-FDMA or DFT-s-OFDM waveform.
%
%   If IN is a column vector then the function returns the transform
%   precoded symbols OUT as a column vector. If IN is a matrix then each
%   column is processed separately and the function returns OUT as a
%   matrix. The number of rows of IN must be an integer multiple of MRB*12
%   (the allocated number of subcarriers). And both IN and OUT are of same
%   size. Typically this function will be used with a single column input
%   representing a single transmission layer. Nominally the value of MRB
%   will be (2^alpha2)*(3^alpha3)*(5^alpha5), where {alpha2, alpha3,
%   alpha5} is a set of non-negative integers.
%
%   Note that in the NR uplink, transform precoding is used in the
%   following cases,
%   - Prior to MIMO precoding of PUSCH with single transmission layer
%   - After symbol modulation of PUCCH format 3
%   - After block-wise spreading of PUCCH format 4
%
%   Example:
%   % Generate an input codeword of length 960 and perform scrambling,
%   % symbol modulation with the modulation scheme as 16QAM, layer mapping
%   % with single transmission layer and then transform precoding with an
%   % allocated bandwidth of 2 RBs.
%
%   cw = randi([0 1],960,1);
%   ncellid = 42;
%   rnti = 101;
%   scrambled = nrPUSCHScramble(cw,ncellid,rnti);
%   modulation = '16QAM';
%   modSym = nrSymbolModulate(scrambled,modulation);
%   layeredSym = nrLayerMap(modSym,1);
%
%   tpSym = nrTransformPrecode(layeredSym,2);
%
%   See also nrTransformDeprecode, nrLayerMap, nrPUSCHCodebook.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    narginchk(2,2);
    fcnName = 'nrTransformPrecode';

    if isnumeric(in) && isempty(in)
        % Empty in - Empty out
        out = in;
        return;
    else
        % Validate input data
        validateattributes(in,{'double','single'},{'2d'},fcnName,'IN');

        % Validate Mrb
        validateattributes(Mrb,{'numeric'},{'nonempty','scalar',...
            'finite','real','positive','integer'},fcnName,'MRB');
    end

    % Get the number of subcarriers
    Msc = double(Mrb)*12;

    % Get the number of OFDM symbols
    nSymbols = size(in,1)/Msc;
    if nSymbols ~= fix(nSymbols)
        coder.internal.error('nr5g:nrTransformPrecodeDeprecode:InvalidNumOfModSymbols',size(in,1),Msc);
    end

    % Perform transform precoding
    xtilde0 = reshape(in,Msc,numel(in)/Msc);
    out = reshape(fft(xtilde0)/sqrt(Msc),size(in));

end
