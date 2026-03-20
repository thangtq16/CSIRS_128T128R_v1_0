function out = nrTransformDeprecode(in,Mrb)
%nrTransformDeprecode Transform deprecoding
%   OUT = nrTransformDeprecode(IN,MRB) returns transform deprecoded symbols
%   OUT given the modulation symbols IN. MRB represents the number of
%   resource blocks associated with the PUSCH or PUCCH format 3 or format 4
%   transmission. It defines the length of sub-blocks of IN that are
%   transform deprecoded separately. In the NR uplink, transform
%   deprecoding is used in conjunction with CP-OFDM demodulation, for
%   single layer transmissions, to demodulate a SC-FDMA or DFT-s-OFDM
%   waveform.
%
%   If IN is a column vector then the function returns the transform
%   deprecoded symbols OUT as a column vector. If IN is a matrix then each
%   column is processed separately and the function returns OUT as a
%   matrix. The number of rows of IN must be an integer multiple of MRB*12
%   (the allocated number of subcarriers). And both IN and OUT are of same
%   size. Typically this function will be used with a single column input
%   representing a single transmission layer. Nominally the value of MRB
%   will be (2^alpha2)*(3^alpha3)*(5^alpha5), where {alpha2, alpha3,
%   alpha5} is a set of non-negative integers.
%
%   Note that in the NR uplink, transform deprecoding is used in the
%   following cases,
%   - After MIMO deprecoding of PUSCH with single transmission layer
%   - Prior to symbol demodulation of PUCCH format 3
%   - Prior to block-wise despreading of PUCCH format 4
%
%   Example:
%   % Generate an input codeword of length 960 and perform scrambling,
%   % symbol modulation with the modulation scheme as 16QAM, layer mapping
%   % with single transmission layer and then transform precoding and
%   % deprecoding with an allocated bandwidth of 2 RBs. 
%
%   cw = randi([0 1],960,1);
%   ncellid = 42;
%   rnti = 101;
%   scrambled = nrPUSCHScramble(cw,ncellid,rnti);
%   modulation = '16QAM';
%   modSym = nrSymbolModulate(scrambled,modulation);
%   layeredSym = nrLayerMap(modSym,1);
%   tpSym = nrTransformPrecode(layeredSym,2);
%
%   tdpSym = nrTransformDeprecode(tpSym,2);
%
%   See also nrTransformPrecode, nrPUSCHCodebook, nrLayerDemap.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    narginchk(2,2);
    fcnName = 'nrTransformDeprecode';

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

    % Perform transform deprecoding
    ytilde0 = reshape(in,Msc,numel(in)/Msc);
    out = reshape(ifft(ytilde0)*sqrt(Msc),size(in));

end
