function [waveform, info] = nrOFDMModulateCallForCodegen(c, cgrid, scs, cp, windowingProp, sr, carrierFreq)
    %nrOFDMModulateCallForCodegen Use Nfft in OFDMModulate calls for code generation as SampleRate name-value argument with carrier config is not supported.
    %
    %   Note: This is an internal undocumented function and its API and/or
    %   functionality may change in subsequent releases.
    
    %   Copyright 2020-2023 The MathWorks, Inc.
    
    %#codegen
        
    if coder.target('MATLAB')
        % Calculate windowing samples from windowing Pct (%) and FFT size
        % used by nrOFDMModulate for the specified SampleRate
        windowingSamples = windowingPerc2Samples(windowingProp, scs, cp, c.NSizeGrid, sr);
        
        % Call nrOFDMModulate with desired SampleRate
        [waveform, info] = nrOFDMModulate(c, cgrid, 'Windowing', windowingSamples, 'SampleRate', sr, 'CarrierFrequency', carrierFreq);
    else
        % Calculate FFT size from SampleRate and SCS, and ensure that FFT
        % size is integer
        nfft = round(sr/(scs*1e3));
        fftsr = scs*1e3*nfft;
        coder.internal.errorIf(~isequal(fftsr,sr),'nr5g:nrWaveformGenerator:SampleRateCodegen',sr,scs);
        
        % Calculate windowing samples from windowing Pct (%) and the
        % calculated FFT size
        windowingSamples = windowingPerc2Samples(windowingProp, scs, cp, nfft);
        
        % Call nrOFDMModulate with the calculated FFT size
        [waveform, info] = nrOFDMModulate(c, cgrid, 'Windowing', windowingSamples, 'Nfft', nfft, 'CarrierFrequency', carrierFreq);        
    end
end

% Convert WindowingPercent to number of samples
function nsamp = windowingPerc2Samples(windowingProp, scs, cp, varargin)
    % This function accepts 1 or 2 optional inputs. If there is only one
    % optional input, this is NFFT. Otherwise, the two inputs are NRB and
    % SampleRate, respectively.
    if numel(windowingProp) <= 1
        windowingPct = windowingProp;
    else
        % nrDLCarrierConfig has up to 8 elements in this case
        % [15  30   60normal   60extended   120   240   480   960]
        % nrULCarrierConfig has up to 7 elements in this case (doesn't include the 240 kHz SSB case)
        % [15  30   60normal   60extended   120   480   960]
        muID = 1 + log2(scs/15) + ... 
            1*(scs>=120) + ...  % Add indexing offset to skip past the 60 kHz/extended CP entry
            1*(scs==60 && strcmpi(cp, 'extended'));  % Add indexing offset in this case
        % Support backward compatibility with shorter vectors (pre FR2-2) by 
        % using largest SCS WP value to cover the 480, 960 cases, if they were not provided
        windowingPct = windowingProp(min(muID,length(windowingProp)));
    end
    if nargin > 4
        nrb = varargin{1};
        sr = varargin{2};
        r = nrOFDMInfo(nrb, scs, 'SampleRate', sr);
        nfft = r.Nfft;
    else
        nfft = varargin{1};
    end
    nsamp = floor(windowingPct*nfft/100);
end
