function waveform = OFDMModulate(grid,info,nSlot,overlapping,hasSampleRate)
%OFDMModulate OFDM modulation
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   WAVEFORM = OFDMModulate(GRID,INFO,NSLOT,OVERLAPPING,HASSAMPLERATE)
%   performs OFDM modulation of the resource grid GRID given OFDM related
%   information INFO. NSLOT is used to select the correct cyclic prefix
%   lengths and symbol phases in INFO to use during OFDM modulation.
%   OVERLAPPING (true,false) specifies whether or not overlapping of OFDM
%   symbols should be performed when applying windowing. HASSAMPLERATE
%   (true,false) specifies whether or not a user-specified sample is
%   present.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    N = size(grid,2);
    [cpLengths,symbolPhases] = nr5g.internal.OFDMInfoRelativeNSlot(info,nSlot,N);
    
    % Phase precompensation, TS 38.211 Section 5.4
    gridc = grid .* exp(1i * symbolPhases(1:N));
    
    % OFDM modulation, TS 38.211 Section 5.3.1
    K = size(gridc,1);
    firstSC = (info.Nfft/2) - (K/2) + 1;
    nullIndices = [1:(firstSC-1) (firstSC+K):info.Nfft].';
    unwindowed = ofdmmod(gridc,info.Nfft,cpLengths(1:N),nullIndices);
    
    % Windowing (and optional overlapping)
    if ~isempty(info.Windowing)
        windowed = windowing(unwindowed,info,N,cpLengths,overlapping);
    else
        windowed = unwindowed;
    end
    
    % Resampling (if required)
    if (hasSampleRate || isempty(coder.target))
        r = info.Resampling;
        if (any([r.L r.M]~=1))
            h = designMultirateFIR(r.L,r.M,r.TW,r.AStop);
            waveform = resample(windowed,r.L,r.M,h);
        else
            waveform = windowed;
        end
    else
        % Inputs to 'designMultirateFIR' must be compile-time constants for
        % code generation, but 'r.M', 'r.L' and 'r.TW' cannot be computed
        % as constants if the function syntax is using nrCarrierConfig.
        % This 'else' should not be reached when performing code
        % generation, the function syntax is using nrCarrierConfig and
        % resampling is required, because an outer function should have
        % issued an error for this case
        waveform = windowed;
    end

end

function waveform = windowing(unwindowed,info,N,cpLengths,overlapping)

    % 'E' is the number of samples by which each OFDM symbol is cyclically
    % extended for the purposes of windowing and overlapping
    E = info.Windowing;
    
    % Calculate output waveform duration 'T' and number of antennas 'P'
    % and create empty waveform
    T = size(unwindowed,1);
    if (~overlapping)
        T = T + (E*N);
    end
    P = size(unwindowed,2);
    waveform = complex(zeros([T P],'like',unwindowed));
    
    % Initialize the cursors used to index the input and output waveforms
    % and the useful OFDM symbol period 'N_u'
    incursor = 0;
    outcursor = 0;
    N_u = info.Nfft;
    
    % For each OFDM symbol
    for n = 1:N
    
        % Get the cyclic prefix length 'N_CP' and total OFDM symbol
        % duration 'N_tot'
        N_CP = cpLengths(n);
        N_tot = N_CP + N_u;
    
        % Extract the current unwindowed OFDM symbol
        symbol = unwindowed(incursor + (1:N_tot),1:P);
        incursor = incursor + N_tot;
    
        % Create the window function (duration N_tot+E), cyclically extend
        % the OFDM symbol by a further 'E' samples at the start, and then
        % apply the window function (pointwise multiply)
        window = raised_cosine_window(N_tot,E);
        extended = [symbol(N_u-E + (1:E),1:P); symbol];
        windowed = extended .* window;
    
        % If overlapping is enabled and this is the last OFDM symbol
        if (overlapping && n==N)
    
            % Add the first 'N_tot' samples of the windowed OFDM symbol to
            % the output waveform
            outidx = outcursor + (1:N_tot);
            waveform(outidx,1:P) = waveform(outidx,1:P) + windowed(1:N_tot,1:P);
    
            % Add the last 'E' samples of the windowed OFDM symbol to
            % the first 'E' samples of the waveform
            waveform(1:E,1:P) = waveform(1:E,1:P) + windowed(end-E+1:end,1:P);
    
            % Perform a cyclic shift of -E samples to move the window
            % transition at the start of the waveform to the end of the
            % waveform. The first sample of the waveform is now the first
            % sample of the first cyclic prefix
            waveform = circshift(waveform,-E,1);
    
        else
    
            % Add the windowed OFDM symbol to the output waveform
            outidx = outcursor + (1:(N_tot + E));
            waveform(outidx,1:P) = waveform(outidx,1:P) + windowed;
    
        end
    
        % Update the output cursor to point to the start of where the next
        % OFDM symbol should be added to the waveform, taking into
        % consideration whether or not overlapping is enabled
        outcursor = outcursor + N_tot;
        if (~overlapping)
            outcursor = outcursor + E;
        end

    end

end

% Raised Cosine window creation; creates a window function of length n+e
% with raised cosine transitions on the first and last 'e' samples.
function p = raised_cosine_window(n,e)

    p = 0.5*(1-sin(pi*(e+1-2*(1:e).')/(2*e)));
    p = [p; ones(n-e,1); flipud(p)];

end
