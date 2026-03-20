function waveform = OFDMModulate(carrier,prach,grid,internalinfo)
%OFDMModulate PRACH OFDM modulation
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.

%  Copyright 2022 The MathWorks, Inc.

%#codegen

    % Get the number of antennas in the resource grid
    P = size(grid,3);
    
    % If PRACH is active in this slot and there are some OFDM symbols in
    % this PRACH slot
    isActive = nr5g.internal.prach.isActive(prach);
    if (numel(internalinfo.CyclicPrefixLengths)>0)

        % Calculate PRACH slot sample indices corresponding to guard
        % periods and active OFDM symbols or cyclic prefixes
        [guardIndices,activeIndices] = ofdmIndices(internalinfo);

        % Create waveform including guard periods
        waveform = complex(zeros(max([guardIndices; activeIndices]),P));

        % Perform OFDM modulation if PRACH is active in this slot (includes
        % windowing without overlapping)
        if isActive
            overlapping = false;
            hasSampleRate = false;
            if mod(internalinfo.NSubcarriers,2)
                % There is an odd number of subcarriers in the grid. Append
                % an empty subcarrier to the grid so that the OFDM
                % modulator can proceed without issues. This addition means
                % that the active PRACH waveform is practically shifted in
                % frequency. This needs to be taken care of before OFDM
                % modulation, by adding the right phase for symbol phase
                % compensation, and after OFDM modulation, by performing a
                % frequency shift in time to make sure that the PRACH
                % waveform is correctly placed in frequency.
                grid = [grid; zeros(1,size(grid,2))];
            end

            % Perform OFDM modulation
            unguardedWaveform = nr5g.internal.OFDMModulate(grid,internalinfo,[],overlapping,hasSampleRate);
        else
            % PRACH not active, create an empty waveform
            unguardedWaveform = []; % for codegen
        end

    else
        
        % Create an empty waveform
        unguardedWaveform = []; % for codegen
        activeIndices = zeros(0,1);
        waveform = complex(zeros([0 P]));
        
    end
    
    % Add the offset period to the (possibly empty) waveform
    waveform = [zeros([internalinfo.OffsetLength P]); waveform];
    activeIndices = activeIndices + internalinfo.OffsetLength;
    
    % 'T' is the number of samples in the waveform
    T = size(waveform,1);
    
    % 'E' is the number of samples by which each OFDM symbol is cyclically
    % extended for the purposes of windowing and overlapping
    E = internalinfo.Windowing;

    % Initialize the cursors used to index the input and output waveforms
    % and the useful OFDM symbol period 'N_u'
    incursor = 0;
    outcursor = 0;
    N_u = internalinfo.Nfft;

    % For each OFDM symbol of an active PRACH
    N = isActive*numel(internalinfo.CyclicPrefixLengths);
    for n = 1:N

        % Get the cyclic prefix length 'N_CP' and total OFDM symbol
        % duration 'N_tot'
        N_CP = internalinfo.CyclicPrefixLengths(n);
        N_tot = N_CP + N_u;

        % Extract the current windowed OFDM symbol
        windowed = unguardedWaveform(incursor + (1:(N_tot + E)),1:P);
        incursor = incursor + N_tot + E;

        % Add the windowed OFDM symbol to the output waveform, including 
        % wrapping the window transition back to the start of the waveform
        % if necessary
        outidx = activeIndices(outcursor+1) + (0:(N_tot + E - 1));
        outidx = mod(outidx - 1,T) + 1;
        waveform(outidx,1:P) = waveform(outidx,1:P) + windowed;
        
        % Update the output cursor to point to the start of where the next
        % OFDM symbol should be added to the waveform
        outcursor = outcursor + N_tot;

    end
    
    % Perform a cyclic shift of -E samples. The first active sample of the
    % waveform (activeIndices(1)) is now the first sample of the first
    % cyclic prefix
    waveform = circshift(waveform,-E,1);
    
    % Perform frequency shift in time to place the PRACH waveform in the
    % right frequency
    t = repmat((0:size(waveform,1)-1).' / internalinfo.SampleRate,1,P);
    waveform = waveform .* exp(1i*2*pi*internalinfo.FrequencyShift*t);
    
    % Calculate carrier OFDM information
    carrierinfo = nr5g.internal.OFDMInfo(carrier,struct());
    
    % Adjust output level to give unit power per carrier RE
    % (for scaling factor Beta_PRACH = 1)
    gain = sqrt(internalinfo.Nfft / carrierinfo.Nfft) / sqrt(prach.LRA);
    waveform = waveform * gain;
    
end

% Calculate the sets of PRACH slot sample indices corresponding to guard
% periods ('gind') and active OFDM symbols or cyclic prefixes ('aind')
function [gind,aind] = ofdmIndices(info)
    
    a = info.CyclicPrefixLengths + info.Nfft;
    g = info.GuardLengths;
    starts = cumsum([1 a(1:end-1)+g(1:end-1)]) + a;
    gind = zeros(sum(g),1);
    ends = starts + g - 1;
    for i = 1:numel(g)
        gind(sum(g(1:i-1)) + (1:g(i))) = (starts(i):ends(i)).';
    end
    aind = (1:ends(end)).';
    aind(gind) = [];
    
end