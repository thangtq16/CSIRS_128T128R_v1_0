function H = OFDMChannelResponse(ofdminfo,pathGains,pathFilters,offset)
%OFDMChannelResponse OFDM channel response
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   OFDMChannelResponse(ofdminfo,pathGains,pathFilters,offset) returns the
%   OFDM channel response as the superposition of path filters OFDM
%   response weighted by the path gains.

%   Copyright 2024 The MathWorks, Inc.

%#codegen

    % Indices of unused FFT bins
    K = ofdminfo.NSubcarriers;
    nfft = ofdminfo.Nfft;
    firstSC = (nfft/2) - (K/2) + 1;
    nullIndices = [1:(firstSC-1) (firstSC+K):nfft].';
    
    % Zero-pad the path filters to match the FFT length and synchronize the
    % path filters according to the timing offset
    [ns,nf] = size(pathFilters);
    pathFilters = [pathFilters; zeros(nfft-ns,nf)];
    if offset ~= 0
        pathFilters = circshift(pathFilters,-offset,1);
    end

    % Frequency response of the synchronized path filters
    Hcfr = fftshift(fft(pathFilters,nfft),1);
    Hcfr(nullIndices,:) = [];

    % Calculate the OFDM channel response by superposition of the path
    % filters response weighted by the path gains
    pg = permute(pathGains,[2 1 4 3]);
    H = pagemtimes(Hcfr,pg);

end
