function [offset,mag] = nrPerfectTimingEstimate(pathGains,pathFilters)
%nrPerfectTimingEstimate perfect timing estimation
%   [OFFSET,MAG] = nrPerfectTimingEstimate(PATHGAINS,PATHFILTERS) performs
%   perfect timing estimation. To find the peak of the channel impulse
%   response, the function first reconstructs the impulse response from the
%   channel path gains array PATHGAINS and the path filter impulse response
%   matrix PATHFILTERS. The function returns the estimated timing offset
%   OFFSET in samples and the channel impulse response magnitude MAG.
%
%   PATHGAINS must be an array of size Ncs-by-Np-by-Nt-by-Nr, where Ncs is
%   the number of channel snapshots, Np is the number of paths, Nt is the
%   number of transmit antennas and Nr is the number of receive antennas.
%   The channel impulse response is averaged across all channel snapshots
%   and summed across all transmit antennas and receive antennas before
%   timing estimation.
%
%   PATHFILTERS must be a matrix of size Nh-by-Np where Nh is the number of
%   impulse response samples.
%
%   OFFSET is a scalar indicating estimated timing offset, an integer
%   number of samples relative to the first sample of the channel impulse
%   response reconstructed from PATHGAINS and PATHFILTERS.
%
%   MAG is a matrix of size Nh-by-Nr giving the impulse response magnitude
%   for each receive antenna.
%
%   Example:
%   % Configure a TDL-C channel with 100ns delay spread and plot the 
%   % impulse response magnitude and estimated timing offset.
%
%   tdl = nrTDLChannel;
%   tdl.DelayProfile = 'TDL-C';
%   tdl.DelaySpread = 100e-9;
%   
%   tdlInfo = info(tdl);
%   Nt = tdlInfo.NumTransmitAntennas;
%   in = complex(zeros(100,Nt),zeros(100,Nt));
%
%   [~,pathGains] = tdl(in);
%   pathFilters = getPathFilters(tdl);
%
%   [offset,mag] = nrPerfectTimingEstimate(pathGains,pathFilters);
%   [Nh,Nr] = size(mag);
%   plot(0:(Nh-1),mag,'o:');
%   hold on;
%   plot([offset offset],[0 max(mag(:))*1.25],'k:','LineWidth',2);
%   axis([0 Nh-1 0 max(mag(:))*1.25]);
%   legends = "|h|, antenna " + num2cell(1:Nr);
%   legend([legends "Timing offset estimate"]);
%   ylabel('|h|');
%   xlabel('Channel impulse response samples');
%
%   See also nrPerfectChannelEstimate, nrTimingEstimate, nrChannelEstimate,
%   nrTDLChannel, nrCDLChannel.

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

    narginchk(2,2);

    % Validate inputs
    fcnName = 'nrPerfectTimingEstimate';
    validateattributes(pathFilters,{'double'}, ...
        {'2d'},fcnName,'PATHFILTERS');
    coder.internal.errorIf(size(pathGains,2)~=size(pathFilters,2), ...
        'nr5g:nrPerfectTimingEstimate:InconsistentPaths', ...
        size(pathGains,2),size(pathFilters,2));

    [offset,mag] = channelDelay(pathGains,pathFilters.');
    
end
