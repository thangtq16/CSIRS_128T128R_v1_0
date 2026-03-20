classdef nrCDLChannel< matlab.System
%nrCDLChannel TR 38.901 Clustered Delay Line (CDL) channel
%   CHAN = nrCDLChannel creates a CDL MIMO fading channel System object,
%   CHAN. This object filters an input signal through the CDL MIMO channel
%   to obtain the channel-impaired signal. This object implements the
%   following aspects of TR 38.901:
%   * Section 7.7.1 Clustered Delay Line (CDL) models
%   * Section 7.7.3 Scaling of delays 
%   * Section 7.7.6 K-factor for LOS channel models
%   * Section 7.7.5.1 Scaling of angles
%   * Section 7.6.10 Dual mobility
%   Note that TR 38.901 supersedes the original TR 38.900 study report.
%
%   CHAN = nrCDLChannel(Name,Value) creates a CDL MIMO channel object,
%   CHAN, with the specified property Name set to the specified Value. You
%   can specify additional name-value pair arguments in any order as
%   (Name1,Value1,...,NameN,ValueN).
%   
%   CHAN = nrCDLChannel(DelayProfile='Custom',InitialDelayProfile=PROFILE,
%   DelaySpread=SPREAD,KFactor=K) creates a custom CDL channel object. The
%   initial delay profile PROFILE ('CDL-A','CDL-B','CDL-C','CDL-D',
%   'CDL-E') sets the PathDelays, AveragePathGains, AnglesAoD, AnglesAoA,
%   AnglesZoD, AnglesZoA, HasLOSCluster, KFactorFirstCluster, AngleSpreads,
%   and XPR properties to the corresponding table values from TR 38.901
%   Tables 7.7.1-1 to 7.7.1-5. SPREAD and K are optional inputs to specify
%   the delay spread and Rician K factor of the channel. The default
%   DelaySpread is 30 ns and no K-factor scaling for LOS channels.
%   
%   Step method syntax for ChannelFiltering set to true:
%
%   Y = step(CHAN,X) filters the input signal X through a CDL MIMO fading
%   channel and returns the result in Y. The input X can be a double or
%   single precision data type scalar, vector, or 2-D matrix. X is of size
%   Ns-by-Nt, where Ns is the number of samples and Nt is the number of
%   transmit antennas. Y is the output signal of size Ns-by-Nr, where Nr is
%   the number of receive antennas. Y contains values of the same type as
%   the input signal X.
% 
%   [Y,PATHGAINS] = step(CHAN,X) also returns the MIMO channel path gains
%   of the underlying fading process in PATHGAINS. PATHGAINS is of size
%   Ncs-by-Np-by-Nt-by-Nr, where Np is the number of paths, and Ncs is the
%   number of channel snapshots, controlled by the SampleDensity property.
%   PATHGAINS is of the same data type as the input signal X.
%
%   [Y,PATHGAINS,SAMPLETIMES] = step(CHAN,X) also returns the sample times
%   of the channel snapshots (1st dimension elements) of PATHGAINS.
%   SAMPLETIMES is of size Ncs-by-1 and is of double precision data type
%   with real values. To use this syntax, set ChannelResponseOutput to
%   'path-gains'.
%
%   [Y,OFDMRESPONSE,OFFSET] = step(CHAN,X,CARRIER) returns the channel OFDM
%   response OFDMRESPONSE and the timing offset OFFSET in samples
%   associated to the strongest path in the channel. OFDMRESPONSE is a
%   K-by-N-by-Nr-by-Nt array where K is the number of subcarriers, N is the
%   number of OFDM symbols in the input waveform, Nr is the number of
%   receive antennas and Nt is the number of transmit antennas.
%   OFDMRESPONSE is calculated by applying OFDM demodulation to the channel
%   impulse response according to the carrier configuration object CARRIER.
%   To use this syntax, set ChannelResponseOutput to 'ofdm-response'.
%
%   Step method syntax for ChannelFiltering set to false:
%
%   [PATHGAINS,SAMPLETIMES] = step(CHAN) produces path gains PATHGAINS and
%   sample times SAMPLETIMES as described above, where the duration of the
%   fading process is given by the NumTimeSamples property. In this case
%   the object acts as a source of path gains and sample times without
%   filtering an input signal. The data type of PATHGAINS is specified by
%   the OutputDataType property. To use this syntax, set
%   ChannelResponseOutput to 'path-gains'.
%
% 
%   [OFDMRESPONSE,OFFSET] = step(CHAN,CARRIER) returns the channel
%   frequency response OFDMRESPONSE and the timing offset OFFSET in samples
%   associated to the strongest path in the channel. OFDMRESPONSE is a
%   K-by-N-by-Nr-by-Nt array where K is the number of subcarriers, N is the
%   number of OFDM symbols in the input waveform, Nr is the number of
%   receive antennas and Nt is the number of transmit antennas. The number
%   of OFDM symbols N depends on the value of the NumTimeSamples property.
%   OFDMRESPONSE is calculated by applying OFDM demodulation to the channel
%   impulse response according to the carrier configuration object CARRIER.
%   To use this syntax, set ChannelResponseOutput to 'ofdm-response'.
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj,x) and y = obj(x) are
%   equivalent.
%
%   nrCDLChannel methods:
%
%   step                   - Filter input signal through a CDL MIMO fading channel
%                            (see above)
%   release                - Allow property value and input characteristics changes
%   clone                  - Create a CDL channel object with same property values
%   isLocked               - Locked status (logical)
%   <a href="matlab:help nrCDLChannel/reset">reset</a>                  - Reset states of filters, and random stream if the
%                            RandomStream property is set to 'mt19937ar with seed'
%   <a href="matlab:help nrCDLChannel/infoImpl">info</a>                   - Return characteristic information about the CDL 
%                            channel
%   getPathFilters         - Get filter impulse responses for the filters which 
%                            apply the path delays to the input waveform
%   displayChannel         - Visualize CDL channel characteristics
%   swapTransmitAndReceive - Swap transmit and receive antennas
%
%   nrCDLChannel properties:
%
%   DelayProfile              - CDL delay profile
%   PathDelays                - Discrete path delay vector (s)
%   AveragePathGains          - Average path gain vector (dB)
%   AnglesAoD                 - Azimuth of departure angles vector (deg)
%   AnglesAoA                 - Azimuth of arrival angles vector (deg)
%   AnglesZoD                 - Zenith of departure angles vector (deg)
%   AnglesZoA                 - Zenith of arrival angles vector (deg)
%   HasLOSCluster             - Line of sight cluster (logical)
%   KFactorFirstCluster       - K-factor of first cluster (dB)
%   AngleScaling              - Enable angle scaling (logical)
%   AngleSpreads              - Scaled or cluster-wise RMS angle spreads vector (deg)
%   MeanAngles                - Scaled mean angles vector (deg)
%   RayCoupling               - Ray coupling within a cluster
%   XPR                       - Cross polarization power ratio (dB)
%   InitialPhases             - Initial phases (deg)
%   DelaySpread               - Desired delay spread (s)
%   CarrierFrequency          - Carrier frequency (Hz)
%   MaximumDopplerShift       - Maximum Doppler shift (Hz)
%   UTDirectionOfTravel       - User equipment (UE) direction of travel (deg)
%   KFactorScaling            - Enable K-factor scaling (logical)
%   KFactor                   - Desired Rician K-factor (dB)
%   SampleRate                - Input signal sample rate (Hz)
%   MovingScattererProportion - Proportion of moving scatterers in the channel
%   MaximumScattererSpeed     - Maximum speed of moving scatterers (m/s)
%   TransmitAntennaArray      - Transmit antenna array characteristics
%   TransmitArrayOrientation  - Orientation of the transmit antenna array
%   ReceiveAntennaArray       - Receive antenna array characteristics
%   ReceiveArrayOrientation   - Orientation of the receive antenna array
%   SampleDensity             - Number of time samples per half wavelength 
%   NormalizePathGains        - Normalize channel fading process (logical)
%   InitialTime               - Start time of fading process (s)
%   NumStrongestClusters      - Number of strongest clusters to split into subclusters
%   ClusterDelaySpread        - Cluster delay spread (s)
%   RandomStream              - Source of random number stream
%   Seed                      - Initial seed of mt19937ar random number stream
%   NormalizeChannelOutputs   - Normalize channel outputs (logical)
%   ChannelFiltering          - Perform filtering of input signal (logical)
%   NumTimeSamples            - Number of time samples
%   OutputDataType            - Path gain output data type
%   TransmitAndReceiveSwapped - Transmit and receive antennas swapped (logical)
%   ChannelResponseOutput     - Specify the type of the returned channel response
% 
%   % Example 1:
%   % Configure a CDL channel and filter an input signal. Use CDL-D delay 
%   % profile, 10 ns delay spread and UE velocity 15 km/h.
%
%   v = 15.0;                    % UE velocity in km/h
%   fc = 4e9;                    % carrier frequency in Hz
%   c = physconst('lightspeed'); % speed of light in m/s
%   fd = (v*1000/3600)/c*fc;     % UE max Doppler frequency in Hz
%
%   cdl = nrCDLChannel;
%   cdl.DelayProfile = 'CDL-D';
%   cdl.DelaySpread = 10e-9;
%   cdl.CarrierFrequency = fc;
%   cdl.MaximumDopplerShift = fd;
%
%   % Configure transmit and receive antenna arrays. The transmit array is
%   % configured as [M N P M_g N_g] = [2 2 2 1 1], corresponding to 1 panel
%   % (M_g=1,N_g=1) with 2x2 antennas (M=2,N=2) and two polarization angles
%   % (P=2). The receive antenna array is configured as [M N P M_g N_g] = 
%   % [1 1 2 1 1], corresponding to a single pair of cross-polarized 
%   % co-located antennas
%
%   cdl.TransmitAntennaArray.Size = [2 2 2 1 1];
%   cdl.ReceiveAntennaArray.Size = [1 1 2 1 1];
%
%   % Create a random waveform of 1 subframe duration with 8 antennas
%   % and pass it through the channel
%
%   SR = 15.36e6;
%   T = SR * 1e-3;
%   cdl.SampleRate = SR;
%   cdlinfo = info(cdl);
%   Nt = cdlinfo.NumInputSignals;
%
%   txWaveform = complex(randn(T,Nt),randn(T,Nt));
%
%   rxWaveform = cdl(txWaveform);
%
%   % Example 2: 
%   % Demonstrate the effect of the SampleDensity property. 
%
%   % Configure a channel for SISO operation and delay profile CDL-B. Set
%   % the maximum Doppler shift to 300 Hz and the channel sampling rate to
%   % 10 kHz
%
%   cdl = nrCDLChannel;
%   cdl.TransmitAntennaArray.Size = [1 1 1 1 1];
%   cdl.ReceiveAntennaArray.Size = [1 1 1 1 1];
%   cdl.DelayProfile = 'CDL-B';
%   cdl.MaximumDopplerShift = 300.0;
%   cdl.SampleRate = 10e3;
%   cdl.Seed = 19;
% 
%   % Plot the step response of the channel and the corresponding path gain
%   % snapshots for various values of the SampleDensity property, which
%   % controls how often the channel snapshots are taken relative to the
%   % Doppler frequency
%
%   T = 40; in = ones(T,1); SR = cdl.SampleRate;
%   disp(['input length T=' num2str(T) ' samples']);
%   s = [Inf 5 2]; % sample densities
% 
%   legends = {};
%   figure; hold on;
%   for i = 1:length(s)
%     
%       % execute channel with chosen sample density
%       release(cdl); cdl.SampleDensity = s(i);
%       [out,pathgains,sampletimes] = cdl(in);
%       chInfo = info(cdl); tau = chInfo.ChannelFilterDelay;
%     
%       % plot channel output against time
%       t = cdl.InitialTime + ((0:(T-1))-tau).' / SR;
%       h = plot(t,abs(out),'o-'); h.MarkerSize = 2; h.LineWidth = 1.5;
%       desc = ['SampleDensity=' num2str(s(i))];
%       legends = [legends ['output, ' desc]];
%       disp([desc ', Ncs=' num2str(length(sampletimes))]);
%     
%       % plot path gains against sample times
%       h2 = plot(sampletimes-tau/SR,abs(sum(pathgains,2)),'o');
%       h2.Color = h.Color; h2.MarkerFaceColor = h.Color;
%       legends = [legends ['path gains, ' desc]];
%     
%   end
%   xlabel('time (s)');
%   title('Channel output and path gains versus SampleDensity');
%   ylabel('channel magnitude');
%   legend(legends,'Location','NorthWest');
%
%   % SampleDensity equal to Inf ensures that a channel snapshot is taken
%   % for every input sample. SampleDensity equal to X takes channel
%   % snapshots at a rate of Fcs, which is equal to 
%   % 2 * X * (sum(MaximumDopplerShift) + (2 * (MaximumScattererSpeed/lambda0)))
%   % where lambda0 is the carrier wavelength.
%   % The channel snapshots are applied to the input waveform by means of
%   % zero order hold interpolation. Note that an extra snapshot is taken
%   % beyond the end of the input, where some of the final output samples
%   % use this extra value to help minimize the interpolation error. Note
%   % that the channel output contains a transient (and delay) due to the
%   % filters that implement the path delays.
%
%   % Example 3:
%   % Configure a 64-by-4 channel, set the delay spread to 300 ns,
%   % filter an input signal, and plot the received waveform spectrum.
%
%   % Specify the antenna array geometry
%   cdl = nrCDLChannel;
%   cdl.TransmitAntennaArray.Size = [2 4 2 2 2];
%   cdl.TransmitAntennaArray.ElementSpacing = [0.5 0.5 1.0 2.0];
%   cdl.ReceiveAntennaArray.Size = [2 1 2 1 1];
%   cdl.DelaySpread = 300e-9;
%
%   % Create a random waveform of 1 subframe duration for 64 antennas
%   SR = 30.72e6;
%   T = SR * 1e-3;
%   cdl.SampleRate = SR;
%   cdlinfo = info(cdl);
%   Nt = cdlinfo.NumInputSignals;
%   txWaveform = complex(randn(T,Nt),randn(T,Nt));
%
%   % The antenna array elements are mapped to the waveform channels
%   % (columns) in the order that a 5-D array of size
%   % TransmitAntennaArray.Size or ReceiveAntennaArray.Size is linearly
%   % indexed (across the dimensions first to last). See the
%   % TransmitAntennaArray or ReceiveAntennaArray property help for more
%   % details.
%
%   % Pass the waveform through the channel
%   rxWaveform = cdl(txWaveform);
%
%   % Plot the received waveform spectrum
%   analyzer = spectrumAnalyzer('SampleRate',cdl.SampleRate);
%   analyzer.Title = ['Received signal spectrum for ' cdl.DelayProfile];
%   analyzer(rxWaveform);
%
%   % Example 4: 
%   % Configure a LOS channel and specify the orientation of the transmit 
%   % and receive antenna arrays to point to each other.
%
%   cdl = nrCDLChannel; 
%   cdl.DelayProfile = 'CDL-D'; % LOS channel
%   cdl.TransmitAntennaArray.Element = '38.901';
%   cdl.ReceiveAntennaArray.Element = '38.901';
%
%   % Use path angles of the LOS component to orient the Tx and Rx arrays
%   cdlInfo = cdl.info;
%   txLOSOrientation = [cdlInfo.AnglesAoD(1) cdlInfo.AnglesZoD(1)-90 0]';
%   rxLOSOrientation = [cdlInfo.AnglesAoA(1) cdlInfo.AnglesZoA(1)-90 0]';
%   cdl.TransmitArrayOrientation = txLOSOrientation; 
%   cdl.ReceiveArrayOrientation = rxLOSOrientation; 
%
%   cdl.displayChannel('LinkEnd','Tx');
%   view(0,90)
%   cdl.displayChannel('LinkEnd','Rx')
%   view(0,90)
%
%   % Example 5: 
%   % Configure a CDL channel with a Phased Array System Toolbox(TM)
%   % antenna array and display the channel characteristics.
%   
%   cdl = nrCDLChannel;
%   cdl.TransmitAntennaArray = phased.URA;
%   cdl.TransmitAntennaArray.Element = phased.CrossedDipoleAntennaElement;
%   cdl.TransmitAntennaArray.ArrayNormal = 'y';
%   cdl.TransmitArrayOrientation = [0; 30; 0];
%   
%   cdl.displayChannel('LinkEnd','Tx');
%
%   See also nrTDLChannel, nrHSTChannel, comm.MIMOChannel,
%   nrPerfectChannelEstimate, nrPerfectTimingEstimate.

 
%   Copyright 2016-2024 The MathWorks, Inc.

    methods
        function out=nrCDLChannel
            % Parse inputs for construction of custom CDL channels based on
            % the input InitialDelayProfile
        end

        function out=displayChannel(~) %#ok<STOUT>
            %displayChannel Show CDL channel characteristics
            %   HFIG = displayChannel(obj) displays geometric and
            %   electromagnetic characteristics of the CDL channel at the
            %   transmitter and receiver. The visualization includes the
            %   position, polarization, and directivity radiation pattern of
            %   the antenna elements, and the cluster paths directions and
            %   average path gains. Since all antenna elements are equal, the
            %   visualization only contains the radiation pattern of the first
            %   one. The cluster paths are centered at the first element of the
            %   array to help visualize its orientation relative to the cluster
            %   paths directions. The output HFIG is an array of handles to the
            %   figures created.
            %
            %   HFIG = displayChannel(...,NAME,VALUE) specifies additional
            %   options as NAME,VALUE pairs to allow control over the display
            %   of individual characteristics of the CDL channel:
            %
            %   'LinkEnd'        - 'Both' displays transmit and receive sides
            %                      (default)
            %                      'Tx' displays the transmit side only
            %                      'Rx' displays the receive side only
            %   'Polarization'   - Polarization angle of the antenna elements
            %                      'on' (default), 'off'
            %   'ElementPattern' - Directivity radiation pattern of the antenna
            %                      elements 'on' (default), 'off'
            %   'ClusterPaths'   - Direction and average gain of cluster paths
            %                      'on' (default), 'off'
        end

        function out=getNumInputsImpl(~) %#ok<STOUT>
        end

        function out=getNumOutputsImpl(~) %#ok<STOUT>
        end

        function out=getPathFilters(~) %#ok<STOUT>
            %getPathFilters Get path filter impulse responses
            %   H = getPathFilters(obj) returns a double precision real matrix
            %   of size Nh-by-Np where Nh is the number of impulse response
            %   samples and Np is the number of paths. Each column of H
            %   contains the filter impulse response for each path of the delay
            %   profile. This information facilitates reconstruction of a
            %   perfect channel estimate when used in conjunction with the
            %   PATHGAINS output of the step method. These filters do not
            %   change once the object is created, therefore it only needs to
            %   be called once.
        end

        function out=infoImpl(~) %#ok<STOUT>
            %info Returns characteristic information about the CDL channel
            %   S = info(CHAN) returns a structure containing characteristic
            %   information, S, about the CDL fading channel. A description of
            %   the fields and their values is as follows:
            % 
            %   ClusterTypes        - A row cell array of character vectors,
            %                         indicating the type of each cluster in
            %                         the delay profile ('LOS',
            %                         'SubclusteredNLOS', 'NLOS')
            %   PathDelays          - A row vector providing the delays of the
            %                         discrete channel paths, in seconds. These
            %                         values include the effect of the desired
            %                         delay spread scaling, and desired
            %                         K-factor scaling if enabled. 
            %   AveragePathGains    - A row vector of the average gains of the
            %                         discrete path or cluster, in dB. These
            %                         values include the effect of K-factor
            %                         scaling if enabled.
            %   AnglesAoD           - A row vector of the Azimuth of Departure
            %                         angles of the clusters in degrees.
            %   AnglesAoA           - A row vector of the Azimuth of Arrival
            %                         angles of the clusters in degrees.
            %   AnglesZoD           - A row vector of the Zenith of Departure
            %                         angles of the clusters in degrees.
            %   AnglesZoA           - A row vector of the Zenith of Arrival
            %                         angles of the clusters in degrees.
            %   KFactorFirstCluster - K-factor of first cluster of delay
            %                         profile, in dB. If the first cluster of
            %                         the delay profile follows a Laplacian
            %                         rather than Rician distribution,
            %                         KFactorFirstCluster is -Inf.
            %   ClusterAngleSpreads - A row vector of cluster-wise RMS angle
            %                         spreads [C_ASD C_ASA C_ZSD C_ZSA] (deg)
            %   XPR                 - A scalar of cross polarization power
            %                         ratio (dB) or NaN when DelayProfile =
            %                         'Custom' and XPR is specified as a
            %                         matrix.
            %   NumTransmitAntennas - Number of transmit antennas.
            %   NumInputSignals     - Number of input signals to CDL channel.
            %   NumReceiveAntennas  - Number of receive antennas.
            %   NumOutputSignals    - Number of output signals of CDL channel.
            %   ChannelFilterDelay  - Channel filter delay in samples.
            %   MaximumChannelDelay - Maximum channel delay in samples. This
            %                         delay consists of the maximum propagation
            %                         delay and the ChannelFilterDelay.
            %
            %   Note that the step of splitting of the strongest clusters into
            %   sub-clusters described in TR 38.901 Section 7.5 requires
            %   sorting of the clusters by their average power. Therefore if
            %   the NumStrongestClusters property is non-zero (only applicable
            %   for DelayProfile='Custom') the fields of the information
            %   structure are sorted by average power. AveragePathGains is in
            %   descending order of average gain and ClusterTypes, PathDelays,
            %   AnglesAoD, AnglesAoA, AnglesZoD and AnglesZoA are sorted
            %   accordingly. Also, if the HasLOSCluster property is set, the
            %   NLOS (Laplacian) part of that cluster can be sorted such that
            %   it is not adjacent to the LOS cluster. However, KFactorFirstCluster
            %   still indicates the appropriate K-factor.
        end

        function out=isElementSetHomogeneous(~) %#ok<STOUT>
        end

        function out=isHomogeneous(~) %#ok<STOUT>
        end

        function out=isInactivePropertyImpl(~) %#ok<STOUT>
        end

        function out=isInputComplexityMutableImpl(~) %#ok<STOUT>
        end

        function out=isInputSizeMutableImpl(~) %#ok<STOUT>
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
            % Handling backwards compatibility of array orientation
        end

        function out=makeCDLChannelStructure(~) %#ok<STOUT>
        end

        function out=makeCustomCDL(~) %#ok<STOUT>
            % Create a CDL preset model struct to help set up the output
            % custom CDL channel. This model struct contains the average
            % path gains, path delays, and angles of the specified input
            % CDL preset scaled by the input delay spread and K factor.
        end

        function out=matlabCodegenSoftNontunableProperties(~) %#ok<STOUT>
        end

        function out=processTunedPropertiesImpl(~) %#ok<STOUT>
            % If any property has been tuned that influences the setup and
            % reset steps
        end

        function out=releaseImpl(~) %#ok<STOUT>
        end

        function out=resetImpl(~) %#ok<STOUT>
            % reset the time to the last InitialTime property value set
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
        end

        function out=setupChannelFilter(~) %#ok<STOUT>
        end

        function out=setupImpl(~) %#ok<STOUT>
            % Set up everything that can be influenced by the tunable
            % properties; the object is currently unlocked, so treat all
            % properties as if they have been tuned
        end

        function out=stepImpl(~) %#ok<STOUT>
            % Configure the input size and output data type. Note that the
            % number of antennas in 'insize', used by CDLChannel below, is
            % applicable to TransmitAndReceiveSwapped = false. If
            % TransmitAndReceiveSwapped = true, the path gains are
            % subsequently permuted to give the reciprocal channel
        end

        function out=swapTransmitAndReceive(~) %#ok<STOUT>
            %swapTransmitAndReceive Swap transmit and receive antennas
            %   Call this method to swap the role of the transmit and receive
            %   antennas within the channel model, corresponding to reversing
            %   the link direction of the channel. Calling this method does not
            %   alter the channel fading. Therefore, if P is the path gains
            %   array obtained from a channel object without calling
            %   swapTransmitAndReceive and PT is the path gains array of an
            %   identical object after calling swapTransmitAndReceive, then PT
            %   = permute(P,[1 2 4 3]). That is, P and PT have their transmit
            %   and receive antenna dimensions swapped, therefore they
            %   represent reciprocal channels. If the method is called again,
            %   the transmit and receive antennas are swapped back (the link
            %   reverts to the original link direction). By calling this method
            %   during a simulation, and passing waveforms for each link
            %   direction to the channel, TDD operation can be modeled while
            %   maintaining channel reciprocity. To establish the current state
            %   of the channel, inspect the TransmitAndReceiveSwapped property.
            %   Note that when the transmit and receive antennas are swapped,
            %   the following property pairs are swapped to reflect the change
            %   of link direction: TransmitAntennaArray and
            %   ReceiveAntennaArray, TransmitArrayOrientation and
            %   ReceiveArrayOrientation, and additionally for DelayProfile =
            %   'Custom': AnglesAoD and AnglesAoA, AnglesZoD and AnglesZoA. The
            %   AngleSpreads and MeanAngles properties are also rearranged to
            %   swap elements related to departure and arrival angles. The
            %   following pairs of fields of the info method output structure
            %   are also swapped: NumTransmitAntennas and NumReceiveAntennas,
            %   NumInputSignals and NumOutputSignals.
        end

        function out=validateAntennaArray(~) %#ok<STOUT>
        end

        function out=validateInputsImpl(~) %#ok<STOUT>
        end

        function out=validatePropertiesImpl(~) %#ok<STOUT>
        end

    end
    properties
        %AngleScaling Enable angle scaling (logical)
        %   Set this property to true to apply scaling of angles as
        %   described in TR 38.901 Section 7.7.5.1. This property applies
        %   when you set the DelayProfile property to 'CDL-A', 'CDL-B',
        %   'CDL-C', 'CDL-D' or 'CDL-E'. When you set this property to 
        %   true, the desired angle scaling is set using the AngleSpreads 
        %   and MeanAngles properties.
        %
        %   The default value of this property is false.
        AngleScaling;

        %AngleSpreads Scaled or cluster-wise RMS angle spreads vector (deg)
        %   
        %   When DelayProfile is set to 'CDL-A', 'CDL-B', 'CDL-C', 'CDL-D',
        %   or 'CDL-E' and AngleScaling is set to true, AngleSpreads
        %   specifies the values of the desired root-mean square (RMS)
        %   angle spreads of the channel (AS_desired) used for angle
        %   scaling. This property is specified as a row vector [ASD ASA
        %   ZSD ZSA]. ASD is the desired RMS azimuth spread of departure
        %   angles. ASA, ZSD, and ZSA are the corresponding azimuth spread
        %   of arrival, zenith spread of departure, and zenith spread of
        %   arrival angles, respectively. See TR 38.901 Section 7.7.5.1.
        %   
        %   When DelayProfile is set to 'Custom', AngleSpreads specifies
        %   the values of the cluster-wise RMS angle spreads used for
        %   scaling ray offset angles within a cluster. This property is
        %   specified as a row vector [C_ASD C_ASA C_ZSD C_ZSA]. C_ASD is
        %   the cluster-wise RMS azimuth spread of departure angles. C_ASA,
        %   C_ZSD, and C_ZSA are the corresponding values for the azimuth
        %   spread of arrival, zenith spread of departure, and zenith
        %   spread of arrival angles, respectively. See TR 38.901 Section
        %   7.7.1 step 1. In this case, angle scaling according to Section
        %   7.7.5.1 is not performed.
        %
        %   The default value of this property is [5.0 11.0 3.0 3.0].
        AngleSpreads;

        %AnglesAoA Azimuth of arrival angles vector (deg)
        %   Specify the azimuth of arrival angle for each cluster in
        %   degrees as a double-precision, real, scalar or row vector. This
        %   property applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0.
        AnglesAoA;

        %AnglesAoD Azimuth of departure angles vector (deg)
        %   Specify the azimuth of departure angle for each cluster in
        %   degrees as a double-precision, real, scalar or row vector. This
        %   property applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0.
        AnglesAoD;

        %AnglesZoA Zenith of arrival angles vector (deg)
        %   Specify the zenith of arrival angle for each cluster in degrees
        %   as a double-precision, real, scalar or row vector. This
        %   property applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0.
        AnglesZoA;

        %AnglesZoD Zenith of departure angles vector (deg)
        %   Specify the zenith of departure angle for each cluster in
        %   degrees as a double-precision, real, scalar or row vector. This
        %   property applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0.
        AnglesZoD;

        %AveragePathGains Average path gain vector (dB)
        %   Specify the average gains of the discrete paths in decibels as
        %   a double-precision, real, scalar or row vector. The average
        %   path gains are also referred to as cluster powers in TR 38.901.
        %   AveragePathGains must have the same size as PathDelays. This
        %   property applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0.
        AveragePathGains;

        %CarrierFrequency Carrier frequency (Hz)
        %   Specify the carrier frequency in Hertz as a scalar.
        %   
        %   The default value of this property is 4 GHz.
        CarrierFrequency;

        %ChannelFiltering Perform filtering of input signal (logical)
        %   Set this property to false to disable channel filtering. If set
        %   to false then the step method does not accept an input signal
        %   and the duration of the fading process realization is
        %   controlled by the NumTimeSamples property (at the sampling rate
        %   given by the SampleRate property). In this case, the step
        %   method only returns the path gains and sample times but no
        %   output signal.
        %
        %   The default value of this property is true.
        ChannelFiltering;

        %ChannelResponseOutput Specify the type of returned channel response
        %   When this property is set to 'path-gains', the step method
        %   returns the channel path gains and the corresponding sample
        %   times. Set this property to 'ofdm-response' to make the step
        %   method return the channel OFDM response and the timing offset.
        %
        %   The default value of this property is 'path-gains'.
        ChannelResponseOutput;

        %ClusterDelaySpread Cluster delay spread (s)
        %   Specify the cluster delay spread (C_DS) as a real nonnegative
        %   scalar in seconds. The value is used to specify the delay
        %   offset between sub-clusters for clusters split into
        %   sub-clusters. See TR 38.901 Section 7.5 step 11. This property
        %   applies when DelayProfile is set to 'Custom' and
        %   NumStrongestClusters is greater than zero.
        %
        %   The default value of this property is 3.90625ns.
        ClusterDelaySpread;

        %DelayProfile CDL delay profile
        %   Specify the CDL delay profile as one of 'CDL-A', 'CDL-B',
        %   'CDL-C', 'CDL-D', 'CDL-E' or 'Custom'. See TR 38.901 Section
        %   7.7.1, Tables 7.7.1-1 to 7.7.1-5. When you set this property to
        %   'Custom', the delay profile is configured using the following 
        %   properties: PathDelays, AveragePathGains, AnglesAoD, AnglesAoA, 
        %   AnglesZoD, AnglesZoA, HasLOSCluster, KFactorFirstCluster, 
        %   AngleSpreads, XPR, NumStrongestClusters.
        %
        %   The default value of this property is 'CDL-A'.
        DelayProfile;

        %DelaySpread Desired delay spread (s)
        %   Specify the desired RMS delay spread in seconds (DS_desired) as
        %   a scalar. See TR 38.901 Section 7.7.3, and Tables 7.7.3-1 and
        %   7.7.3-2 for examples of desired RMS delay spreads. This
        %   property applies when you set the DelayProfile property to
        %   'CDL-A', 'CDL-B', 'CDL-C', 'CDL-D' or 'CDL-E'.
        %
        %   The default value of this property is 30e-9.
        DelaySpread;

        %HasLOSCluster Line of sight cluster (logical)
        %   Set this property to true to specify that the delay profile has
        %   a line of sight (LOS) cluster. This property applies when
        %   DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is false.
        HasLOSCluster;

        %InitialPhases Initial phases (deg)
        %   Specify the initial phases of all rays for the four
        %   polarization combinations in degrees. If set to 'Random', the
        %   channel draws uniformly distributed random phases, as defined
        %   in TR 38.901 Section 7.5 Step 10, using the random number
        %   stream, RandomStream. Otherwise, set InitialPhases to an array
        %   of size N-by-M-by-4 to explicitly initialize the phases, where
        %   N is the number of clusters and M is the number of rays per
        %   cluster (M=20 rays). The four N-by-M planes, in the third
        %   dimension, correspond to the theta/theta, theta/phi, phi/theta,
        %   and phi/phi polarization combinations, respectively. Note that
        %   N is the number of clusters before any splitting into
        %   subclusters (see NumStrongestClusters), not considering the LOS
        %   cluster (see HasLOSCluster), and therefore is equal to the
        %   number of elements in PathDelays. This property applies when
        %   DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 'Random'.
        InitialPhases;

        %InitialTime Start time of fading process (s)
        %   Specify the time offset of the fading process as a real
        %   nonnegative scalar. This property is tunable. 
        %
        %   The default value of this property is 0.0.
        InitialTime;

        %KFactor Desired Rician K-factor (dB)
        %   Specify the desired K-factor in dB (K_desired) as a scalar.
        %   This property applies when you set the KFactorScaling property
        %   to true. See TR 38.901 Section 7.7.6, and see Table 7.5-6 for
        %   typical K-factors. Note that K-factor scaling modifies both the
        %   path delays and path powers. Note that the K-factor applies to
        %   the overall delay profile. The K-factor before the scaling
        %   is K_model described in TR 38.901 Section 7.7.6, the ratio of
        %   the power of the LOS part of the first cluster to the total
        %   power of all the Laplacian clusters, including the Laplacian
        %   part of the first cluster.
        %
        %   The default value of this property is 9.0 dB.
        KFactor;

        %KFactorFirstCluster K-factor of first cluster (dB)
        %   Specify the K-factor of the first cluster of the delay profile
        %   in dB (K_1) as a scalar. This property applies when
        %   DelayProfile is set to 'Custom' and HasLOSCluster is set to
        %   true.
        %
        %   The default value of this property is 13.3 dB. This is the
        %   value defined for delay profile CDL-D.
        KFactorFirstCluster;

        %KFactorScaling Apply K-factor scaling (logical)
        %   Set this property to true to apply K-factor scaling as
        %   described in TR 38.901 Section 7.7.6. Note that K-factor
        %   scaling modifies both the path delays and path powers. This
        %   property applies if DelayProfile is set to 'CDL-D' or 'CDL-E'.
        %   When you set this property to true, the desired K-factor is set
        %   using the KFactor property.
        %
        %   The default value of this property is false.
        KFactorScaling;

        %MaximumDopplerShift Maximum Doppler shift (Hz) 
        %   Specify the maximum Doppler shift for all channel paths in
        %   Hertz as a double precision, real, nonnegative scalar or row
        %   vector. When specified as a scalar, this property is the maximum
        %   Doppler shift of the receiver. When specified as a 
        %   row vector of the form [Rx Tx], Rx and Tx specify the maximum 
        %   Doppler shift of the receiver and transmitter, respectively.
        % 
        %   When you set the MaximumDopplerShift to 0, the channel remains static
        %   for the entire input. To generate a new channel realization, 
        %   call the reset function on the nrCDLChannel object.
        %
        %   The default value of this property is 5 Hz.
        MaximumDopplerShift;

        %MaximumScattererSpeed Maximum speed of the moving scatterers in
        %the channel
        %   Specify the maximum speed of the moving scatterers in the channel
        %   in m/s as a real, nonnegative scalar.
        %
        %   This property applies to dual-mobility configurations. To enable 
        %   dual mobility, set the MaximumDopplerShift property to a 1-by-2 
        %   vector or the UTDirectionOfTravel property to a 2-by-2 matrix.
        %   
        %   The default value of this property is 5 m/s.
        MaximumScattererSpeed;

        %MeanAngles Scaled mean angles vector (deg)
        %   Specify the desired mean angles of the channel (mu_desired)
        %   used for angle scaling as a row vector [AoD AoA ZoD ZoA]. AoD
        %   is the desired mean azimuth angle of departure after scaling.
        %   AoA, ZoD, and ZoA are the corresponding values for azimuth of
        %   arrival, zenith of departure, and zenith of arrival,
        %   respectively. See TR 38.901 Section 7.7.5.1. This property only
        %   applies when AngleScaling is set to true.
        %
        %   The default value of this property is zero for each angle.
        MeanAngles;

        %MovingScattererProportion Proportion of moving scatterers
        %   Specify the proportion of moving scatterers in the channel as a real, 
        %   nonnegative scalar between 0 and 1. When 0, none of the scatterers
        %   are moving. When 1, every scatterer is moving.
        %
        %   This property applies to dual-mobility configurations. To enable 
        %   dual mobility, set the MaximumDopplerShift property to a 1-by-2 
        %   vector or the UTDirectionOfTravel property to a 2-by-2 matrix.
        %
        %   The default value of this property is 0.2.
        MovingScattererProportion;

        %NormalizeChannelOutputs Normalize channel outputs by the number of receive antennas (logical)
        %   Set this property to true to normalize the channel outputs by
        %   the number of receive antennas. When you set this property to
        %   false, there is no normalization for channel outputs.
        %
        %   The default value of this property is true.
        NormalizeChannelOutputs;

        %NormalizePathGains Normalize channel fading process (logical)
        %   Set this property to true to normalize the amplitude of the
        %   channel fading process by the average path gains. When you set
        %   this property to false, there is no normalization. The average
        %   path gains are specified by the delay profile (See TR 38.901
        %   Tables 7.7.1-1 to 7.7.1-5) or by the AveragePathGains property
        %   if DelayProfile is set to 'Custom'. The average path gains are
        %   also referred to as cluster powers in TR 38.901. This
        %   normalization does not include other channel gains such as
        %   polarization or antenna element directivity. The default value
        %   of this property is true.
        NormalizePathGains;

        %NumStrongestClusters Number of strongest clusters to split into subclusters
        %   The number of strongest clusters to split into sub-clusters.
        %   See TR 38.901 Section 7.5 step 11. This property applies when
        %   DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.
        NumStrongestClusters;

        %NumTimeSamples Number of time samples
        %   Specify the number of time samples used to set the duration of
        %   the fading process realization as a positive integer scalar.
        %   This property applies when ChannelFiltering is false. This
        %   property is tunable.
        %
        %   The default value of this property is 30720.
        NumTimeSamples;

        %OutputDataType Path gain output data type
        %   Specify the path gain output data type as one of 'double' or
        %   'single'. This property applies when ChannelFiltering is false.
        % 
        %   The default value of this property is 'double'.
        OutputDataType;

        %PathDelays Discrete path delay vector (s)
        %   Specify the delays of the discrete paths in seconds as a
        %   double-precision, real, scalar or row vector. This property
        %   applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0.
        PathDelays;

        %RandomStream Source of random number stream
        %   Specify the source of the random number stream as one of
        %   'Global stream' or 'mt19937ar with seed'. The channel generates
        %   uniformly distributed random numbers to initialize the ray
        %   phases and coupling. When RandomStream is 'Global stream', the
        %   channel generates random numbers using the current global
        %   random number stream. In this case, the reset method only
        %   resets the filters. When RandomStream is 'mt19937ar with seed',
        %   the channel generates random numbers using the mt19937ar
        %   algorithm. In this case, the reset method not only resets the
        %   filters but also reinitializes the random number stream to the
        %   value of the Seed property. Set RandomStream to 'mt19937ar with
        %   seed' to produce repeatable channel fading. When DelayProfile =
        %   'Custom', this property applies if you set the InitialPhases or
        %   RayCoupling properties to 'Random'.
        %
        %   The default value of this property is 'mt19937ar with seed'.
        RandomStream;

        %RayCoupling Ray coupling within a cluster
        %   Specify the coupling of departure and arrival rays within a
        %   cluster for both azimuth and elevation. If set to 'Random', the
        %   channel randomly couples the rays, as defined in TR 38.901
        %   Section 7.5 Step 8, using the random number stream,
        %   RandomStream. Otherwise, set RayCoupling to an array of size
        %   N-by-M-by-3 to explicitly define the ray coupling, where N is
        %   the number of clusters and M is the number of rays per cluster
        %   (M=20 rays). The three N-by-M planes, in the third dimension,
        %   correspond to the AoD/AoA, ZoD/ZoA, and AoD/ZoD ray couplings,
        %   respectively. Each row in each N-by-M plane must be a
        %   permutation of ray indices from 1 to M describing the
        %   corresponding ray coupling within the cluster. Note that N is
        %   the number of clusters before any splitting into subclusters
        %   (see NumStrongestClusters), not considering the LOS cluster
        %   (see HasLOSCluster), and therefore is equal to the number of
        %   elements in PathDelays. This property applies when DelayProfile
        %   is set to 'Custom'.
        %
        %   The default value of this property is 'Random'.
        RayCoupling;

        %ReceiveAntennaArray Receive antenna array characteristics
        %   Structure or Phased Array System Toolbox antenna array object
        %   specifying the receive antenna array. The default is a
        %   structure containing the following fields:
        %   Size                - Size of antenna array [M,N,P,Mg,Ng]. M
        %                         and N are the number of rows and columns
        %                         in the antenna array. P is the number of
        %                         polarizations (1 or 2). Mg and Ng are the
        %                         number of row and column array panels
        %                         respectively. The defaults are 
        %                         [1,1,2,1,1].
        %   ElementSpacing      - Element spacing in wavelengths expressed
        %                         as [lambda_v lambda_h dg_v dg_h]
        %                         representing the vertical and horizontal
        %                         element spacing and the vertical and
        %                         horizontal panel spacing respectively.
        %                         The panel spacing is measured from the
        %                         center of the panels. The defaults are
        %                         [0.5 0.5 0.5 0.5].
        %   PolarizationAngles  - Polarization angles [theta rho] in
        %                         degrees applicable when P is set to 2.
        %                         The defaults are [0 90] degrees.
        %   Orientation         - Mechanical orientation of the array
        %                         [alpha; beta; gamma] in degrees (bearing,
        %                         downtilt, slant). The default values [0;
        %                         0; 0] indicate that the broadside
        %                         direction of the array points to the
        %                         positive x-axis. Orientation will be
        %                         removed in a future release. Use the
        %                         ReceiveArrayOrientation property instead.
        %   Element             - Antenna element radiation pattern. One of
        %                         'isotropic' or '38.901' (see TR 38.901
        %                         Section 7.3). The default value is 
        %                         'isotropic'.
        %   PolarizationModel   - Model describing how to determine the
        %                         radiation field patterns based on a 
        %                         defined radiation power pattern (see
        %                         TR 38.901 Section 7.3.2). One of
        %                         'Model-1' or 'Model-2'. The default value
        %                         is 'Model-2'.
        %
        % The antenna array elements are mapped to the output waveform
        % channels (columns) in the order that a 5-D array of size
        % M-by-N-by-P-by-Mg-by-Ng is linearly indexed (across the
        % dimensions first to last). The size of the array is given by
        % ReceiveAntennaArray.Size = [M,N,P,Mg,Ng]. For example, an antenna
        % array of size [4,8,2,2,2] has the first M (equals 4) channels
        % mapped to the first column of the first polarization angle of the
        % first panel. The next M (equals 4) antennas are mapped to the
        % next column and so on, such that the first M*N (equals 32)
        % channels are mapped to the first polarization angle of the
        % complete first panel. Then the next 32 channels are mapped in the
        % same fashion to the second polarization angle for the first
        % panel. Subsequent sets of M*N*P (equals 64) channels are then
        % mapped to the remaining panels, panel rows first then panel
        % columns.
        ReceiveAntennaArray;

        %ReceiveArrayOrientation Orientation of the receive antenna array
        % Mechanical orientation of the receive antenna array [alpha; beta;
        % gamma] in degrees (bearing, downtilt, slant). The default values
        % [0; 0; 0] indicate that the broadside direction of the array
        % points to the positive x-axis.
        % 
        % When ReceiveAntennaArray is a Phased Array System Toolbox antenna
        % array object, [alpha; beta; gamma] specify three rotation angles
        % (bearing, downtilt, slant) applied to the array oriented in the
        % local coordinate system. Use the displayChannel method to
        % visually evaluate the resulting orientation of the array.
        ReceiveArrayOrientation;

        %SampleDensity Number of time samples per half wavelength
        %   Number of samples of filter coefficient generation per half 
        %   wavelength. The coefficient generation sampling rate is
        %   F_cg = (sum(MaximumDopplerShift) + (2 * (MaximumScattererSpeed/lambda0))) * 2 * SampleDensity
        %   where lambda0 is the carrier wavelength.
        %   Setting SampleDensity = Inf sets F_cg = SamplingRate.
        %
        %   The default value of this property is 64.
        SampleDensity;

        %SampleRate Input signal sample rate (Hz)
        %   Specify the sample rate of the input signal in Hz as a double
        %   precision, real, positive scalar.
        %
        %   The default value of this property is 30.72e6 Hz.
        SampleRate;

        %Seed Initial seed of mt19937ar random number stream
        %   Specify the initial seed of a mt19937ar random number generator
        %   algorithm as a double precision, real, nonnegative integer
        %   scalar. This property applies when you set the RandomStream
        %   property to 'mt19937ar with seed'. The Seed reinitializes the
        %   mt19937ar random number stream in the reset method.
        %
        %   The default value of this property is 73.
        Seed;

        %TransmitAndReceiveSwapped Transmit and receive antennas swapped (logical)
        %   This property indicates if the transmit and receive antennas in
        %   the channel are swapped. To toggle the state of this property,
        %   call the <a href="matlab:help nrCDLChannel/swapTransmitAndReceive"
        %   >swapTransmitAndReceive</a> method.
        TransmitAndReceiveSwapped;

        %TransmitAntennaArray Transmit antenna array characteristics
        %   Structure or Phased Array System Toolbox antenna array object
        %   specifying the transmit antenna array. The default is a
        %   structure containing the following fields:
        %   Size                - Size of antenna array [M,N,P,Mg,Ng]. M
        %                         and N are the number of rows and columns
        %                         in the antenna array. P is the number of
        %                         polarizations (1 or 2). Mg and Ng are the
        %                         number of row and column array panels
        %                         respectively. The defaults are 
        %                         [2,2,2,1,1].
        %   ElementSpacing      - Element spacing in wavelengths expressed
        %                         as [lambda_v lambda_h dg_v dg_h]
        %                         representing the vertical and horizontal
        %                         element spacing and the vertical and
        %                         horizontal panel spacing respectively.
        %                         The panel spacing is measured from the
        %                         center of the panels. The defaults are
        %                         [0.5 0.5 1.0 1.0].
        %   PolarizationAngles  - Polarization angles [theta rho] in
        %                         degrees applicable when P = 2. The
        %                         defaults are [45 -45] degrees.
        %   Orientation         - Mechanical orientation of the array
        %                         [alpha; beta; gamma] in degrees (bearing,
        %                         downtilt, slant). The default values [0;
        %                         0; 0] indicate that the broadside
        %                         direction of the array points to the
        %                         positive x-axis. Orientation will be
        %                         removed in a future release. Use the
        %                         TransmitArrayOrientation property
        %                         instead.
        %   Element             - Antenna element radiation pattern. One of
        %                         'isotropic' or '38.901' (see TR 38.901 
        %                         Section 7.3). The default value is 
        %                         '38.901'.
        %   PolarizationModel   - Model describing how to determine the
        %                         radiation field patterns based on a 
        %                         defined radiation power pattern (see
        %                         TR 38.901 Section 7.3.2). One of
        %                         'Model-1' or 'Model-2'. The default value
        %                         is 'Model-2'.
        %
        % The antenna array elements are mapped to the input waveform
        % channels (columns) in the order that a 5-D array of size
        % M-by-N-by-P-by-Mg-by-Ng is linearly indexed (across the
        % dimensions first to last). The size of the array is given by
        % TransmitAntennaArray.Size = [M,N,P,Mg,Ng]. For example, an
        % antenna array of size [4,8,2,2,2] has the first M = 4 channels
        % mapped to the first column of the first polarization angle of the
        % first panel. The next M (equals 4) antennas are mapped to the
        % next column and so on, such that the first M*N (equals 32)
        % channels are mapped to the first polarization angle of the
        % complete first panel. Then the next 32 channels are mapped in the
        % same fashion to the second polarization angle for the first
        % panel. Subsequent sets of M*N*P (equals 64) channels are then
        % mapped to the remaining panels, panel rows first then panel
        % columns.
        TransmitAntennaArray;

        %TransmitArrayOrientation Orientation of the transmit antenna array
        % Mechanical orientation of the transmit antenna array [alpha;
        % beta; gamma] in degrees (bearing, downtilt, slant). The default
        % values [0; 0; 0] indicate that the broadside direction of the
        % array points to the positive x-axis.
        % 
        % When TransmitAntennaArray is a Phased Array System Toolbox
        % antenna array object, [alpha; beta; gamma] specify three rotation
        % angles (bearing, downtilt, slant) applied to the array oriented
        % in the local coordinate system. Use the displayChannel method to
        % visually evaluate the resulting orientation of the array.
        TransmitArrayOrientation;

        %UTDirectionOfTravel User equipment (UE) direction of travel (deg)
        %   Specify the user equipment (UE) direction of travel in degrees as
        %   a double precision, real 2-by-1 column vector or 2-by-2 matrix of
        %   azimuth and zenith angles. When specified as a 
        %   2-by-1 vector of the form [azimuth; zenith], azimuth and zenith 
        %   specify the direction of travel of the receiver UE. When
        %   specified as a 2-by-2 matrix of the form [Rx_azimuth, Tx_azimuth; Rx_zenith, Tx_zenith], 
        %   Rx_azimuth and Rx_zenith specify the direction of travel of the 
        %   receiver UE, and Tx_azimuth and Tx_zenith specify the 
        %   direction of travel of the transmitter UE.
        %
        %   The default value of this property is [0; 90] degrees.
        UTDirectionOfTravel;

        %XPR Cross polarization power ratio (dB)
        %   Specify the cross-polarization power ratio in dB as a scalar or
        %   an N-by-M matrix, in which N is the number of clusters and M is
        %   the number of rays per cluster (M=20 rays). This property
        %   applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 10.0 dB. This is the
        %   value defined for delay profile CDL-A.
        XPR;

    end
end
