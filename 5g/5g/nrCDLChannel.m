classdef (StrictDefaults)nrCDLChannel < matlab.System
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
    

%#codegen

% =========================================================================
%   public interface

    methods (Access = public)
        
        % nrCDLChannel constructor
        function obj = nrCDLChannel(varargin)

            % Parse inputs for construction of custom CDL channels based on
            % the input InitialDelayProfile
            [customFromPreset,delayProfile,delaySpread,kFactor] = constructionOptions(varargin{:});

            if ~customFromPreset
                % Set property values from any name-value pairs input to the
                % constructor
                setProperties(obj,nargin,varargin{:});
            else
                % Create custom CDL channel with parsed options
                obj = nrCDLChannel.makeCustomCDL(delayProfile,delaySpread,kFactor);
            end

        end
        
        function h = getPathFilters(obj)
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
            
            if isempty(coder.target) && isLocked(obj)
                % Cache path filters only if the channel is locked.
                % Otherwise, a change in a property could result in
                % inconsistent path filters
                if isempty(obj.pathFilters)
                    obj.pathFilters = nrCDLChannel.makePathFilters(obj);
                end
                h = obj.pathFilters;
            else
                h = nrCDLChannel.makePathFilters(obj);
            end
            
        end
        
        function varargout = displayChannel(obj,varargin)
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
        
            if isLocked(obj)
                model = obj.theStruct;
            else
                validate(obj);
                model = nrCDLChannel.makeCDLChannelStructure(obj);
            end
            
            % The transmit and receive antenna arrays and departure and
            % arrival angles in 'model' are applicable to
            % TransmitAndReceiveSwapped = false, so if
            % TransmitAndReceiveSwapped = true, they are swapped before
            % visualization
            if (obj.TransmitAndReceiveSwapped)
                model = swapFields(model,'TransmitAntennaArray','ReceiveAntennaArray');
                model = swapFields(model,'AnglesAoD','AnglesAoA');
                model = swapFields(model,'AnglesZoD','AnglesZoA');
            end
            
            model.TransmitAntennaArray.Homogeneous = nrCDLChannel.isHomogeneous(obj.TransmitAntennaArray);
            [~,hasSlant] = wireless.internal.channelmodels.getCDLElementSlant(obj.TransmitAntennaArray);
            model.TransmitAntennaArray.HasElementSlant = hasSlant;
            
            model.ReceiveAntennaArray.Homogeneous = nrCDLChannel.isHomogeneous(obj.ReceiveAntennaArray);
            [~,hasSlant] = wireless.internal.channelmodels.getCDLElementSlant(obj.ReceiveAntennaArray);
            model.ReceiveAntennaArray.HasElementSlant = hasSlant;
            
            opts = nr5g.internal.parseOptions('nrCDLChannel',{'LinkEnd','Polarization','ElementPattern','ClusterPaths'},varargin{:});
            h = wireless.internal.channelmodels.displayChannel(model,opts);
            
            if nargout > 0
                varargout{1} = h;
            end
            
        end
        
        function swapTransmitAndReceive(obj)
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
        
            obj.TransmitAndReceiveSwapped = ~obj.TransmitAndReceiveSwapped;
            
            % reset relevant channel filter
            if (~obj.TransmitAndReceiveSwapped)
                reset(obj.channelFilter);
            else
                reset(obj.channelFilterReciprocal);
            end
        
        end
        
    end
    
    properties (Access = public, Nontunable)
        
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
        DelayProfile = 'CDL-A';
        
    end

    properties (Access = public)

        %PathDelays Discrete path delay vector (s)
        %   Specify the delays of the discrete paths in seconds as a
        %   double-precision, real, scalar or row vector. This property
        %   applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0. 
        PathDelays = 0.0;
        
        %AveragePathGains Average path gain vector (dB)
        %   Specify the average gains of the discrete paths in decibels as
        %   a double-precision, real, scalar or row vector. The average
        %   path gains are also referred to as cluster powers in TR 38.901.
        %   AveragePathGains must have the same size as PathDelays. This
        %   property applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0. 
        AveragePathGains = 0.0;
        
    end
    
    properties (Access = public, Dependent)
        
        %AnglesAoD Azimuth of departure angles vector (deg)
        %   Specify the azimuth of departure angle for each cluster in
        %   degrees as a double-precision, real, scalar or row vector. This
        %   property applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0. 
        AnglesAoD;
        
        %AnglesAoA Azimuth of arrival angles vector (deg)
        %   Specify the azimuth of arrival angle for each cluster in
        %   degrees as a double-precision, real, scalar or row vector. This
        %   property applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0.
        AnglesAoA;
        
        %AnglesZoD Zenith of departure angles vector (deg)
        %   Specify the zenith of departure angle for each cluster in
        %   degrees as a double-precision, real, scalar or row vector. This
        %   property applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0.
        AnglesZoD;
        
        %AnglesZoA Zenith of arrival angles vector (deg)
        %   Specify the zenith of arrival angle for each cluster in degrees
        %   as a double-precision, real, scalar or row vector. This
        %   property applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0. 
        AnglesZoA;
        
    end
    
    properties (Access = public, Nontunable)
        
        %HasLOSCluster Line of sight cluster (logical)
        %   Set this property to true to specify that the delay profile has
        %   a line of sight (LOS) cluster. This property applies when
        %   DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is false. 
        HasLOSCluster (1, 1) logical = false;
        
        %KFactorFirstCluster K-factor of first cluster (dB)
        %   Specify the K-factor of the first cluster of the delay profile
        %   in dB (K_1) as a scalar. This property applies when
        %   DelayProfile is set to 'Custom' and HasLOSCluster is set to
        %   true.
        %
        %   The default value of this property is 13.3 dB. This is the
        %   value defined for delay profile CDL-D.
        KFactorFirstCluster = 13.3;
        
        %AngleScaling Enable angle scaling (logical)
        %   Set this property to true to apply scaling of angles as
        %   described in TR 38.901 Section 7.7.5.1. This property applies
        %   when you set the DelayProfile property to 'CDL-A', 'CDL-B',
        %   'CDL-C', 'CDL-D' or 'CDL-E'. When you set this property to 
        %   true, the desired angle scaling is set using the AngleSpreads 
        %   and MeanAngles properties.
        %
        %   The default value of this property is false.
        AngleScaling (1, 1) logical = false;
        
    end
    
    properties (Access = public, Dependent, Nontunable)
        
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

    end

    properties (Access = public, Nontunable)

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
        RayCoupling = 'Random';
        
        %XPR Cross polarization power ratio (dB)
        %   Specify the cross-polarization power ratio in dB as a scalar or
        %   an N-by-M matrix, in which N is the number of clusters and M is
        %   the number of rays per cluster (M=20 rays). This property
        %   applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 10.0 dB. This is the
        %   value defined for delay profile CDL-A.
        XPR = 10.0;

    end

    properties (Access = public)

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
        InitialPhases = 'Random';
        
    end

    properties (Access = public, Nontunable)
        
        %DelaySpread Desired delay spread (s)
        %   Specify the desired RMS delay spread in seconds (DS_desired) as
        %   a scalar. See TR 38.901 Section 7.7.3, and Tables 7.7.3-1 and
        %   7.7.3-2 for examples of desired RMS delay spreads. This
        %   property applies when you set the DelayProfile property to
        %   'CDL-A', 'CDL-B', 'CDL-C', 'CDL-D' or 'CDL-E'.
        %
        %   The default value of this property is 30e-9.
        DelaySpread = 30e-9;
        
        %CarrierFrequency Carrier frequency (Hz)
        %   Specify the carrier frequency in Hertz as a scalar.
        %   
        %   The default value of this property is 4 GHz.
        CarrierFrequency = 4e9;

    end

    properties (Access = public, Dependent)
        
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

    end

    properties(Access = public, Nontunable)
        
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
        MovingScattererProportion = 0.2;

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
        MaximumScattererSpeed = 5;

        %KFactorScaling Apply K-factor scaling (logical)
        %   Set this property to true to apply K-factor scaling as
        %   described in TR 38.901 Section 7.7.6. Note that K-factor
        %   scaling modifies both the path delays and path powers. This
        %   property applies if DelayProfile is set to 'CDL-D' or 'CDL-E'.
        %   When you set this property to true, the desired K-factor is set
        %   using the KFactor property.
        %
        %   The default value of this property is false.
        KFactorScaling (1, 1) logical = false;
        
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
        KFactor = 9.0;
        
        %SampleRate Input signal sample rate (Hz)
        %   Specify the sample rate of the input signal in Hz as a double
        %   precision, real, positive scalar.
        %
        %   The default value of this property is 30.72e6 Hz.
        SampleRate = 30.72e6;
        
    end
        
    properties (Access = public, Dependent, Nontunable)
        
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

    end

    properties (Access = public, Dependent)
        
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

    end

    properties (Access = public, Dependent, Nontunable)
        
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

    end

    properties (Access = public, Dependent)
        
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
        
    end
        
    properties (Access = public, Nontunable)
            
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
        NormalizePathGains (1, 1) logical = true;
        
        %SampleDensity Number of time samples per half wavelength
        %   Number of samples of filter coefficient generation per half 
        %   wavelength. The coefficient generation sampling rate is
        %   F_cg = (sum(MaximumDopplerShift) + (2 * (MaximumScattererSpeed/lambda0))) * 2 * SampleDensity
        %   where lambda0 is the carrier wavelength.
        %   Setting SampleDensity = Inf sets F_cg = SamplingRate.
        %
        %   The default value of this property is 64.
        SampleDensity = 64;
        
    end
        
    properties (Access = public)
            
        %InitialTime Start time of fading process (s)
        %   Specify the time offset of the fading process as a real
        %   nonnegative scalar. This property is tunable. 
        %
        %   The default value of this property is 0.0.
        InitialTime = 0.0;
        
    end
    
    properties (Access = public, Nontunable)  
        
        %NumStrongestClusters Number of strongest clusters to split into subclusters
        %   The number of strongest clusters to split into sub-clusters.
        %   See TR 38.901 Section 7.5 step 11. This property applies when
        %   DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.
        NumStrongestClusters = 0;
        
        %ClusterDelaySpread Cluster delay spread (s)
        %   Specify the cluster delay spread (C_DS) as a real nonnegative
        %   scalar in seconds. The value is used to specify the delay
        %   offset between sub-clusters for clusters split into
        %   sub-clusters. See TR 38.901 Section 7.5 step 11. This property
        %   applies when DelayProfile is set to 'Custom' and
        %   NumStrongestClusters is greater than zero.
        %
        %   The default value of this property is 3.90625ns.
        ClusterDelaySpread = 5.0/1.28 * 1e-9;
        
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
        RandomStream = 'mt19937ar with seed';
        
        %Seed Initial seed of mt19937ar random number stream
        %   Specify the initial seed of a mt19937ar random number generator
        %   algorithm as a double precision, real, nonnegative integer
        %   scalar. This property applies when you set the RandomStream
        %   property to 'mt19937ar with seed'. The Seed reinitializes the
        %   mt19937ar random number stream in the reset method.
        %
        %   The default value of this property is 73. 
        Seed = 73;
        
        %NormalizeChannelOutputs Normalize channel outputs by the number of receive antennas (logical)
        %   Set this property to true to normalize the channel outputs by
        %   the number of receive antennas. When you set this property to
        %   false, there is no normalization for channel outputs.
        %
        %   The default value of this property is true.
        NormalizeChannelOutputs (1, 1) logical = true;
        
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
        ChannelFiltering (1, 1) logical = true;
        
    end
        
    properties (Access = public)
        %NumTimeSamples Number of time samples
        %   Specify the number of time samples used to set the duration of
        %   the fading process realization as a positive integer scalar.
        %   This property applies when ChannelFiltering is false. This
        %   property is tunable.
        %
        %   The default value of this property is 30720.
        NumTimeSamples = 30720;
        
    end
    
    properties (Access = public, Nontunable)
        %OutputDataType Path gain output data type
        %   Specify the path gain output data type as one of 'double' or
        %   'single'. This property applies when ChannelFiltering is false.
        % 
        %   The default value of this property is 'double'.
        OutputDataType = 'double';
        
    end
    
    properties(GetAccess = public, SetAccess = private)        
        %TransmitAndReceiveSwapped Transmit and receive antennas swapped (logical)
        %   This property indicates if the transmit and receive antennas in
        %   the channel are swapped. To toggle the state of this property,
        %   call the <a href="matlab:help nrCDLChannel/swapTransmitAndReceive"
        %   >swapTransmitAndReceive</a> method.
        TransmitAndReceiveSwapped (1, 1) logical = false;
        
    end
    
    properties(Hidden, Access = public, Nontunable)

        UseGPU (1, 1) logical = false;
        
    end
        
    properties (Access = public, Dependent, Nontunable)
        %ChannelResponseOutput Specify the type of returned channel response
        %   When this property is set to 'path-gains', the step method
        %   returns the channel path gains and the corresponding sample
        %   times. Set this property to 'ofdm-response' to make the step
        %   method return the channel OFDM response and the timing offset.
        %
        %   The default value of this property is 'path-gains'.
        ChannelResponseOutput;
    end
    
    % public property setters for validation
    methods
        
        function set.PathDelays(obj,val)
            propName = 'PathDelays';
            validateattributes(val,{'double'},{'real','row','finite'},[class(obj) '.' propName],propName);
            if (isempty(coder.target))
                recordPropertyTuning(obj,propName);
            end
            obj.PathDelays = val;
        end
        
        function set.AveragePathGains(obj,val)
            propName = 'AveragePathGains';
            validateattributes(val,{'double'},{'real','row','finite'},[class(obj) '.' propName], propName);
            if (isempty(coder.target))
                recordPropertyTuning(obj,propName);
            end
            obj.AveragePathGains = val;
        end
        
        function set.AnglesAoD(obj,val)
            propName = 'AnglesAoD';
            validateattributes(val,{'double'},{'real','row','finite'},[class(obj) '.' propName], propName);
            if (isempty(coder.target))
                recordPropertyTuning(obj,propName);
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theAnglesAoD = val;
                else
                    obj.theAnglesAoA = val;
                end
            else
                obj.theAnglesAoD = val;
            end
        end
        
        function set.AnglesAoA(obj,val)
            propName = 'AnglesAoA';
            validateattributes(val,{'double'},{'real','row','finite'},[class(obj) '.' propName], propName);
            if (isempty(coder.target))
                recordPropertyTuning(obj,propName);
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theAnglesAoA = val;
                else
                    obj.theAnglesAoD = val;
                end
            else
                obj.theAnglesAoA = val;
            end
        end
        
        function set.AnglesZoD(obj,val)
            propName = 'AnglesZoD';
            validateattributes(val,{'double'},{'real','row','finite'},[class(obj) '.' propName], propName);
            if (isempty(coder.target))
                recordPropertyTuning(obj,propName);
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theAnglesZoD = val;
                else
                    obj.theAnglesZoA = val;
                end
            else
                obj.theAnglesZoD = val;
            end
        end
        
        function set.AnglesZoA(obj,val)
            propName = 'AnglesZoA';
            validateattributes(val,{'double'},{'real','row','finite'},[class(obj) '.' propName], propName);
            if (isempty(coder.target))
                recordPropertyTuning(obj,propName);
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theAnglesZoA = val;
                else
                    obj.theAnglesZoD = val;
                end
            else
                obj.theAnglesZoA = val;
            end
        end        
        
        function set.KFactorFirstCluster(obj,val)
            propName = 'KFactorFirstCluster';
            validateKFactor(val,[class(obj) '.' propName],propName);
            obj.KFactorFirstCluster = val;
        end
        
        function set.AngleSpreads(obj,val)
            propName = 'AngleSpreads';
            validateattributes(val,{'double'},{'real','size',[1 4],'finite'},[class(obj) '.' propName], propName);
            if (isempty(coder.target))
                if (obj.TransmitAndReceiveSwapped)
                    val = val([2 1 4 3]);
                end
            end
            obj.theAngleSpreads = val;
        end
        
        function set.MeanAngles(obj,val)
            propName = 'MeanAngles';
            validateattributes(val,{'double'},{'real','size',[1 4],'finite'},[class(obj) '.' propName], propName);
            if (isempty(coder.target))
                if (obj.TransmitAndReceiveSwapped)
                    val = val([2 1 4 3]);
                end
            end
            obj.theMeanAngles = val;
        end

        function set.RayCoupling(obj,val)
            propName = 'RayCoupling';
            if ischar(val) || isstring(val)
                valTmp = validatestring(val, {'Random'},[class(obj) '.' propName],propName);
            else
                validateattributes(val,{'numeric'},{'real','integer','positive','nonempty'},[class(obj) '.' propName],propName);
                valTmp = val;
            end
            obj.RayCoupling = valTmp;
        end
        
        function set.XPR(obj,val)
            propName = 'XPR';
            validateattributes(val,{'double'},{'real','2d','nonempty'},[class(obj) '.' propName],propName);
            obj.XPR = val;
        end

        function set.InitialPhases(obj,val)
            propName = 'InitialPhases';
            if ischar(val) || isstring(val)
                valTmp = validatestring(val, {'Random'},[class(obj) '.' propName],propName);
            elseif isscalar(val)
                validateattributes(val,{'numeric'},{'real'},[class(obj) '.' propName],propName);
                valTmp = val;
            else
                validateattributes(val,{'numeric'},{'real','nonempty'},[class(obj) '.' propName],propName);
                valTmp = val;
            end
            if (isempty(coder.target))
                recordPropertyTuning(obj,propName);
            end
            obj.InitialPhases = valTmp;
        end
        
        function set.DelaySpread(obj,val)
            propName = 'DelaySpread';
            validateDelaySpread(val,[class(obj) '.' propName],propName);
            obj.DelaySpread = val;
        end
        
        function set.CarrierFrequency(obj,val)
            propName = 'CarrierFrequency';
            validateattributes(val,{'double'},{'real','scalar','nonnegative','finite'},[class(obj) '.' propName],propName);
            obj.CarrierFrequency = val;
        end
                
        function set.MaximumDopplerShift(obj,val)
            propName = 'MaximumDopplerShift';
            validateattributes(val,{'double'},{'real','nonnegative','2d','finite'},[class(obj) '.' propName],propName);
            coder.internal.errorIf(any(size(val) ~= [1 1]) && any(size(val) ~= [1 2]),'nr5g:nrCDLChannel:MaximumDopplerShiftSize','MaximumDopplerShift',size(val,1),size(val,2))
            if (isempty(coder.target))
                recordPropertyTuning(obj,propName);
                if ~isscalar(val)
                    if (obj.TransmitAndReceiveSwapped)
                        val = val([2 1]);
                    end
                end
            end
            obj.theMaximumDopplerShift = val;
        end
        
        function set.UTDirectionOfTravel(obj,val)
            propName = 'UTDirectionOfTravel';
            validateattributes(val,{'double'},{'real','2d','finite'},[class(obj) '.' propName], propName);
            coder.internal.errorIf(any(size(val) ~= [2 1]) && any(size(val) ~= [2 2]),'nr5g:nrCDLChannel:UTDirectionOfTravelSize','UTDirectionOfTravel',size(val,1),size(val,2))
            if (isempty(coder.target))
                recordPropertyTuning(obj,propName);
                if ~isvector(val)
                    if (obj.TransmitAndReceiveSwapped)
                        val = val([3 1;4 2]);
                    end
                end
            end
            obj.theUTDirectionOfTravel = val;
        end

        function set.MovingScattererProportion(obj,val)
            propName = 'MovingScattererProportion';
            validateattributes(val,{'double'},{'real','nonnegative','scalar','finite','<=',1},[class(obj) '.' propName],propName);
            obj.MovingScattererProportion = val;
        end
        
        function set.MaximumScattererSpeed(obj,val)
            propName = 'MaximumScattererSpeed';
            validateattributes(val,{'double'},{'real','nonnegative','scalar','finite'},[class(obj) '.' propName],propName);
            obj.MaximumScattererSpeed = val;
        end
        
        function set.KFactor(obj,val)
            propName = 'KFactor';
            validateKFactor(val,[class(obj) '.' propName],propName);
            obj.KFactor = val;
        end

        function set.TransmitAntennaArray(obj,val)
            txArray = nrCDLChannel.validateAntennaArray(val,class(obj),'TransmitAntennaArray');
            oprop = 'TransmitArrayOrientation';
            if isstruct(txArray) && isfield(txArray,'Orientation') && any(txArray.Orientation ~= obj.(oprop))
                setSwappedTransmitArrayOrientation(obj,txArray.Orientation,oprop);
                coder.internal.warning('nr5g:nrCDLChannel:OrientationToBeRemoved','TransmitAntennaArray',oprop);
            end
            setSwappedTransmitAntennaArray(obj,txArray);
        end
            
        function set.ReceiveAntennaArray(obj,val)
            rxArray = nrCDLChannel.validateAntennaArray(val,class(obj),'ReceiveAntennaArray');
            oprop = 'ReceiveArrayOrientation';
            if isstruct(rxArray) && isfield(rxArray,'Orientation') && any(rxArray.Orientation ~= obj.(oprop))
                setSwappedReceiveArrayOrientation(obj,rxArray.Orientation,oprop);
                coder.internal.warning('nr5g:nrCDLChannel:OrientationToBeRemoved','ReceiveAntennaArray',oprop);
            end
            setSwappedReceiveAntennaArray(obj,rxArray);
        end
            
        function set.SampleRate(obj,val)
            propName = 'SampleRate';
            validateattributes(val,{'double'},{'real','scalar','positive','finite'},[class(obj) '.' propName],propName);
            obj.SampleRate = val;
        end
        
        function set.SampleDensity(obj,val)
            propName = 'SampleDensity';
            validateattributes(val,{'double'},{'real','scalar','positive'},[class(obj) '.' propName],propName);
            obj.SampleDensity = val;
        end
                
        function set.InitialTime(obj,val)
            propName = 'InitialTime';
            validateattributes(val,{'double'},{'real','scalar','nonnegative','finite'},[class(obj) '.' propName],propName);
            obj.InitialTime = val;
        end
        
        function set.NumStrongestClusters(obj,val)
            propName = 'NumStrongestClusters';
            validateattributes(val,{'numeric'},{'scalar','integer','nonnegative'},[class(obj) '.' propName],propName);
            obj.NumStrongestClusters = val;
        end
        
        function set.ClusterDelaySpread(obj,val)
            propName = 'ClusterDelaySpread';
            validateattributes(val,{'double'},{'real','scalar','nonnegative','finite'},[class(obj) '.' propName],propName);
            obj.ClusterDelaySpread = val;
        end
                
        function set.Seed(obj,val)
            propName = 'Seed';
            validateattributes(val,{'double'},{'real','scalar','integer','nonnegative','finite'},[class(obj) '.' propName],propName);
            obj.Seed = val;
        end
        
        function set.NumTimeSamples(obj,val)
            propName = 'NumTimeSamples';
            validateattributes(val,{'numeric'},{'scalar','integer','positive'},[class(obj) '.' propName],propName);
            obj.NumTimeSamples = val;
        end
        
        function set.ChannelResponseOutput(obj,val)
            if strcmp(val,'ofdm-response')
                obj.OutputOFDMResponse = true;
            else
                obj.OutputOFDMResponse = false;
            end
        end

        function set.TransmitArrayOrientation(obj,val)
            propName = 'TransmitArrayOrientation';
            validateattributes(val,{'numeric'},{'real','size',[3 1],'finite'},[class(obj) '.' propName],propName);
            setSwappedTransmitArrayOrientation(obj,val,propName);
            if isstruct(obj.TransmitAntennaArray) && isfield(obj.TransmitAntennaArray,'Orientation')
                setSwappedTransmitAntennaArray(obj,setfield(obj.TransmitAntennaArray,'Orientation',val));
            end
        end
        
        function set.ReceiveArrayOrientation(obj,val)
            propName = 'ReceiveArrayOrientation';
            validateattributes(val,{'numeric'},{'real','size',[3 1],'finite'},[class(obj) '.' propName],propName);
            setSwappedReceiveArrayOrientation(obj,val,propName);
            if isstruct(obj.ReceiveAntennaArray) && isfield(obj.ReceiveAntennaArray,'Orientation')
                setSwappedReceiveAntennaArray(obj,setfield(obj.ReceiveAntennaArray,'Orientation',val));
            end
        end
        
    end
        
    % property value sets for enumerated properties
    properties(Hidden,Transient)
        
        DelayProfileSet = matlab.system.StringSet({'CDL-A','CDL-B','CDL-C','CDL-D','CDL-E','Custom'});
        RandomStreamSet = matlab.system.StringSet({'Global stream','mt19937ar with seed'});
        OutputDataTypeSet = matlab.system.StringSet({'double','single'});
        ChannelResponseOutputSet = matlab.system.StringSet({'path-gains','ofdm-response'});
        
    end
    
    % public property getters
    methods
        
        function val = get.AnglesAoD(obj)
            val = nrCDLChannel.swapTxRx(obj.theAnglesAoD,obj.theAnglesAoA,obj.TransmitAndReceiveSwapped);
        end
        
        function val = get.AnglesAoA(obj)
            val = nrCDLChannel.swapTxRx(obj.theAnglesAoA,obj.theAnglesAoD,obj.TransmitAndReceiveSwapped);
        end

        function val = get.AnglesZoD(obj)
            val = nrCDLChannel.swapTxRx(obj.theAnglesZoD,obj.theAnglesZoA,obj.TransmitAndReceiveSwapped);
        end

        function val = get.AnglesZoA(obj)
            val = nrCDLChannel.swapTxRx(obj.theAnglesZoA,obj.theAnglesZoD,obj.TransmitAndReceiveSwapped);
        end
        
        function val = get.AngleSpreads(obj)
            val =  obj.theAngleSpreads;
            if (obj.TransmitAndReceiveSwapped)
                val = val([2 1 4 3]);
            end
        end
        
        function val = get.MeanAngles(obj)
            val =  obj.theMeanAngles;
            if (obj.TransmitAndReceiveSwapped)
                val = val([2 1 4 3]);
            end
        end
        
        function val = get.TransmitAntennaArray(obj)
            val = nrCDLChannel.swapTxRx(obj.theTransmitAntennaArray,obj.theReceiveAntennaArray,obj.TransmitAndReceiveSwapped);
        end
        
        function val = get.TransmitArrayOrientation(obj)
            val = nrCDLChannel.swapTxRx(obj.theTransmitArrayOrientation,obj.theReceiveArrayOrientation,obj.TransmitAndReceiveSwapped);
        end
        
        function val = get.ReceiveAntennaArray(obj)
            val = nrCDLChannel.swapTxRx(obj.theReceiveAntennaArray,obj.theTransmitAntennaArray,obj.TransmitAndReceiveSwapped);
        end
        
        function val = get.ReceiveArrayOrientation(obj)
            val = nrCDLChannel.swapTxRx(obj.theReceiveArrayOrientation,obj.theTransmitArrayOrientation,obj.TransmitAndReceiveSwapped);
        end

        function val = get.MaximumDopplerShift(obj)
            val =  obj.theMaximumDopplerShift;
            if ~isscalar(val) && (obj.TransmitAndReceiveSwapped)
                val = val([2 1]);
            end
        end

        function val = get.ChannelResponseOutput(obj)
            if obj.OutputOFDMResponse
                val = 'ofdm-response';
            else
                val = 'path-gains';
            end
        end

        function val = get.UTDirectionOfTravel(obj)
            val =  obj.theUTDirectionOfTravel;
            if ~isvector(val) && (obj.TransmitAndReceiveSwapped)
                val = val([3 1;4 2]);
            end
        end
        
    end
    
% =========================================================================
%   protected interface
    
    methods (Access = protected)
        
        % nrCDLChannel setupImpl method
        function setupImpl(obj,varargin)
            
            % Set up everything that can be influenced by the tunable
            % properties; the object is currently unlocked, so treat all
            % properties as if they have been tuned
            if (obj.ChannelFiltering)
                performSetup(obj,varargin{1});
            else
                performSetup(obj);
            end

            % Setup RNG
            setupRNG(obj);

        end
        
        % nrCDLChannel stepImpl method
        function varargout = stepImpl(obj,varargin)

            % Configure the input size and output data type. Note that the
            % number of antennas in 'insize', used by CDLChannel below, is
            % applicable to TransmitAndReceiveSwapped = false. If
            % TransmitAndReceiveSwapped = true, the path gains are
            % subsequently permuted to give the reciprocal channel

            % Possible number of inputs (in addition to system object)
            %   obj.step():           ChannelFiltering = false, ChannelResponseOutput = 'path-gains'
            %   obj.step(in):         ChannelFiltering = true,  ChannelResponseOutput = 'path-gains'
            %   obj.step(carrier):    ChannelFiltering = false, ChannelResponseOutput = 'ofdm-response'
            %   obj.step(in,carrier): ChannelFiltering = true,  ChannelResponseOutput = 'ofdm-response'
            switch nargin
                case 1 % step()
                    insize = [obj.NumTimeSamples obj.theInfo.NumInputSignals];
                    outputtype = obj.OutputDataType;
                    if ~(obj.UseGPU)
                        outputproto = zeros(0,0,outputtype);
                    else
                        outputproto = zeros(0,0,outputtype,'gpuArray');
                    end
                    numTimeSamples = obj.NumTimeSamples;
                case 2 % step(in), step(carrier)
                    if (obj.ChannelFiltering) % step(in)
                        in = varargin{1};
                        validateInputSignal(obj,in);
                        insize = [size(in,1) obj.theInfo.NumInputSignals];
                        numTimeSamples = size(in,1);
                        outputtype = underlyingType(in);
                        outputproto = zeros(0,0,'like',in);
                    else % step(carrier)
                        carrier = varargin{1};
                        validateattributes(carrier,{'nrCarrierConfig'},{'scalar'},'nrCDLChannel','Carrier specific configuration object');
                        validateCarrierSampleRate(obj,carrier);
                        insize = [obj.NumTimeSamples obj.theInfo.NumInputSignals];
                        numTimeSamples = obj.NumTimeSamples;
                        outputtype = obj.OutputDataType;
                        if ~(obj.UseGPU)
                            outputproto = zeros(0,0,outputtype);
                        else
                            outputproto = zeros(0,0,outputtype,'gpuArray');
                        end
                    end
                case 3 % step(in,carrier)
                    in = varargin{1};
                    validateInputSignal(obj,in);
                    insize = [size(in,1) obj.theInfo.NumInputSignals];
                    numTimeSamples = size(in,1);
                    outputtype = underlyingType(in);
                    outputproto = zeros(0,0,'like',in);
                    carrier = varargin{2};
                    validateattributes(carrier,{'nrCarrierConfig'},{'scalar'},'nrCDLChannel','Carrier specific configuration object');
                    validateCarrierSampleRate(obj,carrier);
            end

            if isa(outputproto,'gpuArray')
                obj.staticChannelInfo.StaticPathGains = gpuArray(obj.staticChannelInfo.StaticPathGains);
            end

            % Compute the time-varying cluster path gains
            [pathgains,sampletimes] = wireless.internal.channelmodels.timeVaryingCDLChannel(obj.theStruct,obj.staticChannelInfo,obj.movingScattererStates,obj.maximumScattererSpeeds,obj.theTime,insize,outputproto);
            
            % Update path gains to give reciprocal channel if required
            if (obj.TransmitAndReceiveSwapped)
                % Permute to swap transmit and receive antennas
                x = permute(pathgains,[1 2 4 3]);
                % Re-normalize channel outputs if required
                if (obj.NormalizeChannelOutputs)
                    % Normalization inside
                    % wireless.internal.channelmodels.staticCDLChannel uses
                    % the non-reciprocal receive antenna count, undo this
                    % and normalize by the reciprocal receive antenna count
                    g = x * sqrt(size(x,3)) / sqrt(size(x,4));
                else
                    g = x;
                end
            else
                g = pathgains;
            end
            
            % Apply channel filtering if required
            if (obj.ChannelFiltering)
                if (~obj.TransmitAndReceiveSwapped)
                    filterToUse = obj.channelFilter;
                else
                    filterToUse = obj.channelFilterReciprocal;
                end
                filterOption = 1 + filterToUse.OptimizeScalarGainFilter;
                out = wireless.internal.channelmodels.smartChannelFiltering(in,filterToUse,obj.SampleRate,g,insize,obj.SampleDensity,sampletimes-obj.theTime,outputtype,filterOption);
            end
            
            % Advance the time according to the input length, or
            % NumTimeSamples if channel filtering is disabled
            obj.theTime = obj.theTime + (insize(1) / obj.SampleRate);
            
            % Calculate frequency response
            if obj.OutputOFDMResponse % "ofdm-response" mode
                % A minimum of one slot is required for the OFDM
                % channel response
                validateNumTimeSamples(obj,numTimeSamples,carrier);

                % Synch
                pathfilters = getPathFilters(obj);
                toffset = channelDelay(g,pathfilters.');

                % Calculate the OFDM channel response at the center of each
                % OFDM symbol
                ofdminfo = nr5g.internal.OFDMInfo(carrier,[]);
                ofdmSymbolCenter = nr5g.internal.OFDMSampleTimesIndices(ofdminfo,carrier.NSlot,sampletimes,numTimeSamples,toffset);
                g = g(ofdmSymbolCenter,:,:,:);
                hgrid = nr5g.internal.OFDMChannelResponse(ofdminfo,g,pathfilters,toffset);

                out2 = hgrid;
                out3 = toffset;
            else
                out2 = g;
                out3 = sampletimes;
            end

            % Outputs
            if (obj.ChannelFiltering)
                varargout = {out out2 out3};
            else
                varargout = {out2 out3};
            end
        end

        % nrCDLChannel resetImpl method
        function resetImpl(obj)

            % reset the time to the last InitialTime property value set
            obj.theTime = obj.InitialTime;
            
            % reset channel filters
            if (obj.ChannelFiltering)
                reset(obj.channelFilter);
                reset(obj.channelFilterReciprocal);
            end
            
            % reset RNG
            resetRNG(obj);
            
            % reset everything that can be influenced by the tunable
            % properties; the object is currently unlocked, so treat all
            % properties as if they have been tuned
            performReset(obj);

        end
        
        % nrCDLChannel releaseImpl method
        function releaseImpl(obj)
            if coder.target('MATLAB')
                % Empty initialization may result in a size mismatch error
                % when codegen'ing if any of these fields are fixed size.
                % Additionally, codegen requires that any field referenced
                % in releaseImpl is definitely set in setupImpl, which is
                % not always the case.
                obj.pdp = [];
                obj.initialPhases = [];
                obj.rayCoupling = [];
                obj.movingScattererStates = [];
                obj.maximumScattererSpeeds = [];

                % Empty path filters as they may not be applicable after
                % changing a property and they would need to be regenerated
                obj.pathFilters = [];
            end
            if (obj.ChannelFiltering)
                release(obj.channelFilter);
                release(obj.channelFilterReciprocal);
            end
            
            if ~isstruct(obj.TransmitAntennaArray)
                release(obj.TransmitAntennaArray);
            end
            if ~isstruct(obj.ReceiveAntennaArray)
                release(obj.ReceiveAntennaArray);
            end
        
        end
        
        % nrCDLChannel getNumInputsImpl method
        function num = getNumInputsImpl(obj)
            num = obj.ChannelFiltering+obj.OutputOFDMResponse;
        end
        
        % nrCDLChannel getNumOutputsImpl method
        function num = getNumOutputsImpl(obj)
            num = 2 + obj.ChannelFiltering;            
        end

        % nrCDLChannel isInputComplexityMutableImpl method
        function flag = isInputComplexityMutableImpl(~,~)
            flag = true;            
        end

        % nrCDLChannel isInputSizeMutableImpl method
        function flag = isInputSizeMutableImpl(~,~)
            flag = true;
        end
        
        % nrCDLChannel infoImpl method
        function s = infoImpl(obj)
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
            
            doValidation = true;
            s = getInfo(obj,doValidation);
            
        end
        
         % nrCDLChannel saveObjectImpl method
        function s = saveObjectImpl(obj)
            
            s = saveObjectImpl@matlab.System(obj);
            s.theMeanAngles = obj.theMeanAngles;
            s.theAngleSpreads = obj.theAngleSpreads;
            s.theReceiveArrayOrientation = obj.theReceiveArrayOrientation;
            s.theTransmitArrayOrientation = obj.theTransmitArrayOrientation;

            % Save Rx phased array if needed
            if isobject(obj.theReceiveAntennaArray)
                s.theReceiveAntennaArray = saveobj(obj.theReceiveAntennaArray);
            else
                s.theReceiveAntennaArray = obj.theReceiveAntennaArray;
            end

            % Save Tx phased array if needed
            if isobject(obj.theTransmitAntennaArray)
                s.theTransmitAntennaArray = saveobj(obj.theTransmitAntennaArray);
            else
                s.theTransmitAntennaArray = obj.theTransmitAntennaArray;
            end

            s.theAnglesZoA = obj.theAnglesZoA;
            s.theAnglesZoD = obj.theAnglesZoD;
            s.theAnglesAoA = obj.theAnglesAoA;
            s.theAnglesAoD = obj.theAnglesAoD;
            s.theMaximumDopplerShift = obj.theMaximumDopplerShift;
            s.theUTDirectionOfTravel = obj.theUTDirectionOfTravel;
            s.theStruct = obj.theStruct;
            s.theInfo = obj.theInfo;
            s.randomStream = obj.randomStream;
            s.pdp = obj.pdp;
            s.initialPhases = obj.initialPhases;
            s.rayCoupling = obj.rayCoupling;
            s.movingScattererStates = obj.movingScattererStates;
            s.maximumScattererSpeeds = obj.maximumScattererSpeeds;
            s.theTime = obj.theTime;
            if (obj.ChannelFiltering)
                s.channelFilter = matlab.System.saveObject(obj.channelFilter);
                s.channelFilterReciprocal = matlab.System.saveObject(obj.channelFilterReciprocal);
            else
                s.channelFilter = obj.channelFilter;
                s.channelFilterReciprocal = obj.channelFilterReciprocal;
            end
            s.TransmitAndReceiveSwapped = obj.TransmitAndReceiveSwapped;
            s.pathFilters = obj.pathFilters;
            s.OutputOFDMResponse = obj.OutputOFDMResponse;
            
        end

        % nrCDLChannel loadObjectImpl method
        function loadObjectImpl(obj,s,wasLocked)
            
            % Handling backwards compatibility of array orientation
            if ~isfield(s,'theTransmitArrayOrientation')
                obj.theTransmitArrayOrientation = s.TransmitAntennaArray.Orientation;
                coder.internal.warning('nr5g:nrCDLChannel:OrientationToBeRemoved','TransmitAntennaArray','TransmitArrayOrientation');
            else
                obj.theTransmitArrayOrientation = s.theTransmitArrayOrientation;
            end
            if ~isfield(s,'theReceiveArrayOrientation')
                obj.theReceiveArrayOrientation = s.ReceiveAntennaArray.Orientation;
                coder.internal.warning('nr5g:nrCDLChannel:OrientationToBeRemoved','ReceiveAntennaArray','ReceiveArrayOrientation');
            else
                obj.theReceiveArrayOrientation = s.theReceiveArrayOrientation;
            end

            if s.ChannelFiltering
                % Instantiate channel filter objects if saved prior to 20b
                if ~isfield(s,'channelFilter') || ...
                       (isfield(s.channelFilter, 'ClassNameForLoadTimeEval') && ...
                       strcmp(s.channelFilter.ClassNameForLoadTimeEval, ...
                       'comm.internal.channel.ChannelFilter'))
                    if (wasLocked)
                        pathDelays = nr5g.internal.nrCDLChannel.getPathDelays(s.theStruct);
                        optimizeScalarGainFilter = false;
                        obj.channelFilter = constructChannelFilter(pathDelays,obj.channelFilterDelay,obj.stopbandAttenuation,s.SampleRate,optimizeScalarGainFilter);
                        obj.channelFilterReciprocal = clone(obj.channelFilter);
                    end
                else
                    obj.channelFilter = matlab.System.loadObject(s.channelFilter);
                    if isfield(s,'TransmitAndReceiveSwapped')
                        % TransmitAndReceiveSwapped property and associated
                        % private properties were added in 21a
                        obj.channelFilterReciprocal = matlab.System.loadObject(s.channelFilterReciprocal);
                    end
                end
            else
                if isfield(s,'channelFilter')
                    obj.channelFilter = s.channelFilter;
                end
                if isfield(s,'TransmitAndReceiveSwapped')
                    % TransmitAndReceiveSwapped property and associated
                    % private properties were added in 21a
                    obj.channelFilterReciprocal = s.channelFilterReciprocal;
                end
            end
            
            obj.theTime = s.theTime;
            obj.rayCoupling = s.rayCoupling;
            obj.initialPhases = s.initialPhases;
            obj.pdp = s.pdp;
            obj.randomStream = s.randomStream;

            if (isfield(s,'theMaximumDopplerShift'))
                % Dual mobility properties were added in 23b
                obj.theMaximumDopplerShift = s.theMaximumDopplerShift;
                obj.theUTDirectionOfTravel = s.theUTDirectionOfTravel;
                obj.movingScattererStates = s.movingScattererStates;
                obj.maximumScattererSpeeds = s.maximumScattererSpeeds;
            end

            if (isfield(s,'TransmitAndReceiveSwapped'))
                % TransmitAndReceiveSwapped property and associated private
                % properties were added in 21a
                obj.theAnglesAoD = s.theAnglesAoD;
                obj.theAnglesAoA = s.theAnglesAoA;
                obj.theAnglesZoD = s.theAnglesZoD;
                obj.theAnglesZoA = s.theAnglesZoA;

                % Load Tx phased array object if needed
                if isfield(s.theTransmitAntennaArray,'ClassNameForLoadTimeEval')
                    % s is a struct of a phased object, set it as an object
                    obj.theTransmitAntennaArray = eval(sprintf('%s.loadobj(s.theTransmitAntennaArray)', ...
                        s.theTransmitAntennaArray.ClassNameForLoadTimeEval));
                else
                    obj.theTransmitAntennaArray = s.theTransmitAntennaArray;
                end

                % Load Rx phased array object if needed
                if isfield(s.theReceiveAntennaArray,'ClassNameForLoadTimeEval')
                    % s is a struct of a phased object, set it as an object
                    obj.theReceiveAntennaArray = eval(sprintf('%s.loadobj(s.theReceiveAntennaArray)', ...
                        s.theReceiveAntennaArray.ClassNameForLoadTimeEval));
                else
                    obj.theReceiveAntennaArray = s.theReceiveAntennaArray;
                end

                obj.TransmitAndReceiveSwapped = s.TransmitAndReceiveSwapped;
                obj.theAngleSpreads = s.theAngleSpreads;
                obj.theMeanAngles = s.theMeanAngles;
            end
            
            if isfield(s,'pathFilters')
                % pathFilters added in 24b
                obj.pathFilters = s.pathFilters;
            else
                obj.pathFilters = [];
            end

            if isfield(s,'OutputOFDMResponse')
                % OutputOFDMResponse added in 24b
                obj.OutputOFDMResponse = s.OutputOFDMResponse;
            else
                obj.OutputOFDMResponse = false;
            end

            loadObjectImpl@matlab.System(obj,s,wasLocked);

            % If the channel was in use (locked), the next call to step()
            % won't run setupImpl and some derived properties need to be
            % initialized.
            if wasLocked
                obj.theStruct = nrCDLChannel.makeCDLChannelStructure(obj);
                obj.theInfo = wireless.internal.channelmodels.CDLChannelInfo(obj.theStruct);

                % Dual mobility was added in 23b
                if ~(isfield(s,'movingScattererStates')) || ~(isfield(s,'maximumScattererSpeeds'))
                    [obj.movingScattererStates,obj.maximumScattererSpeeds] = wireless.internal.channelmodels.getDualMobilityScattererVariables(obj.theStruct,obj.randomStream,obj.theInfo.ClusterTypes);
                end

                % Static path gains caching was added in 24a
                if ~(isfield(s,'staticPathGains'))
                    obj.staticChannelInfo = wireless.internal.channelmodels.staticCDLChannel(obj.theStruct,obj.theInfo,obj.pdp,obj.rayCoupling,obj.initialPhases);
                end
            end
            
        end
       
        % nrCDLChannel isInactivePropertyImpl method
        function flag = isInactivePropertyImpl(obj,prop)
            
            if (any(strcmp(prop,{'PathDelays','AveragePathGains','AnglesAoD','AnglesAoA','AnglesZoD','AnglesZoA','HasLOSCluster','XPR','NumStrongestClusters','RayCoupling','InitialPhases'})))
                flag = ~strcmp(obj.DelayProfile,'Custom');
            elseif (strcmp(prop,'KFactorFirstCluster'))
                flag = ~strcmp(obj.DelayProfile,'Custom') || ~obj.HasLOSCluster;
            elseif (strcmp(prop,'AngleSpreads'))
                flag = ~strcmp(obj.DelayProfile,'Custom') && ~obj.AngleScaling;
            elseif (strcmp(prop,'AngleScaling'))
                flag = strcmp(obj.DelayProfile,'Custom');
            elseif (strcmp(prop,'MeanAngles'))
                flag = strcmp(obj.DelayProfile,'Custom') || ~obj.AngleScaling;
            elseif (strcmp(prop,'KFactorScaling'))
                flag = ~any(strcmp(obj.DelayProfile,{'CDL-D','CDL-E'}));
            elseif (strcmp(prop,'KFactor'))
                flag = strcmp(obj.DelayProfile,'Custom') || ~(obj.KFactorScaling);
            elseif (strcmp(prop,'DelaySpread'))
                flag = strcmp(obj.DelayProfile,'Custom');                
            elseif (strcmp(prop,'Seed'))
                flag = ~strcmp(obj.RandomStream,'mt19937ar with seed');
            elseif (strcmp(prop,'ClusterDelaySpread'))
                flag = ~strcmp(obj.DelayProfile,'Custom') || ~obj.NumStrongestClusters;
            elseif (any(strcmp(prop,{'NumTimeSamples','OutputDataType','UseGPU'})))
                flag = obj.ChannelFiltering;
            elseif (any(strcmp(prop,{'MovingScattererProportion','MaximumScattererSpeed'})))
                flag = isscalar(obj.MaximumDopplerShift) && isvector(obj.UTDirectionOfTravel);
            else
                flag = false;
            end
            
        end
        
        % nrCDLChannel validateInputsImpl method
        function validateInputsImpl(obj,varargin)
            
            if (obj.ChannelFiltering) % step(in) and step(in,carrier)
                in = varargin{1};
                validateInputSignal(obj,in);
                if obj.OutputOFDMResponse % step(in,carrier)
                    carrier = varargin{2};
                    validateCarrierSampleRate(obj,carrier);
                end
            else % step() and step(carrier)
                if obj.OutputOFDMResponse
                    carrier = varargin{1};
                    validateCarrierSampleRate(obj,carrier);
                end
            end
            
        end
        
        % nrCDLChannel validatePropertiesImpl method
        function validatePropertiesImpl(obj)       
        
            validate(obj);
            
        end

        % nrCDLChannel processTunedPropertiesImpl method
        function processTunedPropertiesImpl(obj)
            
            % If any property has been tuned that influences the setup and
            % reset steps
            p = softNontunablePropertyList();
            P = numel(p);
            t = zeros(P,1);
            for i = 1:P
                t(i) = isChangedProperty(obj,p{i});
            end
            if (any(t))

                % Create a convenience structure indicating which
                % properties have been tuned
                tuned = tunedPropertyStructure(t);

                % Perform the subset of the setup and reset steps that is
                % influenced by the tuned properties
                performSetup(obj,tuned,t);
                performReset(obj,tuned);
                
            end

            if (isChangedProperty(obj,'InitialTime'))
                
                % If the tuned InitialTime changes the current time (to the
                % same tolerance as used to design fractional delay
                % filters)
                if (abs(obj.theTime - obj.InitialTime) > (obj.maxFractionalDelayError/obj.SampleRate))
                
                    % Set the time to the InitialTime property value
                    obj.theTime = obj.InitialTime;

                    % Reset channel filters
                    if (obj.ChannelFiltering)
                        reset(obj.channelFilter);
                        reset(obj.channelFilterReciprocal);
                    end
                    
                end
                
            end
            
        end
        
    end
    
    methods (Static, Access = protected)
       
        % validation of antenna array
        function array = validateAntennaArray(val,className,arrayName)
            
            if isstruct(val)
                fields = {'Size','ElementSpacing','PolarizationAngles','Element','PolarizationModel'};
                for i = 1:length(fields)
                    coder.internal.errorIf(~isfield(val,fields{i}),'nr5g:nrCDLChannel:MissingAntennaArrayField',fields{i},arrayName);
                end

                classAndArrayName = [className '.' arrayName];

                array = struct();

                fieldName = 'Size';
                validateattributes(val.Size,{'numeric'},{'integer','size',[1 5],'positive','finite'},[classAndArrayName '.' fieldName],fieldName);
                coder.internal.errorIf(val.Size(3)>2,'nr5g:nrCDLChannel:InvalidNumberPolarizations',[arrayName '.' fieldName],val.Size(3));
                array.Size = val.Size;

                fieldName = 'ElementSpacing';
                validateattributes(val.ElementSpacing,{'double'},{'real','size',[1 4],'nonnegative','finite'},[classAndArrayName '.' fieldName],fieldName);
                array.ElementSpacing = val.ElementSpacing;

                fieldName = 'PolarizationAngles';
                validateattributes(val.PolarizationAngles,{'double'},{'real','row','finite'},[classAndArrayName '.' fieldName],fieldName);
                numPol = length(val.PolarizationAngles);
                coder.internal.errorIf(numPol>2,'nr5g:nrCDLChannel:InvalidPolarizationAngles',[arrayName '.' fieldName],numPol);
                array.PolarizationAngles = val.PolarizationAngles;

                fieldName = 'Orientation';
                if isfield(val,fieldName)
                    validateattributes(val.Orientation,{'double'},{'real','size',[3 1],'finite'},[classAndArrayName '.' fieldName],fieldName);
                    array.Orientation = val.Orientation;
                end
                
                fieldName = 'Element';
                validatestring(val.Element,{'38.901','isotropic'},[classAndArrayName '.' fieldName],[arrayName '.' fieldName]);
                array.Element = val.Element;
                
                fieldName = 'PolarizationModel';
                validatestring(val.PolarizationModel,{'Model-1','Model-2'},[classAndArrayName '.' fieldName],[arrayName '.' fieldName]);            
                array.PolarizationModel = val.PolarizationModel;
             
            elseif isa(val,'phased.internal.AbstractArray') || isa(val,'phased.internal.AbstractSubarray')
                array = val; % pass PhAST array by reference
            else
                coder.internal.error('nr5g:nrCDLChannel:ArrayClassNotSupported',arrayName,class(val));
            end
            
        end
        
        % Methods supporting displayChannel
        
        % Determine if all elements of a PhAST antenna array are equal: all
        % elements are of the same type and their property values are equal
        function homo = isHomogeneous(array)
            homo = true;
            if isa(array,'phased.internal.AbstractSubarray')
                % Replicated subarrays may have different elements in a
                % subarray, but subarrays are all equal.
                if isa(array,'phased.PartitionedArray') && isa(array.Array,'phased.internal.AbstractHeterogeneousArray')
                    homo = nrCDLChannel.isElementSetHomogeneous(array.Array.ElementSet);
                end
            elseif isa(array,'phased.internal.AbstractHeterogeneousArray')
                % A PhAST Heterogeneous array is considered homogeneous if
                % its antenna element set is homogeneous or the element set
                % is not homogeneous but only one element is indexed.
                homoSet = nrCDLChannel.isElementSetHomogeneous(array.ElementSet);
                oneElemUsed = isscalar(unique(array.ElementIndices(:)));
                homo = homoSet || (~homoSet && oneElemUsed);
            elseif isa(array,'phased.NRRectangularPanelArray')
                % phased.NRRectangularPanelArray is homogeneous only if its
                % antenna element set is homogeneous
                homo = nrCDLChannel.isElementSetHomogeneous(array.ElementSet);
            end
        end
        
        % Determine if all elements of a PhAST antenna element set are
        % equal: all elements are of the same type and their property
        % values are equal
        function homo = isElementSetHomogeneous(ElementSet)
            if isscalar(ElementSet)
                homo = true;
            else
                cl = cellfun(@class, ElementSet, 'UniformOutput', false);
                if all(cellfun(@(x) strcmpi(cl{1},x),cl))
                    homo = true;
                    el = 2;
                    while homo && el <= length(ElementSet)
                        prop = properties(ElementSet{el});
                        for p = 1:length(prop)
                            homo = isequal(ElementSet{1}.(prop{p}),ElementSet{el}.(prop{p}));
                        end
                        el = el+1;
                    end
                else
                    homo = false;
                end
            end
        end
        
    end
    
% =========================================================================
%   private

    properties (Access = private, Nontunable)
        
        % The underlying parameter structure used to perform the channel
        % modeling. Note that 'theStruct' is applicable to
        % TransmitAndReceiveSwapped = false, with TransmitAndReceiveSwapped
        % = true being handled by permuting the path gains after executing
        % the underlying channel
        theStruct;
        
        % The underlying transmit and receive antenna properties. These are
        % presented on the public interface via similarly-named dependent
        % properties (drop the "the" from the start of the names below) and
        % transmit/receive property pairs (or elements within a property)
        % are switched around when TransmitAndReceiveSwapped = true
        theAnglesAoD = 0.0;
        theAnglesAoA = 0.0;
        theAnglesZoD = 0.0;
        theAnglesZoA = 0.0;
        theTransmitAntennaArray = nrCDLChannel.antennaArrayStructure([2 2 2 1 1],[0.5 0.5 1.0 1.0],[45 -45],[0;0;0],'38.901',nrCDLChannel.defaultPolarizationModel());
        theTransmitArrayOrientation = [0; 0; 0];
        theReceiveAntennaArray = nrCDLChannel.antennaArrayStructure([1 1 2 1 1],[0.5 0.5 0.5 0.5],[0 90],[0;0;0],'isotropic',nrCDLChannel.defaultPolarizationModel());
        theReceiveArrayOrientation = [0; 0; 0];
        theAngleSpreads = [5.0 11.0 3.0 3.0];
        theMeanAngles = [0.0 0.0 0.0 0.0];
        theMaximumDopplerShift = 5;
        theUTDirectionOfTravel = [0;90];
    end
    
    properties (Access = private)
                
        % Cached info output used to provide info method output
        theInfo;
        
        % channel filters
        channelFilter;
        channelFilterReciprocal;
        
        % RNG stream
        randomStream;
        
        % power delay profile (including columns for angles)
        pdp;
        
        % initial phase array (Phi, TR 38.901 step 10)
        initialPhases;
        
        % ray coupling (TR 38.901 step 8)
        rayCoupling;

        % whether scatterers are moving in the channel or not (alpha, 38.901 Rel-17 equation 7.6-46)
        movingScattererStates;
        
        % maximum speed of each scatterer in the channel (D, 38.901 equation Rel-17 7.6-46)
        maximumScattererSpeeds;
        
        % current time, advanced according to the input length (or
        % NumTimeSamples if ChannelFiltering is set to false) on each step
        % call
        theTime = 0.0;

        % Structure containing path gains per ray at t=0 and departure and
        % arrival unit vectors per ray
        staticChannelInfo;

        % Record of tunable property values prior to the most recent tuning
        preTuning = tunedPropertyStructure();

    end
    
    properties (Access = private, Nontunable)
        % Path filters
        pathFilters;

        % Control the type of channel response output: OFDM response or path gains
        OutputOFDMResponse = false;
    end

    properties (Access = private, Constant)
        
        % channel filter delay, used to design channel filter
        channelFilterDelay = 7;
        
        % stopband attenuation, used to design channel filter
        stopbandAttenuation = 70.0;

        % maximum allowable fractional delay error, used to determine if an
        % InitialTime update requires a filter reset
        maxFractionalDelayError = 0.01;
        
    end
    
    methods (Access = private)
        
        function setupRNG(obj)
            
            if strcmp(obj.RandomStream,'Global stream')
                obj.randomStream = [];
            else
                if isempty(coder.target)
                    obj.randomStream = RandStream('mt19937ar','Seed',obj.Seed);
                else
                    obj.randomStream = coder.internal.RandStream('mt19937ar','Seed',obj.Seed);
                end
            end
            
        end
        
        function resetRNG(obj)
            
            if (~strcmp(obj.RandomStream,'Global stream'))
                reset(obj.randomStream,obj.Seed);
            end
            
        end
        
        function s = getInfo(obj,doValidation)

            if isempty(coder.target) && isLocked(obj)
                s = obj.theInfo;
            else
                if doValidation
                    validate(obj);
                end
                s = wireless.internal.channelmodels.CDLChannelInfo(nrCDLChannel.makeCDLChannelStructure(obj));
            end
            
            % The number of transmit and receive antennas, number of input
            % and output signals, angle spreads, and departure and arrival
            % angles in 's' are applicable to TransmitAndReceiveSwapped =
            % false, so swap them if TransmitAndReceiveSwapped = true and
            % rearrange angle spreads as well.
            if (obj.TransmitAndReceiveSwapped)
                s = swapFields(s,'NumTransmitAntennas','NumReceiveAntennas');
                s = swapFields(s,'NumInputSignals','NumOutputSignals');
                s = swapFields(s,'AnglesAoD','AnglesAoA');
                s = swapFields(s,'AnglesZoD','AnglesZoA');
                s.ClusterAngleSpreads = s.ClusterAngleSpreads([2 1 4 3]);
            end

        end

        function validate(obj)
        
            % Validate properties that are active only for custom delay profile
            if (strcmp(obj.DelayProfile,'Custom'))
                % PDP properties
                pdp_props = {'PathDelays','AveragePathGains','AnglesAoD','AnglesAoA','AnglesZoD','AnglesZoA'};
                pdp_rows = validatePDPProperties(obj,pdp_props);
                min_rows = obj.NumStrongestClusters;
                coder.internal.errorIf(pdp_rows < min_rows,'nr5g:nrCDLChannel:LessThanStrongestClusters',min_rows,strjoin(pdp_props,','));

                M = wireless.internal.channelmodels.getNumberOfRays(obj);
                N = numel(obj.PathDelays);

                % XPR
                if ~isscalar(obj.XPR)
                    % If not scalar, XPR must be an N-by-M array
                    siz = size(obj.XPR);
                    errorFlag = ~all(siz==[N,M]);
                    propName = 'XPR';
                    coder.internal.errorIf(errorFlag,'nr5g:nrCDLChannel:XPRSize',...
                        propName,N,M,siz(1),siz(2));
                end

                % RayCoupling
                if isnumeric(obj.RayCoupling)
                    % If not 'Random', RayCoupling must be an N-by-M-by-3
                    % array with each element <= M
                    siz = size(obj.RayCoupling);
                    errorFlag = (numel(siz)~=3) || ~all(siz==[N,M,3]);
                    propName = 'RayCoupling';
                    coder.internal.errorIf(errorFlag,'nr5g:nrCDLChannel:RayCouplingSize',...
                        propName,N,M,siz(1),siz(2),size(obj.RayCoupling,3));
                    errorFlag = any(obj.RayCoupling>20,'all');
                    coder.internal.errorIf(errorFlag,'nr5g:nrCDLChannel:RayCouplingValue',propName,M);

                    % Throw a warning if multiple values are repeated
                    rayCouplingReshaped = reshape(permute(obj.RayCoupling,[2 1 3]),[M N*3]);
                    warningFlag = false;
                    for idx = 1:size(rayCouplingReshaped,2)
                        warningFlag = warningFlag || (numel(unique(rayCouplingReshaped(:,idx),'stable')) ~= M);
                    end
                    if warningFlag
                        coder.internal.warning('nr5g:nrCDLChannel:RayCouplingRepeatedValues',propName,M);
                    end
                end

                % InitialPhases
                if isnumeric(obj.InitialPhases) && ~isscalar(obj.InitialPhases)
                    % If not 'Random' or a scalar, InitialPhases must be an N-by-M-by-4 array
                    siz = size(obj.InitialPhases);
                    errorFlag = (numel(siz)~=3) || ~all(siz==[N,M,4]);
                    propName = 'InitialPhases';
                    coder.internal.errorIf(errorFlag,'nr5g:nrCDLChannel:InitialPhasesSize',...
                        propName,N,M,siz(1),siz(2),size(obj.InitialPhases,3));
                end
            end

            array = obj.TransmitAntennaArray;
            coder.internal.errorIf(isa(array,'phased.NRRectangularPanelArray') && array.EnablePanelSubarray,'nr5g:nrCDLChannel:ArrayFeatureNotSupported','TransmitAntennaArray');
            if isstruct(array) 
                coder.internal.errorIf(array.Size(3)>length(array.PolarizationAngles),'nr5g:nrCDLChannel:NumberPolarizationsMismatch','TransmitAntennaArray',array.Size(3),length(array.PolarizationAngles));
                checkPanelsOverlap(array,'TransmitAntennaArray');
            end
            array = obj.ReceiveAntennaArray;
            coder.internal.errorIf(isa(array,'phased.NRRectangularPanelArray') && array.EnablePanelSubarray,'nr5g:nrCDLChannel:ArrayFeatureNotSupported','ReceiveAntennaArray');
            if isstruct(array) 
                coder.internal.errorIf(array.Size(3)>length(array.PolarizationAngles),'nr5g:nrCDLChannel:NumberPolarizationsMismatch','ReceiveAntennaArray',array.Size(3),length(array.PolarizationAngles));
                checkPanelsOverlap(array,'ReceiveAntennaArray');
            end
        end
        
        function pdp_rows_out = validatePDPProperties(obj,pdp_props)
            
            Nprop = numel(pdp_props);
            pdp_rows = zeros(1,Nprop);
            for i = 1:Nprop
                pdp_rows(i) = numel(obj.(pdp_props{i}));
            end
            coder.internal.errorIf(numel(unique(pdp_rows))~=1,'nr5g:nrCDLChannel:UnequalDelayProfileLengths',strjoin(pdp_props,','));
            pdp_rows = unique(pdp_rows);
            pdp_rows_out = pdp_rows(1);
            
        end
        
        % validate input signal
        function validateInputSignal(obj,in)
            
            doValidation = false;
            info = getInfo(obj,doValidation);
            ninsig = info.NumInputSignals;
            coder.internal.errorIf(size(in,2)~=ninsig,'nr5g:nrCDLChannel:SignalInputNotMatchTxArray',size(in,2),ninsig);
            validateattributes(in,{'double','single'},{'2d','finite'},class(obj),'signal input');
            
        end
        
        % validate sample rate from carrier
        function validateCarrierSampleRate(obj,carrier)
            ofdmInfo = nr5g.internal.OFDMInfo(carrier,[]);
            carrierSR = ofdmInfo.SampleRate;
            coder.internal.errorIf(carrierSR ~= obj.SampleRate,'nr5g:nrCDLChannel:CarrierSampleRateMismatch',obj.SampleRate,carrierSR);
        end

        function setSwappedTransmitAntennaArray(obj,val)
            
            if (isempty(coder.target))
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theTransmitAntennaArray = val;
                else
                    obj.theReceiveAntennaArray = val;
                end
            else
                obj.theTransmitAntennaArray = val;
            end
            
        end
        
        function setSwappedTransmitArrayOrientation(obj,val,propName)
        
            if (isempty(coder.target))
                recordPropertyTuning(obj,propName);
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theTransmitArrayOrientation = val;
                else
                    obj.theReceiveArrayOrientation = val;
                end
            else
                obj.theTransmitArrayOrientation = val;
            end
            
        end
        
        function setSwappedReceiveAntennaArray(obj,val)
            
            if (isempty(coder.target))
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theReceiveAntennaArray = val;
                else
                    obj.theTransmitAntennaArray = val;
                end
            else
                obj.theReceiveAntennaArray = val;
            end
            
        end
        
        function setSwappedReceiveArrayOrientation(obj,val,propName)
            
            if (isempty(coder.target))
                recordPropertyTuning(obj,propName);
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theReceiveArrayOrientation = val;
                else
                    obj.theTransmitArrayOrientation = val;
                end
            else
                obj.theReceiveArrayOrientation = val;
            end
            
        end
        
        % validate NumTimeSamples
        function validateNumTimeSamples(obj,numTimeSamples,carrier)
            % Calculate symbol lengths
            ofdmInfo = nr5g.internal.OFDMInfo(carrier,[]);
            symbolLengths = ofdmInfo.CyclicPrefixLengths + ofdmInfo.Nfft;
            slotInSubframe = mod(carrier.NSlot,carrier.SlotsPerSubframe);
            numSamplesInCurrentSlot = sum(symbolLengths(slotInSubframe*ofdmInfo.SymbolsPerSlot + (1:ofdmInfo.SymbolsPerSlot)));

            % Check error
            if obj.ChannelFiltering
                coder.internal.errorIf(numTimeSamples< numSamplesInCurrentSlot,'nr5g:nrCDLChannel:NumInputSamplesNotASlotChFiltOn',numTimeSamples,numSamplesInCurrentSlot,carrier.NSlot);
            else
                coder.internal.errorIf(numTimeSamples< numSamplesInCurrentSlot,'nr5g:nrCDLChannel:NumTimeSamplesNotASlotChFiltOff',numTimeSamples,numSamplesInCurrentSlot,carrier.NSlot);
            end
        end

        % get preferred channel filtering policy for a given input
        function p = getFilterPolicy(obj,in)

            outputtype = class(in);
            T = size(in,1);
            outputproto = zeros(0,0,'like',in);
            sampleTimes = wireless.internal.channelmodels.getCDLSampleTimes(obj.theStruct,T,outputproto,obj.theTime);
            Ncs = numel(sampleTimes);
            Np = size(obj.pdp,1);
            Nt = obj.theStruct.NumInputSignals;
            Nr = obj.theStruct.NumOutputSignals;
            sizg = [Ncs Np Nt Nr];
            p = wireless.internal.channelmodels.getFilterPolicy(outputtype,[sizg T]);

        end

    end

    methods (Static, Access = protected)
        
        % Create underlying channel properties from nrCDLChannel
        % properties. Note that 'model' is applicable to
        % TransmitAndReceiveSwapped = false, with TransmitAndReceiveSwapped
        % = true being handled by permuting the path gains after executing
        % the underlying channel
        function model = makeCDLChannelStructure(obj)
            
            model = nr5g.internal.nrCDLChannel.makeCDLChannelStructure(obj.NormalizePathGains,obj.NormalizeChannelOutputs,obj.theMaximumDopplerShift,obj.theUTDirectionOfTravel,obj.CarrierFrequency,obj.Seed,obj.DelaySpread,obj.SampleDensity,obj.SampleRate,obj.DelayProfile,obj.PathDelays,obj.AveragePathGains,obj.theAnglesAoD,obj.theAnglesAoA,obj.theAnglesZoD,obj.theAnglesZoA,obj.HasLOSCluster,obj.KFactorFirstCluster,obj.KFactorScaling,obj.KFactor,obj.AngleScaling,obj.theAngleSpreads,obj.XPR,obj.ClusterDelaySpread,obj.NumStrongestClusters,obj.theMeanAngles,obj.channelFilterDelay,obj.theTransmitAntennaArray,obj.theReceiveAntennaArray,obj.theTransmitArrayOrientation,obj.theReceiveArrayOrientation,obj.RayCoupling,obj.InitialPhases,obj.MovingScattererProportion,obj.MaximumScattererSpeed);
            
        end
        
        % Return a CDL channel (DelayProfile='Custom') populated with a CDL
        % channel preset (CDL-A, ..., CDL-E) and scaled by the inputs delay
        % spread and K factor. K factor only applies for CDL-D and CDL-E.
        function custom = makeCustomCDL(delayProfile,delaySpread,kFactor)

            % Create a CDL preset model struct to help set up the output
            % custom CDL channel. This model struct contains the average
            % path gains, path delays, and angles of the specified input
            % CDL preset scaled by the input delay spread and K factor.
            hasLOS = nr5g.internal.nrCDLChannel.hasLOSCluster(delayProfile,NaN);
            kFactorScaling = hasLOS && ~isempty(kFactor);
            presetModel = nr5g.internal.nrCDLChannel.makeCDLChannelDelayProfileStructure(0,delaySpread,delayProfile,0,0,0,0,0,0,0,0,kFactorScaling,kFactor,0,0,0,0,0,0);

            % Create a custom CDL channel and populate its properties using
            % the info from the presets model struct.
            custom = nrCDLChannel;
            custom.DelayProfile = 'Custom';

            % Copy path delays and angles from the model struct to the
            % custom channel object. If it is a LOS channel, skip the first
            % entry as it is repeated.
            p = 1 + hasLOS;
            custom.PathDelays = presetModel.PathDelays(p:end);
            custom.AnglesAoD = presetModel.AnglesAoD(p:end);
            custom.AnglesAoA = presetModel.AnglesAoA(p:end);
            custom.AnglesZoD = presetModel.AnglesZoD(p:end);
            custom.AnglesZoA = presetModel.AnglesZoA(p:end);

            % Set cluster-wise angle spreads and XPR
            custom.AngleSpreads = presetModel.AngleSpreads;
            custom.XPR = presetModel.XPR;

            % Set the AveragePathGains, HasLOSCluster and
            % KFactorFirstCluster properties.
            if hasLOS
                % For LOS, the first cluster is split into LOS and NLOS in
                % the model struct, so combine these parts into a single
                % cluster and specify the K factor of the first cluster.
                custom.HasLOSCluster = true;
                apg = [pow2db(sum(db2pow(presetModel.AveragePathGains(1:2)))) ...
                       presetModel.AveragePathGains(p+1:end)];
                custom.KFactorFirstCluster = presetModel.AveragePathGains(1) - presetModel.AveragePathGains(2);
            else
                apg = presetModel.AveragePathGains;
            end
            custom.AveragePathGains = apg;

        end

        % Create channel filter from nrCDLChannel properties
        function channelFilter = setupChannelFilter(obj,optimizeScalarGainFilter)
            
            coder.extrinsic('nr5g.internal.nrCDLChannel.makeCDLChannelDelayProfileStructure');
            coder.extrinsic('nr5g.internal.nrCDLChannel.getPathDelays');
            
            theStruct = coder.const(nr5g.internal.nrCDLChannel.makeCDLChannelDelayProfileStructure(obj.NormalizePathGains,obj.DelaySpread,obj.DelayProfile,obj.PathDelays,obj.AveragePathGains,obj.theAnglesAoD,obj.theAnglesAoA,obj.theAnglesZoD,obj.theAnglesZoA,obj.HasLOSCluster,obj.KFactorFirstCluster,obj.KFactorScaling,obj.KFactor,obj.AngleScaling,obj.theAngleSpreads,obj.XPR,obj.ClusterDelaySpread,obj.NumStrongestClusters,obj.theMeanAngles));
            pathDelays = coder.const(nr5g.internal.nrCDLChannel.getPathDelays(theStruct));
            channelFilter = constructChannelFilter(pathDelays,obj.channelFilterDelay,obj.stopbandAttenuation,obj.SampleRate,optimizeScalarGainFilter);
            
        end
        
    end

    methods (Static)

        function p = matlabCodegenSoftNontunableProperties(~)
            p = softNontunablePropertyList();
        end

    end
        
    methods (Static, Access = private)
        
        function pathFilters = makePathFilters(obj)

            if (isempty(coder.target) && obj.ChannelFiltering && isLocked(obj))
                if (~obj.TransmitAndReceiveSwapped)
                    channelFilterInfo = info(obj.channelFilter);
                else
                    channelFilterInfo = info(obj.channelFilterReciprocal);
                end
            else
                if ~isLocked(obj)
                    % Call validation only if it is necessary
                    validate(obj);
                end
                % Note: optimizeScalarGainFilter is an optimization option,
                % it does not affect the path filters
                optimizeScalarGainFilter = false;
                channelFilter = nrCDLChannel.setupChannelFilter(obj,optimizeScalarGainFilter);
                channelFilterInfo = info(channelFilter);
            end
            pathFilters = channelFilterInfo.ChannelFilterCoefficients.';

        end

        function s = antennaArrayStructure(arraySize,arrayElementSpacing,polarizationAngles,orientation,element,polarizationModel)

            s.Size = arraySize;
            s.ElementSpacing = arrayElementSpacing;
            s.PolarizationAngles = polarizationAngles;
            s.Orientation = orientation;
            s.Element = element;
            s.PolarizationModel = polarizationModel;

        end
        
        function d = defaultPolarizationModel()
            
            d = 'Model-2';
            
        end
        
        function val = swapTxRx(txval,rxval,txrxSwapped)
            
            if (isempty(coder.target))
                if (~txrxSwapped)
                    val = txval;
                else
                    val = rxval;
                end
            else
                val = txval;
            end
            
        end
    
    end
    
end

function channelFilter = constructChannelFilter(pathDelays,channelFilterDelay,stopbandAttenuation,sampleRate,optimizeScalarGainFilter)
    
    channelFilter = comm.ChannelFilter( ...
        'SampleRate',sampleRate, ....
        'PathDelays',pathDelays, ...
        'FilterDelaySource','Custom', ...
        'FilterDelay',channelFilterDelay, ...
        'NormalizeChannelOutputs',false, ...
        'OptimizeScalarGainFilter',optimizeScalarGainFilter);
    channelFilter.setStopbandAttenuation(stopbandAttenuation);
    
end

function s = swapFields(s,a,b)

    t = s.(a);
    s.(a) = s.(b);
    s.(b) = t;
    
end

% Warn if antenna panels overlap
function checkPanelsOverlap(array,arrayName)

    panelHeight = (array.Size(1)-1)*array.ElementSpacing(1);
    panelWidth = (array.Size(2)-1)*array.ElementSpacing(2);
    
    vertOverlap = (panelHeight >= array.ElementSpacing(3)) && (array.Size(4)>1);
    horizOverlap = (panelWidth >= array.ElementSpacing(4)) && (array.Size(5)>1);
    if vertOverlap || horizOverlap
        coder.internal.warning('nr5g:nrCDLChannel:PanelsOverlap',arrayName,num2str(panelHeight),num2str(panelWidth));
    end
end

% Parse options for custom CDL construction from presets
%   nrCDLChannel('DelayProfile','Custom','InitialDelayProfile','CDL-X',...)
function [customFromPreset,delayProfile,delaySpread,kFactor] = constructionOptions(varargin)
    
    % Parse inputs for special constructor consisting of
    % DelayProfile, InitialDelayProfile, DelaySpread, and
    % KFactor parameters.
    parseOptions = struct( ...
        'CaseSensitivity',false, ...
        'PartialMatching','unique', ...
        'StructExpand',true, ...
        'IgnoreNulls',false, ...
        'SupportOverrides',true);
    optNames = {'DelayProfile','InitialDelayProfile','DelaySpread','KFactor'};
    [opts,unmatched] = coder.internal.parseInputs({},optNames,parseOptions,varargin{:});

    % Extract specified values or defaults
    defaults = struct('DelayProfile','CDL-A','InitialDelayProfile','','DelaySpread',30e-9,'KFactor',[]);
    s = coder.internal.vararginToStruct(opts,defaults,varargin{:});

    % If the DelayProfile = 'Custom' and InitialDelayProfile is present, this is
    % the syntax to build a custom CDL from a preset one.
    customFromPreset = strcmpi(s.DelayProfile,'Custom') && (opts.InitialDelayProfile ~= 0);
    if customFromPreset
        
        fcnName = 'nrCDLChannel';

        % Validate option InitialDelayProfile
        validatestring(s.InitialDelayProfile,{'CDL-A','CDL-B','CDL-C','CDL-D','CDL-E'},fcnName,'InitialDelayProfile');

        % Validate option DelaySpread
        validateDelaySpread(s.DelaySpread,fcnName,'DelaySpread');

        % Validate option KFactor only for LOS channels
        hasLOS = nr5g.internal.nrCDLChannel.hasLOSCluster(s.InitialDelayProfile,NaN);
        if hasLOS && opts.KFactor
            validateKFactor(s.KFactor,fcnName,'KFactor');
        end

        delayProfile = s.InitialDelayProfile;
        delaySpread = s.DelaySpread;
        kFactor = s.KFactor;

    else % No special syntax

        % If InitialDelayProfile option was specified but DelayProfile was
        % not set to "Custom", provide a helpful error message
        coder.internal.errorIf(opts.InitialDelayProfile ~= 0, 'nr5g:nrCDLChannel:WrongDelayProfile')

        delayProfile = [];
        delaySpread = [];
        kFactor = [];

    end

    if customFromPreset && any(unmatched)
        % When DelayProfile = 'Custom', InitialDelayProfile = 'CDL-X',
        % the only additional options are DelaySpread and KFactor.
        % Error if there are more options specified as PV pairs.
        str = varargin{find(unmatched,1,'first')};
        coder.internal.errorIf(any(unmatched),'nr5g:nrCDLChannel:UnsupportedOption',str);
    end
end

% Common validators for properties and construction options
function validateDelaySpread(in,varargin)
    validateattributes(in,{'double'},{'real','scalar','nonnegative','finite'},varargin{:});
end

function validateKFactor(in,varargin)
    validateattributes(in,{'double'},{'real','scalar','finite'},varargin{:});
end

function recordPropertyTuning(obj,propName)
    if (isLocked(obj))
        obj.preTuning.(propName) = obj.(propName);
    end
end

function performSetup(obj,varargin)

    % Determine input arguments
    if (nargin==3)
        % Called when properties are tuned on a locked object
        tuned = varargin{1};
        t = varargin{2};
    else
        % Called from setupImpl for an unlocked object
        % nargin = (1 + obj.ChannelFiltering)
        if (obj.ChannelFiltering)
            in = varargin{1};
        end
    end

    % Create underlying channel properties from nrCDLChannel properties.
    % Note that the values in 'theStruct' are applicable to
    % TransmitAndReceiveSwapped = false, with TransmitAndReceiveSwapped =
    % true being handled by permuting the path gains after executing the
    % underlying channel
    if (nargin<=2 ... % unlocked object
            || (isempty(coder.target) && any(t))) % locked tuned object
        obj.theStruct = nrCDLChannel.makeCDLChannelStructure(obj);
    end

    % Perform parameter validation, perform sub-clustering if applicable,
    % and create information structure
    if (nargin<=2 ... % unlocked object
            || (isempty(coder.target) && any(t) && ~onlyTunedInitialPhases(obj,tuned,t))) % locked tuned object
        [obj.theInfo,obj.pdp] = wireless.internal.channelmodels.CDLChannelInfo(obj.theStruct);
    end

    % Setup channel filters
    if (obj.ChannelFiltering)

        if (nargin==2 ... % unlocked object
                || (isempty(coder.target) && tuned.PathDelays)) % locked tuned object

            if (nargin==2) % unlocked object

                % Determine the best filter option for the current input
                p = getFilterPolicy(obj,in);

                % If the option is to filter waveform sections where each
                % section corresponds to a first dimension element of the
                % path gains, configure the channel filters for efficient
                % execution of this case
                if (isempty(coder.target))
                    optimizeScalarGainFilter = (p.FilterOption==2);
                else
                    optimizeScalarGainFilter = false;
                end

            else % locked tuned object

                % Keep the current filtering approach
                optimizeScalarGainFilter = obj.channelFilter.OptimizeScalarGainFilter;

            end

            % Create channel filters
            obj.channelFilter = nrCDLChannel.setupChannelFilter(obj,optimizeScalarGainFilter);
            obj.channelFilterReciprocal = nrCDLChannel.setupChannelFilter(obj,optimizeScalarGainFilter);

            % Cached path filters need cleared
            obj.pathFilters = [];

        end

    end

end

function performReset(obj,varargin)

    % Determine input arguments
    if (nargin==2)
        tuned = varargin{1};
    end

    % Set up initial phases
    if (nargin==1 ... % unlocked object
            || (isempty(coder.target) && tunedInitialPhases(obj,tuned))) % locked tuned object
        obj.initialPhases = wireless.internal.channelmodels.getCDLInitialPhases(obj.theStruct,obj.randomStream,obj.theInfo.ClusterTypes);
    end

    % Set up ray coupling - although this is not influenced by tunable
    % properties, it is performed here so that it occurs after the setup of
    % the initial phases, for backwards compatibility in the use of the
    % 'randomStream'
    if (nargin==1) % unlocked object
        obj.rayCoupling = wireless.internal.channelmodels.getCDLRayCoupling(obj.theStruct,obj.randomStream,obj.theInfo.ClusterTypes);
    end

    % Get moving scatterer states and speeds
    if (nargin==1) % unlocked object
        [obj.movingScattererStates,obj.maximumScattererSpeeds] = wireless.internal.channelmodels.getDualMobilityScattererVariables(obj.theStruct,obj.randomStream,obj.theInfo.ClusterTypes);
    end

    % Calculate the static path gains with no Doppler effect
    if (nargin==1 ... % unlocked object
            || (isempty(coder.target) && (tunedAngles(tuned) || tuned.AveragePathGains || tunedInitialPhases(obj,tuned)))) % locked tuned object
        obj.staticChannelInfo = wireless.internal.channelmodels.staticCDLChannel(obj.theStruct,obj.theInfo,obj.pdp,obj.rayCoupling,obj.initialPhases);
    end

end

function p = softNontunablePropertyList()
    p = {'PathDelays' 'AveragePathGains' 'AnglesAoD' 'AnglesAoA' 'AnglesZoD' 'AnglesZoA' 'MaximumDopplerShift' 'UTDirectionOfTravel' 'ReceiveArrayOrientation' 'TransmitArrayOrientation' 'InitialPhases'}.';
end

function tuned = tunedPropertyStructure(t)
    p = softNontunablePropertyList();
    if (nargin==0)
        t = NaN(size(p));
    end
    tuned = cell2struct(num2cell(t),p);
end

function y = tunedAngles(tuned)
    y = any([tuned.AnglesAoD tuned.AnglesAoA tuned.AnglesZoD tuned.AnglesZoA tuned.TransmitArrayOrientation tuned.ReceiveArrayOrientation]);
end

function y = tunedInitialPhases(obj,tuned)
    y = tuned.InitialPhases && ~all(strcmp({obj.preTuning.InitialPhases obj.InitialPhases},'Random'));
end

function y = onlyTunedInitialPhases(obj,tuned,t)
    y = (nnz(t)==1) && tunedInitialPhases(obj,tuned);
end
