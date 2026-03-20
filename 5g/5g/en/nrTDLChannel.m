classdef nrTDLChannel< matlab.System
%nrTDLChannel TR 38.901 or 38.811 Tapped Delay Line (TDL) channel
%   CHAN = nrTDLChannel creates a TDL MIMO fading channel System object,
%   CHAN. This object filters an input signal through the TDL MIMO channel
%   to obtain the channel-impaired signal. This object implements the
%   following aspects of TR 38.901:
%   * Section 7.7.2 Tapped Delay Line (TDL) models
%   * Section 7.7.3 Scaling of delays 
%   * Section 7.7.6 K-factor for LOS channel models
%   * Section 7.7.5.2 TDL extension: Applying a correlation matrix
%   This object also implements the following aspects of TR 38.811:
%   * Section 6.9.2 TDL models
%
%   CHAN = nrTDLChannel(Name,Value) creates a TDL MIMO channel object,
%   CHAN, with the specified property Name set to the specified Value. You
%   can specify additional name-value pair arguments in any order as
%   (Name1,Value1,...,NameN,ValueN).
%
%   Step method syntax for ChannelFiltering set to true:
%
%   Y = step(CHAN,X) filters the input signal X through a TDL MIMO fading
%   channel and returns the result in Y. The input X can be a double or
%   single precision data type scalar, vector, or 2-D matrix. X is of size
%   Ns-by-Nt, where Ns is the number of samples and Nt is the number of
%   transmit antennas. Y is the output signal of size Ns-by-Nr, where Nr is
%   the number of receive antennas. Y is of the same data type as the input
%   signal X.
% 
%   [Y,PATHGAINS] = step(CHAN,X) returns the MIMO channel path gains of the
%   underlying fading process in PATHGAINS. PATHGAINS is of size
%   Ns-by-Np-by-Nt-by-Nr, where Np is the number of paths. PATHGAINS is of
%   the same data type as the input signal X.
%
%   [Y,PATHGAINS,SAMPLETIMES] = step(CHAN,X) also returns the sample times
%   of the channel snapshots (1st dimension elements) of PATHGAINS.
%   SAMPLETIMES is of size Ns-by-1 and is of double precision data type
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
%   nrTDLChannel methods:
%
%   step                   - Filter input signal through a TDL MIMO fading
%                            channel (see above)
%   release                - Allow property value and input characteristics
%                            changes
%   clone                  - Create TDL channel object with same property 
%                            values
%   isLocked               - Locked status (logical)
%   <a href="matlab:help nrTDLChannel/reset">reset</a>                  - Reset states of filters, and random stream if the
%                            RandomStream property is set to 'mt19937ar with seed'
%   <a href="matlab:help nrTDLChannel/infoImpl">info</a>                   - Return characteristic information about the TDL 
%                            channel
%   getPathFilters         - Get filter impulse responses for the filters
%                            which apply the path delays to the input 
%                            waveform
%   swapTransmitAndReceive - Swap transmit and receive antennas
%
%   nrTDLChannel properties:
%
%   DelayProfile               - TDL delay profile
%   PathDelays                 - Discrete path delay vector (s)
%   AveragePathGains           - Average path gain vector (dB)
%   FadingDistribution         - Rayleigh or Rician fading
%   KFactorFirstTap            - K-factor of first tap (dB)
%   DelaySpread                - Desired delay spread (s)
%   SatelliteDopplerShift      - Doppler shift due to satellite movement (Hz)
%   MaximumDopplerShift        - Maximum Doppler shift (Hz)
%   KFactorScaling             - Enable K-factor scaling (logical)
%   KFactor                    - Desired Rician K-factor (dB)
%   SampleRate                 - Input signal sample rate (Hz)
%   PathGainSampleRate         - Path gain generation sample rate choice ('signal' or 'auto')
%   MIMOCorrelation            - Correlation between UE and BS antennas
%   Polarization               - Antenna polarization arrangement
%   TransmissionDirection      - Transmission direction (Uplink/Downlink)
%   NumTransmitAntennas        - Number of transmit antennas
%   NumReceiveAntennas         - Number of receive antennas
%   TransmitCorrelationMatrix  - Transmit spatial correlation matrix (or 3-D array)
%   ReceiveCorrelationMatrix   - Receive spatial correlation matrix (or 3-D array)
%   TransmitPolarizationAngles - Transmit polarization slant angles in degrees
%   ReceivePolarizationAngles  - Receive polarization slant angles in degrees
%   XPR                        - Cross polarization power ratio (dB)
%   SpatialCorrelationMatrix   - Combined correlation matrix (or 3-D array)
%   NormalizePathGains         - Normalize path gains (logical)
%   InitialTime                - Start time of fading process (s)
%   NumSinusoids               - Number of sinusoids in sum-of-sinusoids technique
%   RandomStream               - Source of random number stream
%   Seed                       - Initial seed of mt19937ar random number stream
%   NormalizeChannelOutputs    - Normalize channel outputs (logical)
%   ChannelFiltering           - Perform filtering of input signal (logical)
%   NumTimeSamples             - Number of time samples
%   OutputDataType             - Path gain output data type
%   TransmitAndReceiveSwapped  - Transmit and receive antennas swapped (logical)
%   ChannelResponseOutput      - Specify the type of the returned channel response
%
%   Note that for non-terrestrial network (NTN) delay profiles, when the
%   MaximumDopplerShift and SatelliteDopplerShift properties are set to
%   zero, the channel remains static for the entire input. In case of other
%   delay profiles, when the MaximumDopplerShift property is set to zero,
%   the channel remains static for entire input. In both cases, you can use
%   the reset method to generate a new channel realization.
%
%   % Example 1: 
%   % Configure a TDL channel, filter an input signal and plot the received
%   % waveform spectrum. Use TDL-C delay profile, 300 ns delay spread and 
%   % UE velocity 30 km/h.
%   
%   v = 30.0;                    % UE velocity in km/h
%   fc = 4e9;                    % carrier frequency in Hz
%   c = physconst('lightspeed'); % speed of light in m/s
%   fd = (v*1000/3600)/c*fc;     % UE max Doppler frequency in Hz
%
%   tdl = nrTDLChannel;
%   tdl.DelayProfile = 'TDL-C';
%   tdl.DelaySpread = 300e-9;
%   tdl.MaximumDopplerShift = fd;
%
%   % Create a random waveform of 1 subframe duration with 1 antenna, pass 
%   % it through the channel and plot the received waveform spectrum
%
%   SR = 30.72e6;
%   T = SR * 1e-3;
%   tdl.SampleRate = SR;
%   tdlinfo = info(tdl);
%   Nt = tdlinfo.NumTransmitAntennas;
%
%   txWaveform = complex(randn(T,Nt),randn(T,Nt));
%
%   rxWaveform = tdl(txWaveform);
%
%   analyzer = spectrumAnalyzer('SampleRate',tdl.SampleRate);
%   analyzer.Title = ['Received signal spectrum for ' tdl.DelayProfile];
%   analyzer(rxWaveform);
%
%   % Example 2:
%   % Plot the path gains of a TDL-E delay profile in a SISO case for a
%   % Doppler shift of 70 Hz.
%    
%   tdl = nrTDLChannel;
%   tdl.SampleRate = 500e3;
%   tdl.NumTransmitAntennas = 1;
%   tdl.NumReceiveAntennas = 1;
%   tdl.MaximumDopplerShift = 70;
%   tdl.DelayProfile = 'TDL-E';
%    
%   % dummy input signal, its length determines the number of path gain
%   % samples generated
%   in = zeros(1000,tdl.NumTransmitAntennas);
%    
%   % generate path gains
%   [~, pathGains] = tdl(in);
%   mesh(10*log10(abs(pathGains)));
%   view(26,17); xlabel('channel path');
%   ylabel('sample (time)'); zlabel('magnitude (dB)');
%
%   % Example 3:
%   % Configure a channel with cross-polar antennas and filter an input 
%   % signal. Use TDL-D delay profile, 10 ns delay spread and a desired
%   % overall K-factor of 7.0 dB. Configure cross-polar antennas according
%   % to TS 36.101 Annex B.2.3A.3 4x2 high correlation.
%
%   tdl = nrTDLChannel;
%   tdl.NumTransmitAntennas = 4;
%   tdl.DelayProfile = 'TDL-D';
%   tdl.DelaySpread = 10e-9;
%   tdl.KFactorScaling = true;
%   tdl.KFactor = 7.0; % desired model K-factor (K_desired) dB
%   tdl.MIMOCorrelation = 'High';
%   tdl.Polarization = 'Cross-Polar';
%
%   % Create a random waveform of 1 subframe duration with 4 antennas and 
%   % pass it through the channel
%
%   SR = 1.92e6;
%   T = SR * 1e-3;
%   tdl.SampleRate = SR;
%   tdlinfo = info(tdl);
%   Nt = tdlinfo.NumTransmitAntennas;
%
%   txWaveform = complex(randn(T,Nt),randn(T,Nt));
%
%   rxWaveform = tdl(txWaveform);
%
%   % Example 4:
%   % Configure a channel with a customized delay profile and filter an 
%   % input signal. Set two channel taps as follows:
%   %   tap 1: Rician, average power 0 dB, K-factor 10 dB, delay zero
%   %   tap 2: Rayleigh, average power -5 dB, delay 45 ns
%
%   tdl = nrTDLChannel;
%   tdl.NumTransmitAntennas = 1;
%   tdl.DelayProfile = 'Custom';
%   tdl.FadingDistribution = 'Rician';
%   tdl.KFactorFirstTap = 10.0; % K-factor of 1st tap (K_1) in dB
%   tdl.PathDelays = [0.0 45e-9];
%   tdl.AveragePathGains = [0.0 -5.0];
%
%   % Create a random waveform of 1 subframe duration with 1 antenna and
%   % pass it through the channel
%
%   SR = 30.72e6;
%   T = SR * 1e-3;
%   tdl.SampleRate = SR;
%   tdlinfo = info(tdl);
%   Nt = tdlinfo.NumTransmitAntennas;
%
%   txWaveform = complex(randn(T,Nt),randn(T,Nt));
%
%   rxWaveform = tdl(txWaveform);
%
%   % Example 5:
%   % Configure an NTN channel for a satellite moving at  an altitude of
%   % 600 km with a speed of 7562.2 m/s and having an elevation angle of
%   % 50 degrees with UE. Use NTN-TDL-A profile with 100 ns delay spread,
%   % UE speed of 3 km/hr, and carrier frequency of 2 GHz.
%
%   % Calculate the Doppler shift due to satellite and maximum Doppler
%   % shift due to scattering environment around UE
%   r = physconst('earthradius');              % Earth radius in m
%   c = physconst('lightspeed');               % Speed of light in m/s
%   fc = 2e9;                                  % Carrier frequency in Hz
%   theta = 50;                                % Elevation angle in degrees
%   h = 600e3;                                 % Satellite altitude in m
%   vSat = 7562.2;                             % Satellite speed in m/s
%   vUE = 3*1000/3600;                         % UE speed in m/s
%   fdMaxUE = (vUE*fc)/c;                      % UE maximum Doppler shift in Hz
%   fdSat = (vSat*fc/c)*(r*cosd(theta)/(r+h)); % Satellite Doppler shift in Hz
%
%   % Configure the TDL channel
%   ntnChan = nrTDLChannel;
%   ntnChan.DelayProfile = 'NTN-TDL-A';
%   ntnChan.DelaySpread = 100e-9;
%   ntnChan.SatelliteDopplerShift = fdSat;
%   ntnChan.MaximumDopplerShift = fdMaxUE;
%
%   % Create a random waveform of 1 subframe duration with the configured
%   % number of antennas
%   SR = 30.72e6;
%   T = SR*1e-3;
%   ntnChan.SampleRate = SR;
%   ntnChanInfo = info(ntnChan);
%   Nt = ntnChanInfo.NumTransmitAntennas;
%   txWaveform = randn(T,Nt,'like',1i);
%
%   % Pass the waveform through the channel
%   rxWaveform = ntnChan(txWaveform);
%
%   See also nrCDLChannel, nrHSTChannel, comm.MIMOChannel,
%   nrPerfectTimingEstimate, nrPerfectChannelEstimate.

 
%   Copyright 2016-2025 The MathWorks, Inc.

    methods
        function out=nrTDLChannel
            % Set property values from any name-value pairs input to the
            % constructor
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
            %   PATHGAINS output of the step method. These filters don't change
            %   once the object is created, therefore it only needs to be
            %   called once.
        end

        function out=infoImpl(~) %#ok<STOUT>
            %info Returns characteristic information about the TDL channel
            %   S = info(CHAN) returns a structure containing characteristic
            %   information, S, about the TDL fading channel. A description of
            %   the fields and their values is as follows:
            % 
            %   ChannelFilterDelay       - Channel filter delay in samples.
            %   MaximumChannelDelay      - Maximum channel delay in samples. 
            %                              This delay consists of the maximum
            %                              propagation delay and the
            %                              ChannelFilterDelay.
            %   AveragePathGains         - A row vector of the average gains of the
            %                              discrete paths, in dB. These values
            %                              include the effect of K-factor scaling if
            %                              enabled. 
            %   PathDelays               - A row vector providing the delays of the
            %                              discrete channel paths, in seconds. These
            %                              values include the effect of the desired
            %                              delay spread scaling, and desired
            %                              K-factor scaling if enabled.
            %   KFactorFirstTap          - K-factor of first tap of delay profile,
            %                              in dB. If the first tap of the delay
            %                              profile follows a Rayleigh rather than
            %                              Rician distribution, KFactorFirstTap
            %                              is -Inf.
            %   NumTransmitAntennas      - Number of transmit antennas.
            %   NumReceiveAntennas       - Number of receive antennas.
            %   SpatialCorrelationMatrix - Combined correlation matrix (or 3-D array).
        end

        function out=isInactivePropertyImpl(~) %#ok<STOUT>
        end

        function out=isInputComplexityMutableImpl(~) %#ok<STOUT>
        end

        function out=isInputSizeMutableImpl(~) %#ok<STOUT>
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
        end

        function out=processTunedPropertiesImpl(~) %#ok<STOUT>
            % Perform actions when tunable properties change
            % between calls to the System object
        end

        function out=releaseImpl(~) %#ok<STOUT>
        end

        function out=resetImpl(~) %#ok<STOUT>
            % reset the MIMOChannel
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
        end

        function out=setupImpl(~) %#ok<STOUT>
            % Construct the MIMOChannel. Note that 'theChannel' is
            % applicable to TransmitAndReceiveSwapped = false, with
            % TransmitAndReceiveSwapped = true being handled by permuting
            % the path gains after executing the underlying channel
        end

        function out=stepImpl(~) %#ok<STOUT>
            % Possible number of inputs (in addition to system object)
            %   obj.step():           ChannelFiltering = false, ChannelResponseOutput = 'path-gains'
            %   obj.step(in):         ChannelFiltering = true,  ChannelResponseOutput = 'path-gains'
            %   obj.step(carrier):    ChannelFiltering = false, ChannelResponseOutput = 'ofdm-response'
            %   obj.step(in,carrier): ChannelFiltering = true,  ChannelResponseOutput = 'ofdm-response'
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
            %   of link direction: NumTransmitAntennas and NumReceiveAntennas,
            %   TransmitCorrelationMatrix and ReceiveCorrelationMatrix,
            %   TransmitPolarizationAngles and ReceivePolarizationAngles. The
            %   NumTransmitAntennas and NumReceiveAntennas fields of the info
            %   method output structure are also swapped. The
            %   SpatialCorrelationMatrix property and corresponding field of
            %   the info method output structure are also rearranged to swap
            %   matrix elements related to transmit and receive antennas.
        end

        function out=validateInputsImpl(~) %#ok<STOUT>
        end

        function out=validatePropertiesImpl(~) %#ok<STOUT>
        end

    end
    properties
        %AveragePathGains Average path gain vector (dB)
        %   Specify the average gains of the discrete paths in deciBels as
        %   a double-precision, real, scalar or row vector.
        %   AveragePathGains must have the same size as PathDelays. This
        %   property applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0.
        AveragePathGains;

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

        %ChannelResponseOutput Specify the type of the returned channel response
        %   When this property is set to 'path-gains', the step method
        %   returns the channel path gains and the corresponding sample
        %   times. Set this property to 'ofdm-response' to make the step
        %   method return the channel OFDM response and the timing offset.
        %
        %   The default value of this property is 'path-gains'.
        ChannelResponseOutput;

        %DelayProfile TDL delay profile
        %   Specify the TDL delay profile as one of 'TDL-A', 'TDL-B',
        %   'TDL-C', 'TDL-D', 'TDL-E', 'TDLA30', 'TDLB100', 'TDLC300',
        %   'TDLC60', 'TDLD30', 'TDLA10', 'TDLD10', 'NTN-TDL-A',
        %   'NTN-TDL-B', 'NTN-TDL-C', 'NTN-TDL-D', 'NTN-TDLA100',
        %   'NTN-TDLC5', or 'Custom'. Delay profiles 'TDL-A' to 'TDL-E' are
        %   defined in TR 38.901 Section 7.7.2, Tables 7.7.2-1 to 7.7.2-5.
        %   Delay profiles 'TDLA30', 'TDLB100', 'TDLC300', 'TDLA10', and
        %   'TDLD10' are defined in TS 38.101-4 Annex B.2.1 and TS 38.104
        %   Annex G.2.1. Delay profiles 'TDLC60' and 'TDLD30' are defined
        %   in TS 38.101-4 Annex B.2.1. When you set this property to
        %   'Custom', the delay profile is configured using the PathDelays,
        %   AveragePathGains, FadingDistribution and KFactorFirstTap
        %   properties.
        %
        %   The default value of this property is 'TDL-A'.
        DelayProfile;

        %DelaySpread Desired delay spread (s)
        %   Specify the desired RMS delay spread in seconds (DS_desired) as
        %   a scalar. See TR 38.901 Section 7.7.3, and Tables 7.7.3-1 and
        %   7.7.3-2 for examples of desired RMS delay spreads. This
        %   property applies when you set the DelayProfile property to
        %   'TDL-A', 'TDL-B', 'TDL-C', 'TDL-D', 'TDL-E', 'NTN-TDL-A',
        %   'NTN-TDL-B', 'NTN-TDL-C', or 'NTN-TDL-D'.
        %
        %   The default value of this property is 30e-9.
        DelaySpread;

        %FadingDistribution Fading process statistical distribution
        %   Specify the fading distribution of the channel as one of
        %   'Rayleigh' or 'Rician'. This property applies when DelayProfile
        %   is set to 'Custom'.
        %
        %   The default value of this property is 'Rayleigh' (the
        %   channel is Rayleigh fading).
        FadingDistribution;

        %InitialTime Start time of the fading process (s)
        %   Specify the time offset of the fading process as a real
        %   nonnegative scalar.
        %
        %   The default value of this property is 0.
        InitialTime;

        %KFactor Desired Rician K-factor (dB)
        %   Specify the desired K-factor in dB (K_desired) as a scalar.
        %   This property applies when you set the KFactorScaling property
        %   to true. See TR 38.901 Section 7.7.6, and see Table 7.5-6 for
        %   typical K-factors. Note that K-factor scaling modifies both the
        %   path delays and path powers. Note that the K-factor applies to
        %   the overall delay profile. The K-factor after the scaling is
        %   K_model described in TR 38.901 Section 7.7.6, the ratio of the
        %   power of the LOS part of the first path to the total power of
        %   all the Rayleigh paths, including the Rayleigh part of the
        %   first path.
        %
        %   The default value of this property is 9.0 dB.
        KFactor;

        %KFactorFirstTap K-factor of first tap (dB)
        %   Specify the K-factor of the first tap of the delay profile in
        %   dB (K_1) as a scalar. This property applies when DelayProfile
        %   is set to 'Custom' and FadingDistribution is set to 'Rician'.
        %
        %   The default value of this property is 13.3 dB. This is the
        %   value defined for delay profile TDL-D.
        KFactorFirstTap;

        %KFactorScaling Apply K-factor scaling (logical)
        %   Set this property to true to apply K-factor scaling as
        %   described in TR 38.901 Section 7.7.6. Note that K-factor
        %   scaling modifies both the path delays and path powers. This
        %   property applies if DelayProfile is set to 'TDL-D', 'TDL-E',
        %   'NTN-TDL-C', or 'NTN-TDL-D'. When you set this property to
        %   true, the desired K-factor is set using the KFactor property.
        %
        %   The default value of this property is false.
        KFactorScaling;

        %MIMOCorrelation Correlation between UE and BS antennas
        %   Specify the desired MIMO correlation as one of 'Low', 'Medium',
        %   'Medium-A', 'UplinkMedium', 'High' or 'Custom'. Other than
        %   'Custom', the values correspond to MIMO correlation levels
        %   defined in TS 36.101 and TS 36.104. The 'Low' and 'High'
        %   correlation levels are the same for both uplink and downlink
        %   and are therefore applicable to both TS 36.101 and TS 36.104.
        %   Note that 'Low' correlation is equivalent to no correlation
        %   between antennas. The 'Medium' and 'Medium-A' correlation
        %   levels are defined in TS 36.101 Annex B.2.3.2 for
        %   TransmissionDirection = 'Downlink'. The 'Medium' correlation
        %   level is defined in TS 36.104 Annex B.5.2 for
        %   TransmissionDirection = 'Uplink'. When you set this property to
        %   'Custom', the correlation between UE antennas is specified
        %   using the ReceiveCorrelationMatrix property and the correlation
        %   between BS antennas is specified using the
        %   TransmitCorrelationMatrix property. See TR 38.901 Section
        %   7.7.5.2.
        %
        %   The default value of this property is 'Low'.
        MIMOCorrelation;

        %MaximumDopplerShift Maximum Doppler shift (Hz)
        %   Specify the maximum Doppler shift due to the scattering
        %   environment around the user equipment (UE) for all channel
        %   paths in Hertz as a double precision real nonnegative scalar
        %   value. This property controls the Doppler spread of the
        %   channel. When this property is set to 0, there is no Doppler
        %   spread and the channel assumes that the UE is static.
        %
        %   The default value of this property is 5 Hz.
        MaximumDopplerShift;

        %NormalizeChannelOutputs Normalize channel outputs by the number of receive antennas (logical)
        %   Set this property to true to normalize the channel outputs by
        %   the number of receive antennas. When you set this property to
        %   false, there is no normalization for channel outputs.
        %
        %   The default value of this property is true.
        NormalizeChannelOutputs;

        %NormalizePathGains Normalize path gains to total power of 0 dB (logical)
        %   Set this property to true to normalize the fading processes
        %   such that the total power of the path gains, averaged over
        %   time, is 0 dB. When you set this property to false, there is no
        %   normalization on path gains. The average powers of the path
        %   gains are specified by the selected delay profile or by the
        %   AveragePathGains property if DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is true.
        NormalizePathGains;

        %NumReceiveAntennas Number of receive antennas
        %   Specify the number of receive antennas as a numeric, real,
        %   positive integer scalar. This property applies when you set the
        %   MIMOCorrelation property to 'Low', 'Medium', 'Medium-A',
        %   'UplinkMedium' or 'High'.
        %
        %   The default value of this property is 2.
        NumReceiveAntennas;

        %NumSinusoids Number of sinusoids used to model the fading process
        %   Specify the number of sinusoids used to model the channel as a
        %   positive integer scalar.
        %
        %   The default value of this property is 48.
        NumSinusoids;

        %NumTimeSamples Number of time samples
        %   Specify the number of time samples used to set the duration of
        %   the fading process realization as a positive integer scalar.
        %   This property applies when ChannelFiltering is false. This
        %   property is tunable.
        %
        %   The default value of this property is 30720.
        NumTimeSamples;

        %NumTransmitAntennas Number of transmit antennas
        %   Specify the number of transmit antennas as a numeric, real,
        %   positive integer scalar. This property applies when you set the
        %   MIMOCorrelation property to 'Low', 'Medium', 'Medium-A',
        %   'UplinkMedium' or 'High', or when both the MIMOCorrelation and
        %   Polarization properties are set to 'Custom'.
        %
        %   The default value of this property is 1.
        NumTransmitAntennas;

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

        %PathGainSampleRate path gain generation sample rate
        %   To use the same sample rate as the input signal (SampleRate),
        %   set this property to 'signal'. To use a lower rate
        %   automatically chosen based on MaximumDopplerShift, set this
        %   property to 'auto'.
        PathGainSampleRate;

        %Polarization Antenna polarization arrangement
        %   Specify the antenna polarization arrangement as one of
        %   'Co-Polar', 'Cross-Polar' or 'Custom'. 
        %
        %   The default value of this property is 'Co-Polar'.
        Polarization;

        %RandomStream Source of random number stream
        %   Specify the source of the random number stream as one of 
        %   'Global stream' or 'mt19937ar with seed'. The channel generates
        %   uniformly distributed random numbers to initialize the sinusoid
        %   phases. When RandomStream is 'Global stream', the channel
        %   generates random numbers using the current global random number
        %   stream. In this case, the reset method only resets the filters.
        %   When RandomStream is 'mt19937ar with seed', the channel
        %   generates random numbers using the mt19937ar algorithm. In this
        %   case, the reset method not only resets the filters but also
        %   reinitializes the random number stream to the value of the Seed
        %   property. Set RandomStream to 'mt19937ar with seed' to produce
        %   repeatable channel fading.
        %
        %   The default value of this property is 'mt19937ar with seed'.
        RandomStream;

        %ReceiveCorrelationMatrix Receive spatial correlation matrix (or 3D array)
        %   Specify the spatial correlation of the receiver as a double
        %   precision, real or complex, 2-D matrix or 3-D array. This
        %   property applies when you set the MIMOCorrelation property to
        %   'Custom' and the Polarization property to 'Co-Polar' or
        %   'Cross-Polar'. The first dimension of ReceiveCorrelationMatrix
        %   should be the same as the number of receive antennas Nr. If the
        %   channel is frequency-flat (PathDelays is a scalar),
        %   ReceiveCorrelationMatrix is a 2-D Hermitian matrix of size
        %   Nr-by-Nr. The main diagonal elements must be all ones, while
        %   the off-diagonal elements must be real or complex numbers with
        %   a magnitude smaller than or equal to one.
        %  
        %   If the channel is frequency-selective (PathDelays is a row
        %   vector of length Np), ReceiveCorrelationMatrix can be specified
        %   as a 2-D matrix, in which case each path has the same receive
        %   spatial correlation matrix. Alternatively, it can be specified
        %   as a 3-D array of size Nr-by-Nr-by-Np, in which case each path
        %   can have its own different receive spatial correlation matrix.
        % 
        %   The default value of this property is [1 0; 0 1].
        ReceiveCorrelationMatrix;

        %ReceivePolarizationAngles Receive polarization slant angles in degrees
        %   Specify the receiver antenna polarization angles, in degrees,
        %   as a double-precision row vector. This property applies when
        %   MIMOCorrelation is set to 'Custom' and Polarization is set to
        %   'Cross-Polar'.
        %
        %   The default value of this property is [90 0].
        ReceivePolarizationAngles;

        %SampleRate Sample rate (Hz)
        %   Specify the sample rate of the input signal in Hz as a double
        %   precision, real, positive scalar.
        %
        %   The default value of this property is 30.72e6 Hz.
        SampleRate;

        %SatelliteDopplerShift Doppler shift due to satellite movement (Hz)
        %   Specify the Doppler shift due to satellite movement for all
        %   channel taps in Hertz as a double precision real scalar value.
        %   This value is calculated based on the satellite altitude,
        %   satellite elevation angle, carrier frequency, and satellite
        %   velocity. This property is tunable. This property applies when
        %   you set the DelayProfile property to 'NTN-TDL-A', 'NTN-TDL-B',
        %   'NTN-TDL-C', 'NTN-TDL-D', 'NTN-TDLA100', or 'NTN-TDLC5'.
        %
        %   The default value is 0 Hz, which corresponds to the Doppler
        %   shift due to a satellite with an elevation angle of 90 degrees.
        SatelliteDopplerShift;

        %Seed Initial seed of mt19937ar random number stream
        %   Specify the initial seed of a mt19937ar random number generator
        %   algorithm as a double precision, real, nonnegative integer
        %   scalar. This property applies when you set the RandomStream
        %   property to 'mt19937ar with seed'. The Seed reinitializes the
        %   mt19937ar random number stream in the reset method.
        %
        %   The default value of this property is 73.
        Seed;

        %SpatialCorrelationMatrix Combined correlation matrix (or 3-D array)
        %   Specify the combined spatial correlation for the channel as a
        %   double precision, 2-D matrix or 3-D array. This property
        %   applies when you set the MIMOCorrelation property to 'Custom'
        %   and the Polarization property to 'Custom'. The first dimension
        %   of SpatialCorrelationMatrix determines the product of the
        %   number of transmit antennas Nt and the number of receive
        %   antennas Nr. If the channel is frequency-flat (PathDelays is a
        %   scalar), SpatialCorrelationMatrix is a 2-D Hermitian matrix of
        %   size NtNr-by-NtNr. The magnitude of any off-diagonal element
        %   must be no larger than the geometric mean of the two
        %   corresponding diagonal elements.
        %  
        %   If the channel is frequency-selective (PathDelays is a row
        %   vector of length Np), SpatialCorrelationMatrix can be specified
        %   as a 2-D matrix, in which case each path has the same spatial
        %   correlation matrix. Alternatively, it can be specified as a 3-D
        %   array of size NtNr-by-NtNr-by-Np, in which case each path can
        %   have its own different spatial correlation matrix.
        % 
        %   The default value of this property is [1 0; 0 1].
        SpatialCorrelationMatrix;

        %TransmissionDirection Transmission direction (Uplink/Downlink)
        %   Specify the transmission direction as one of 'Downlink' |
        %   'Uplink'. This property applies when you set the
        %   MIMOCorrelation property to 'Low', 'Medium', 'Medium-A',
        %   'UplinkMedium', or 'High'. Note that this property describes
        %   the transmission direction when the TransmitAndReceiveSwapped
        %   property is false. The opposite transmission direction applies
        %   when the TransmitAndReceiveSwapped property is true.
        %
        %   The default value of this property is 'Downlink'.
        TransmissionDirection;

        %TransmitAndReceiveSwapped Transmit and receive antennas swapped (logical)
        %   This property indicates if the transmit and receive antennas in 
        %   the channel are swapped. To toggle the state of this property,
        %   call the <a href="matlab:help nrTDLChannel/swapTransmitAndReceive"
        %   >swapTransmitAndReceive</a> method.
        TransmitAndReceiveSwapped;

        %TransmitCorrelationMatrix Transmit spatial correlation matrix (or 3D array)
        %   Specify the spatial correlation of the transmitter as a double
        %   precision, real or complex, 2-D matrix or 3-D array. This
        %   property applies when you set the MIMOCorrelation property to
        %   'Custom' and the Polarization property to 'Co-Polar' or
        %   'Cross-Polar'. The first dimension of TransmitCorrelationMatrix
        %   should be the same as the number of transmit antennas Nt. If
        %   the channel is frequency-flat (PathDelays is a scalar),
        %   TransmitCorrelationMatrix is a 2-D Hermitian matrix of size
        %   Nt-by-Nt. The main diagonal elements must be all ones, while
        %   the off-diagonal elements must be real or complex numbers with
        %   a magnitude smaller than or equal to one.
        %
        %   If the channel is frequency-selective (PathDelays is a row
        %   vector of length Np), TransmitCorrelationMatrix can be
        %   specified as a 2-D matrix, in which case each path has the same
        %   transmit spatial correlation matrix. Alternatively, it can be
        %   specified as a 3-D array of size Nt-by-Nt-by-Np, in which case
        %   each path can have its own different transmit spatial
        %   correlation matrix.
        %
        %   The default value of this property is [1].
        TransmitCorrelationMatrix;

        %TransmitPolarizationAngles Transmit polarization slant angles in degrees
        %   Specify the transmitter antenna polarization angles, in
        %   degrees, as a double-precision row vector. This property
        %   applies when MIMOCorrelation is set to 'Custom' and
        %   Polarization is set to 'Cross-Polar'.
        %
        %   The default value of this property is [45 -45].
        TransmitPolarizationAngles;

        %XPR Cross polarization power ratio (dB)
        %   Specify the cross-polarization power ratio in dB as a scalar or
        %   row vector. The XPR is defined as used in the Clustered Delay
        %   Line (CDL) models in TR 38.901 Section 7.7.1, where the XPR is
        %   the ratio between the vertical-to-vertical and
        %   vertical-to-horizontal polarizations (P_vv/P_vh). Therefore the
        %   XPR in dB is zero or greater. This property applies when
        %   MIMOCorrelation is set to 'Custom' and Polarization is set to
        %   'Cross-Polar'.
        %
        %   If the channel is frequency-selective (PathDelays is a row
        %   vector of length Np), XPR can be specified as a scalar, in
        %   which case each path has the same XPR. Alternatively, it can be
        %   specified as a vector of size 1-by-Np, in which case each path
        %   can have its own different XPR.
        %
        %   The default value of this property is 10.0 dB.
        XPR;

    end
end
