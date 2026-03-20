classdef (StrictDefaults)nrTDLChannel < matlab.System
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
    
%#codegen

% =========================================================================
%   public interface

    methods (Access = public)
        
        % nrTDLChannel constructor
        function obj = nrTDLChannel(varargin)
            
            % Set property values from any name-value pairs input to the
            % constructor
            setProperties(obj,nargin,varargin{:});
            
        end
        
        function h = getPathFilters(obj)
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

            if isempty(coder.target) && isLocked(obj)
                if isempty(obj.pathFilters)
                    % Cache path filters only if the channel is locked.
                    % Otherwise, a change in a property could result in
                    % inconsistent path filters
                    obj.pathFilters = nrTDLChannel.makePathFilters(obj);
                end
                h = obj.pathFilters;
            else
                h = nrTDLChannel.makePathFilters(obj);
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
        %   of link direction: NumTransmitAntennas and NumReceiveAntennas,
        %   TransmitCorrelationMatrix and ReceiveCorrelationMatrix,
        %   TransmitPolarizationAngles and ReceivePolarizationAngles. The
        %   NumTransmitAntennas and NumReceiveAntennas fields of the info
        %   method output structure are also swapped. The
        %   SpatialCorrelationMatrix property and corresponding field of
        %   the info method output structure are also rearranged to swap
        %   matrix elements related to transmit and receive antennas.

            obj.TransmitAndReceiveSwapped = ~obj.TransmitAndReceiveSwapped;
        
            % reset relevant channel filter
            if (~obj.TransmitAndReceiveSwapped)
                reset(obj.channelFilter);
            else
                reset(obj.channelFilterReciprocal);
            end
            
        end
        
    end
    
    % public properties 
    properties (Access = public, Nontunable)
        
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
        DelayProfile = 'TDL-A';

        %PathDelays Discrete path delay vector (s)
        %   Specify the delays of the discrete paths in seconds as a
        %   double-precision, real, scalar or row vector. This property
        %   applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0. 
        PathDelays = 0.0;
        
        %AveragePathGains Average path gain vector (dB)
        %   Specify the average gains of the discrete paths in deciBels as
        %   a double-precision, real, scalar or row vector.
        %   AveragePathGains must have the same size as PathDelays. This
        %   property applies when DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is 0.0. 
        AveragePathGains = 0.0;

        %FadingDistribution Fading process statistical distribution
        %   Specify the fading distribution of the channel as one of
        %   'Rayleigh' or 'Rician'. This property applies when DelayProfile
        %   is set to 'Custom'.
        %
        %   The default value of this property is 'Rayleigh' (the
        %   channel is Rayleigh fading). 
        FadingDistribution = 'Rayleigh';
        
        %KFactorFirstTap K-factor of first tap (dB)
        %   Specify the K-factor of the first tap of the delay profile in
        %   dB (K_1) as a scalar. This property applies when DelayProfile
        %   is set to 'Custom' and FadingDistribution is set to 'Rician'.
        %
        %   The default value of this property is 13.3 dB. This is the
        %   value defined for delay profile TDL-D.
        KFactorFirstTap = 13.3;
        
        %DelaySpread Desired delay spread (s)
        %   Specify the desired RMS delay spread in seconds (DS_desired) as
        %   a scalar. See TR 38.901 Section 7.7.3, and Tables 7.7.3-1 and
        %   7.7.3-2 for examples of desired RMS delay spreads. This
        %   property applies when you set the DelayProfile property to
        %   'TDL-A', 'TDL-B', 'TDL-C', 'TDL-D', 'TDL-E', 'NTN-TDL-A',
        %   'NTN-TDL-B', 'NTN-TDL-C', or 'NTN-TDL-D'.
        %
        %   The default value of this property is 30e-9.
        DelaySpread = 30e-9;
    end

    properties (Access = public)
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
        SatelliteDopplerShift = 0;
    end

    properties (Access = public, Nontunable)
        %MaximumDopplerShift Maximum Doppler shift (Hz)
        %   Specify the maximum Doppler shift due to the scattering
        %   environment around the user equipment (UE) for all channel
        %   paths in Hertz as a double precision real nonnegative scalar
        %   value. This property controls the Doppler spread of the
        %   channel. When this property is set to 0, there is no Doppler
        %   spread and the channel assumes that the UE is static.
        %
        %   The default value of this property is 5 Hz.
        MaximumDopplerShift = 5.0;
        
        %KFactorScaling Apply K-factor scaling (logical)
        %   Set this property to true to apply K-factor scaling as
        %   described in TR 38.901 Section 7.7.6. Note that K-factor
        %   scaling modifies both the path delays and path powers. This
        %   property applies if DelayProfile is set to 'TDL-D', 'TDL-E',
        %   'NTN-TDL-C', or 'NTN-TDL-D'. When you set this property to
        %   true, the desired K-factor is set using the KFactor property.
        %
        %   The default value of this property is false.
        KFactorScaling (1, 1) logical = false;
        
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
        KFactor = 9.0;
        
        %SampleRate Sample rate (Hz)
        %   Specify the sample rate of the input signal in Hz as a double
        %   precision, real, positive scalar.
        %
        %   The default value of this property is 30.72e6 Hz.
        SampleRate = 30.72e6;
        
        %PathGainSampleRate path gain generation sample rate
        %   To use the same sample rate as the input signal (SampleRate),
        %   set this property to 'signal'. To use a lower rate
        %   automatically chosen based on MaximumDopplerShift, set this
        %   property to 'auto'.
        PathGainSampleRate = 'signal'
        
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
        MIMOCorrelation = 'Low';
        
        %Polarization Antenna polarization arrangement
        %   Specify the antenna polarization arrangement as one of
        %   'Co-Polar', 'Cross-Polar' or 'Custom'. 
        %
        %   The default value of this property is 'Co-Polar'.
        Polarization = 'Co-Polar';
        
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
        TransmissionDirection = 'Downlink';
        
    end
        
    properties (Access = public, Dependent, Nontunable)
            
        %NumTransmitAntennas Number of transmit antennas
        %   Specify the number of transmit antennas as a numeric, real,
        %   positive integer scalar. This property applies when you set the
        %   MIMOCorrelation property to 'Low', 'Medium', 'Medium-A',
        %   'UplinkMedium' or 'High', or when both the MIMOCorrelation and
        %   Polarization properties are set to 'Custom'.
        %
        %   The default value of this property is 1.
        NumTransmitAntennas;
        
        %NumReceiveAntennas Number of receive antennas
        %   Specify the number of receive antennas as a numeric, real,
        %   positive integer scalar. This property applies when you set the
        %   MIMOCorrelation property to 'Low', 'Medium', 'Medium-A',
        %   'UplinkMedium' or 'High'.
        %
        %   The default value of this property is 2.
        NumReceiveAntennas;
        
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
        
        %TransmitPolarizationAngles Transmit polarization slant angles in degrees
        %   Specify the transmitter antenna polarization angles, in
        %   degrees, as a double-precision row vector. This property
        %   applies when MIMOCorrelation is set to 'Custom' and
        %   Polarization is set to 'Cross-Polar'.
        %
        %   The default value of this property is [45 -45].        
        TransmitPolarizationAngles;

        %ReceivePolarizationAngles Receive polarization slant angles in degrees
        %   Specify the receiver antenna polarization angles, in degrees,
        %   as a double-precision row vector. This property applies when
        %   MIMOCorrelation is set to 'Custom' and Polarization is set to
        %   'Cross-Polar'.
        %
        %   The default value of this property is [90 0].
        ReceivePolarizationAngles;
        
    end
        
    properties (Access = public, Nontunable)
        
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
        XPR = 10.0;
        
    end

    properties (Access = public, Dependent, Nontunable)
            
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
        
    end
    
    properties (Access = public, Nontunable)
            
        %NormalizePathGains Normalize path gains to total power of 0 dB (logical)
        %   Set this property to true to normalize the fading processes
        %   such that the total power of the path gains, averaged over
        %   time, is 0 dB. When you set this property to false, there is no
        %   normalization on path gains. The average powers of the path
        %   gains are specified by the selected delay profile or by the
        %   AveragePathGains property if DelayProfile is set to 'Custom'.
        %
        %   The default value of this property is true. 
        NormalizePathGains (1, 1) logical = true;
        
        %InitialTime Start time of the fading process (s)
        %   Specify the time offset of the fading process as a real
        %   nonnegative scalar.
        %
        %   The default value of this property is 0.
        InitialTime = 0.0;
        
        %NumSinusoids Number of sinusoids used to model the fading process
        %   Specify the number of sinusoids used to model the channel as a
        %   positive integer scalar.
        %
        %   The default value of this property is 48.
        NumSinusoids = 48;
        
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
        %   call the <a href="matlab:help nrTDLChannel/swapTransmitAndReceive"
        %   >swapTransmitAndReceive</a> method.
        TransmitAndReceiveSwapped (1, 1) logical = false;
        
    end

    properties (Access = public, Dependent, Nontunable)
        %ChannelResponseOutput Specify the type of the returned channel response
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
            obj.PathDelays = val;
        end
        
        function set.AveragePathGains(obj,val)
            propName = 'AveragePathGains';
            validateattributes(val,{'double'},{'real','row','finite'},[class(obj) '.' propName], propName);
            obj.AveragePathGains = val;
        end
        
        function set.KFactorFirstTap(obj,val)
            propName = 'KFactorFirstTap';
            validateattributes(val,{'double'},{'real','scalar','finite'},[class(obj) '.' propName],propName);
            obj.KFactorFirstTap = val;
        end
        
        function set.DelaySpread(obj,val)
            propName = 'DelaySpread';
            validateattributes(val,{'double'},{'real','scalar','nonnegative','finite'},[class(obj) '.' propName],propName);
            obj.DelaySpread = val;
        end

        function set.SatelliteDopplerShift(obj,val)
            propName = 'SatelliteDopplerShift';
            validateattributes(val,{'double'},{'real','scalar','finite'},[class(obj) '.' propName],propName);
            obj.(propName) = val;
        end

        function set.MaximumDopplerShift(obj,val)
            propName = 'MaximumDopplerShift';
            validateattributes(val,{'double'},{'real','scalar','nonnegative','finite'},[class(obj) '.' propName],propName);
            obj.MaximumDopplerShift = val;
        end
        
        function set.KFactor(obj,val)
            propName = 'KFactor';
            validateattributes(val,{'double'},{'real','scalar','finite'},[class(obj) '.' propName],propName);
            obj.KFactor = val;
        end
        
        function set.SampleRate(obj,val)
            propName = 'SampleRate';
            validateattributes(val,{'double'},{'real','scalar','positive','finite'},[class(obj) '.' propName],propName);
            obj.SampleRate = val;
        end
        
        function set.NumTransmitAntennas(obj,val)
            propName = 'NumTransmitAntennas';
            validateattributes(val,{'numeric'},{'real','scalar','integer','>=',1},[class(obj) '.' propName],propName);
            if (isempty(coder.target))
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theNumTransmitAntennas = val;
                else
                    obj.theNumReceiveAntennas = val;
                end
            else
                obj.theNumTransmitAntennas = val;
            end
        end
        
        function set.NumReceiveAntennas(obj,val)
            propName = 'NumReceiveAntennas';
            validateattributes(val,{'numeric'},{'real','scalar','integer','>=',1},[class(obj) '.' propName],propName);
            if (isempty(coder.target))
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theNumReceiveAntennas = val;
                else
                    obj.theNumTransmitAntennas = val;
                end
            else
                obj.theNumReceiveAntennas = val;
            end
        end
        
        function set.TransmitCorrelationMatrix(obj,val)
            propName = 'TransmitCorrelationMatrix';
            validateattributes(val,{'double'},{'finite','nonempty'},[class(obj) '.' propName],propName);
            coder.internal.errorIf(ndims(val)>3,'nr5g:nrTDLChannel:CorrMtxMoreThan3D','TransmitCorrelationMatrix');
            if (isempty(coder.target))
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theTransmitCorrelationMatrix = val;
                else
                    obj.theReceiveCorrelationMatrix = val;
                end
            else
                obj.theTransmitCorrelationMatrix = val;
            end
        end
        
        function set.ReceiveCorrelationMatrix(obj,val)
            propName = 'ReceiveCorrelationMatrix';
            validateattributes(val,{'double'},{'finite','nonempty'},[class(obj) '.' propName],propName);
            coder.internal.errorIf(ndims(val)>3,'nr5g:nrTDLChannel:CorrMtxMoreThan3D','ReceiveCorrelationMatrix');
            if (isempty(coder.target))
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theReceiveCorrelationMatrix = val;
                else
                    obj.theTransmitCorrelationMatrix = val;
                end
            else
                obj.theReceiveCorrelationMatrix = val;
            end
        end
        
        function set.TransmitPolarizationAngles(obj,val)
            propName = 'TransmitPolarizationAngles';
            validateattributes(val,{'double'},{'real','row','finite'},[class(obj) '.' propName], propName);
            coder.internal.errorIf(size(val,2)>2,'nr5g:nrTDLChannel:InvalidNumPolAngles','TransmitPolarizationAngles');
            if (isempty(coder.target))
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theTransmitPolarizationAngles = val;
                else
                    obj.theReceivePolarizationAngles = val;
                end
            else
                obj.theTransmitPolarizationAngles = val;
            end
        end
        
        function set.ReceivePolarizationAngles(obj,val)
            propName = 'ReceivePolarizationAngles';
            validateattributes(val,{'double'},{'real','row','finite'},[class(obj) '.' propName], propName);
            coder.internal.errorIf(size(val,2)>2,'nr5g:nrTDLChannel:InvalidNumPolAngles','ReceivePolarizationAngles');
            if (isempty(coder.target))
                if (~obj.TransmitAndReceiveSwapped)
                    obj.theReceivePolarizationAngles = val;
                else
                    obj.theTransmitPolarizationAngles = val;
                end
            else
                obj.theReceivePolarizationAngles = val;
            end
        end
        
        function set.XPR(obj,val)
            propName = 'XPR';
            validateattributes(val,{'double'},{'real','row','nonnegative'},[class(obj) '.' propName],propName);
            obj.XPR = val;
        end
        
        function set.SpatialCorrelationMatrix(obj,val)
            propName = 'SpatialCorrelationMatrix';
            validateattributes(val,{'double'},{'finite','nonempty'},[class(obj) '.' propName],propName);
            coder.internal.errorIf(ndims(val)>3,'nr5g:nrTDLChannel:CorrMtxMoreThan3D','SpatialCorrelationMatrix');
            if (isempty(coder.target))
                if (obj.TransmitAndReceiveSwapped)
                    Nt = obj.theNumTransmitAntennas;
                    Nr = size(obj.theSpatialCorrelationMatrix,1) / Nt;
                    val = nrTDLChannel.makeReciprocalSpatialCorrelationMatrix(val,Nr);
                end
            end
            obj.theSpatialCorrelationMatrix = val;
        end
        
        function set.InitialTime(obj,val)
            propName = 'InitialTime';
            validateattributes(val,{'double'},{'real','scalar','nonnegative','finite'},[class(obj) '.' propName],propName);
            obj.InitialTime = val;
        end
        
        function set.NumSinusoids(obj,val)
            propName = 'NumSinusoids';
            validateattributes(val,{'numeric'},{'scalar','integer','>=',1},[class(obj) '.' propName],propName);
            obj.NumSinusoids = val;
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
        
    end
    
    % property value sets for enumerated properties
    properties(Hidden,Transient)
        
        DelayProfileSet = matlab.system.StringSet({'TDL-A','TDL-B','TDL-C','TDL-D','TDL-E','TDLA30','TDLB100', ...
            'TDLC300','TDLC60','TDLD30','TDLA10','TDLD10','NTN-TDL-A','NTN-TDL-B','NTN-TDL-C','NTN-TDL-D', ...
            'NTN-TDLA100','NTN-TDLC5','Custom'});
        MIMOCorrelationSet = matlab.system.StringSet({'Low','Medium','Medium-A','UplinkMedium','High','Custom'});
        FadingDistributionSet = matlab.system.StringSet({'Rayleigh','Rician'});
        RandomStreamSet = matlab.system.StringSet({'Global stream','mt19937ar with seed'});
        PolarizationSet = matlab.system.StringSet({'Co-Polar','Cross-Polar','Custom'});
        TransmissionDirectionSet = matlab.system.StringSet({'Downlink','Uplink'});
        OutputDataTypeSet = matlab.system.StringSet({'double','single'});
        ChannelResponseOutputSet = matlab.system.StringSet({'path-gains','ofdm-response'});
        PathGainSampleRateSet = matlab.system.StringSet({'signal','auto'});
        
    end
    
    % public property getters
    methods
       
        function val = get.NumTransmitAntennas(obj)
            val = nrTDLChannel.swapTxRx(obj.theNumTransmitAntennas,obj.theNumReceiveAntennas,obj.TransmitAndReceiveSwapped);
        end
        
        function val = get.NumReceiveAntennas(obj)
            val = nrTDLChannel.swapTxRx(obj.theNumReceiveAntennas,obj.theNumTransmitAntennas,obj.TransmitAndReceiveSwapped);
        end
        
        function val = get.TransmitCorrelationMatrix(obj)
            val = nrTDLChannel.swapTxRx(obj.theTransmitCorrelationMatrix,obj.theReceiveCorrelationMatrix,obj.TransmitAndReceiveSwapped);
        end
        
        function val = get.ReceiveCorrelationMatrix(obj)
            val = nrTDLChannel.swapTxRx(obj.theReceiveCorrelationMatrix,obj.theTransmitCorrelationMatrix,obj.TransmitAndReceiveSwapped);
        end
        
        function val = get.TransmitPolarizationAngles(obj)
            val = nrTDLChannel.swapTxRx(obj.theTransmitPolarizationAngles,obj.theReceivePolarizationAngles,obj.TransmitAndReceiveSwapped);
        end
        
        function val = get.ReceivePolarizationAngles(obj)
            val = nrTDLChannel.swapTxRx(obj.theReceivePolarizationAngles,obj.theTransmitPolarizationAngles,obj.TransmitAndReceiveSwapped);
        end
        
        function val = get.SpatialCorrelationMatrix(obj)
            if (isempty(coder.target))
                if (~obj.TransmitAndReceiveSwapped)
                    val = obj.theSpatialCorrelationMatrix;
                else
                    Nt = obj.theNumTransmitAntennas;
                    val = nrTDLChannel.makeReciprocalSpatialCorrelationMatrix(obj.theSpatialCorrelationMatrix,Nt);
                end
            else
                val = obj.theSpatialCorrelationMatrix;
            end
        end

        function val = get.ChannelResponseOutput(obj)
            if obj.OutputOFDMResponse
                val = 'ofdm-response';
            else
                val = 'path-gains';
            end
        end
        
    end

    methods (Hidden, Access = {?wirelessChannelDesigner.TDLDialog})
      function c = getMIMOChannel(obj)
        c = obj.theChannel;
      end
    end

% =========================================================================
%   protected interface
    
    methods (Access = protected)
        
        % nrTDLChannel setupImpl method
        function setupImpl(obj,varargin)

            % Construct the MIMOChannel. Note that 'theChannel' is
            % applicable to TransmitAndReceiveSwapped = false, with
            % TransmitAndReceiveSwapped = true being handled by permuting
            % the path gains after executing the underlying channel
            if (obj.ChannelFiltering)
                outputDataType = class(varargin{1});
            else
                outputDataType = obj.OutputDataType;
            end
            obj.theTime = obj.InitialTime;
            obj.theChannel = nrTDLChannel.makeMIMOChannel(obj,outputDataType);

            % Initial channel matrix for LOS component, if required
            % TS 38.101-4 Section B.2, TS 38.101-5 Section B.2, or
            % TS 38.521-5 Section B.2
            if (any(strcmp(obj.DelayProfile,{'TDLD30','TDLD10','NTN-TDLC5'})))
                doValidation = false;
                transmitAndReceiveSwapped = false;
                s = getInfo(obj,doValidation,transmitAndReceiveSwapped);
                Nt = s.NumTransmitAntennas;
                Nr = s.NumReceiveAntennas;
                obj.theInitialChannelLOS = permute(nr5g.internal.staticChannelMatrix(Nt,Nr),[3 4 2 1]);
            end

            % Setup channel filters
            if (obj.ChannelFiltering)

                % Determine the best filter option for the current input
                in = varargin{1};
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
                obj.channelFilter = nrTDLChannel.setupChannelFilter(obj,optimizeScalarGainFilter);
                obj.channelFilterReciprocal = nrTDLChannel.setupChannelFilter(obj,optimizeScalarGainFilter);

            end

            % Validate the dependency of Doppler and sample rate
            nrTDLChannel.validateSatelliteDoppler(obj);

        end

        % nrTDLChannel validateInputsImpl method
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

        % nrTDLChannel stepImpl method
        function varargout = stepImpl(obj,varargin)

            % Possible number of inputs (in addition to system object)
            %   obj.step():           ChannelFiltering = false, ChannelResponseOutput = 'path-gains'
            %   obj.step(in):         ChannelFiltering = true,  ChannelResponseOutput = 'path-gains'
            %   obj.step(carrier):    ChannelFiltering = false, ChannelResponseOutput = 'ofdm-response'
            %   obj.step(in,carrier): ChannelFiltering = true,  ChannelResponseOutput = 'ofdm-response'

            if obj.ChannelFiltering % step(in, __)
                in = varargin{1};
                validateInputSignal(obj,in);
                % Get the number of signal samples and data type from input
                numTimeSamples = size(in,1);
                outputtype = class(in);
                if nargin>2 % step(in, carrier)
                    carrier = varargin{2};
                    validateattributes(carrier,{'nrCarrierConfig'},{'scalar'},'nrTDLChannel','Carrier specific configuration object');
                    validateCarrierSampleRate(obj,carrier);
                end
            else % step(__)
                % Get the number of equivalent signal samples and data type
                % from property values
                numTimeSamples = obj.NumTimeSamples;
                outputtype = obj.OutputDataType;
                if nargin>1 % step(carrier)
                    carrier = varargin{1};
                    validateattributes(carrier,{'nrCarrierConfig'},{'scalar'},'nrTDLChannel','Carrier specific configuration object');
                    validateCarrierSampleRate(obj,carrier);
                end
            end

            % Calculate the number of path gain samples to generate
            obj.theChannel.NumSamples = obj.getNumPathGainSamples(numTimeSamples);

            % Execute the MIMOChannel to get path gains and sample times
            if strcmpi(obj.PathGainSampleRate,'auto')
                % Allow possible fractional sample period in the previous
                % step call
                pathgains = obj.theChannel(obj.theTime);
                sampletimes = obj.theTime + (0:obj.theChannel.NumSamples-1).'/obj.theChannel.SampleRate;
                obj.theNumSamplesProcessed = obj.theNumSamplesProcessed + obj.theChannel.NumSamples;
                startTime = obj.theTime;
            else
                pathgains = obj.theChannel();
                startTime = obj.InitialTime + obj.theNumSamplesProcessed/obj.SampleRate;
                obj.theNumSamplesProcessed = obj.theNumSamplesProcessed + obj.theChannel.NumSamples;
                Nend = obj.theNumSamplesProcessed;
                Ns = size(pathgains,1);
                sampletimes = obj.InitialTime + (Nend-Ns:Nend-1).'/obj.SampleRate;
            end

            % Apply initial channel matrix for LOS component if required,
            % see TS 38.101-4 Section B.2, TS 38.101-5 Section B.2, or TS
            % 38.521-5 Section B.2. Note that the first element of the
            % second dimension of pathGains includes both the LOS and NLOS
            % parts of the first tap produced by obj.theChannel, so the
            % application of the initial channel matrix here affects the
            % NLOS part. This is not specified in TS 38.101-4, but amounts
            % to a random phase shift of elements already having uniformly
            % distributed random phase, so it does not affect the nature of
            % the channel model
            if (~isempty(obj.theInitialChannelLOS))
                pathgains(:,1,:,:) = pathgains(:,1,:,:) .* obj.theInitialChannelLOS;
            end

            % Apply Doppler shift due to satellite for all the paths
            if nrTDLChannel.isNTN(obj)
                % Get the phase shift to be applied at each sample
                dopplerPhase = obj.theSatelliteDopplerPhase + ...
                    2*pi*(1/obj.theChannel.SampleRate)*obj.SatelliteDopplerShift * (0:obj.theChannel.NumSamples)';
                obj.theSatelliteDopplerPhase = dopplerPhase(end,1);
                % Apply the phase shifts to the path gains
                pathgains = pathgains .* exp(1j*dopplerPhase(1:(end-1),1));
            end

            % Update path gains to give reciprocal channel if required
            if (obj.TransmitAndReceiveSwapped)
                % Permute to swap transmit and receive antennas
                x = permute(pathgains,[1 2 4 3]);
                % Re-normalize channel outputs if required
                if (obj.NormalizeChannelOutputs)
                    % Normalization inside obj.theChannel uses the
                    % non-reciprocal receive antenna count, undo this and
                    % normalize by the reciprocal receive antenna count
                    g = x * sqrt(size(x,3)) / sqrt(size(x,4));
                else
                    g = x;
                end
            else
                g = pathgains;
            end

            % Apply channel filtering
            if (obj.ChannelFiltering)
                if (~obj.TransmitAndReceiveSwapped)
                    filterToUse = obj.channelFilter;
                else
                    filterToUse = obj.channelFilterReciprocal;
                end
                sampleDensity = Inf;
                if strcmpi(obj.PathGainSampleRate,"auto")
                    sampleDensity = 64;
                end
                filterOption = 1 + filterToUse.OptimizeScalarGainFilter;
                out = wireless.internal.channelmodels.smartChannelFiltering(in,filterToUse,obj.SampleRate,g,numTimeSamples,sampleDensity,sampletimes-startTime,outputtype,filterOption);
            end

            % Calculate frequency response
            if obj.OutputOFDMResponse % "ofdm-response" mode
                % A minimum of one slot is required for the OFDM channel
                % response
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

            % Advance channel time when PathGainSampleRate='auto'
            if strcmpi(obj.PathGainSampleRate,'auto')
                obj.theTime = obj.theTime + numTimeSamples/obj.SampleRate;
            end

        end
        
        % nrTDLChannel resetImpl method
        function resetImpl(obj)
            
            % reset the MIMOChannel
            reset(obj.theChannel);
            obj.theNumSamplesProcessed = 0;
            obj.theSatelliteDopplerPhase = 0;
            obj.theTime = obj.InitialTime;
            
            % reset channel filters            
            if (obj.ChannelFiltering)
                reset(obj.channelFilter);
                reset(obj.channelFilterReciprocal);
            end

        end

        % nrTDLChannel releaseImpl method
        function releaseImpl(obj)
            release(obj.theChannel);
            if (obj.ChannelFiltering)
                release(obj.channelFilter);
                release(obj.channelFilterReciprocal);
            end

            if coder.target('MATLAB')
                % Empty path filters as they may not be applicable after
                % changing a property and they would need to be regenerated
                obj.pathFilters = [];
            end
        end

        % nrTDLChannel getNumInputsImpl method
        function num = getNumInputsImpl(obj)
            num = obj.ChannelFiltering+obj.OutputOFDMResponse;
        end
        
        % nrTDLChannel getNumOutputsImpl method
        function num = getNumOutputsImpl(obj)
            num = 2+obj.ChannelFiltering;
        end

        % nrTDLChannel isInputComplexityMutableImpl method
        function flag = isInputComplexityMutableImpl(~,~)            
            flag = true;            
        end

        % nrTDLChannel isInputSizeMutableImpl method
        function flag = isInputSizeMutableImpl(~,~)            
            flag = true;
        end

        % nrTDLChannel infoImpl method
        function s = infoImpl(obj)
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
            
            doValidation = true;
            s = getInfo(obj,doValidation,obj.TransmitAndReceiveSwapped);
            
        end
        
        % nrTDLChannel saveObjectImpl method
        function s = saveObjectImpl(obj)
            
            s = saveObjectImpl@matlab.System(obj);
            s.theInitialChannelLOS = obj.theInitialChannelLOS;
            s.theNumSamplesProcessed = obj.theNumSamplesProcessed;
            s.theChannel = matlab.System.saveObject(obj.theChannel);
            s.theNumTransmitAntennas = obj.theNumTransmitAntennas;
            s.theNumReceiveAntennas = obj.theNumReceiveAntennas;
            s.theTransmitCorrelationMatrix = obj.theTransmitCorrelationMatrix;
            s.theReceiveCorrelationMatrix = obj.theReceiveCorrelationMatrix;
            s.theTransmitPolarizationAngles = obj.theTransmitPolarizationAngles;
            s.theReceivePolarizationAngles = obj.theReceivePolarizationAngles;
            s.theSpatialCorrelationMatrix = obj.theSpatialCorrelationMatrix;
            s.channelFilter = matlab.System.saveObject(obj.channelFilter);
            s.channelFilterReciprocal = matlab.System.saveObject(obj.channelFilterReciprocal);
            s.TransmitAndReceiveSwapped = obj.TransmitAndReceiveSwapped;
            s.theSatelliteDopplerPhase = obj.theSatelliteDopplerPhase;
            s.pathFilters = obj.pathFilters;
            s.OutputOFDMResponse = obj.OutputOFDMResponse;
            s.theTime = obj.theTime;

        end

        % nrTDLChannel loadObjectImpl method
        function loadObjectImpl(obj,s,wasLocked)
            
            if (isfield(s,'TransmitAndReceiveSwapped'))
                % TransmitAndReceiveSwapped property and associated private
                % properties were added in 21a
                obj.TransmitAndReceiveSwapped = s.TransmitAndReceiveSwapped;
                obj.channelFilterReciprocal = matlab.System.loadObject(s.channelFilterReciprocal);
                obj.channelFilter = matlab.System.loadObject(s.channelFilter);
                obj.theSpatialCorrelationMatrix = s.theSpatialCorrelationMatrix;
                obj.theReceivePolarizationAngles = s.theReceivePolarizationAngles;
                obj.theTransmitPolarizationAngles = s.theTransmitPolarizationAngles;
                obj.theReceiveCorrelationMatrix = s.theReceiveCorrelationMatrix;
                obj.theTransmitCorrelationMatrix = s.theTransmitCorrelationMatrix;
                obj.theNumReceiveAntennas = s.theNumReceiveAntennas;
                obj.theNumTransmitAntennas = s.theNumTransmitAntennas;
            end
            obj.theChannel = matlab.System.loadObject(s.theChannel);
            if (isfield(s,'theNumSamplesProcessed'))
                % 'theNumSamplesProcessed' was added in 23a
                obj.theNumSamplesProcessed = s.theNumSamplesProcessed;
            else
                % Set 'theNumSamplesProcessed' from 'theChannel' if saved
                % prior to 23a
                theInfo = info(obj.theChannel);
                obj.theNumSamplesProcessed = theInfo.NumSamplesProcessed;
            end
            if (isfield(s,'theInitialChannelLOS'))
                % 'theInitialChannelLOS' was added in 24a
                obj.theInitialChannelLOS = s.theInitialChannelLOS;
            else
                % Unused, only applies to delay profiles added in 24a
                obj.theInitialChannelLOS = [];
            end
            if (isfield(s,'theSatelliteDopplerPhase'))
                % 'theSatelliteDopplerPhase' was added in 24a
                obj.theSatelliteDopplerPhase = s.theSatelliteDopplerPhase;
            else
                % Unused, set to default
                obj.theSatelliteDopplerPhase = 0;
            end
            if isfield(s,'pathFilters')
                % pathFilters added in 24b
                obj.pathFilters = s.pathFilters;
            else
                obj.pathFilters = [];
            end
            if (isfield(s,'theTime'))
                % 'theTime' was added in 24b
                obj.theTime = s.theTime;
            else
                % Unused, set to default
                obj.theTime = s.InitialTime;
            end

            if isfield(s,'OutputOFDMResponse')
                % OutputOFDMResponse added in 24b
                obj.OutputOFDMResponse = s.OutputOFDMResponse;
            else
                obj.OutputOFDMResponse = false;
            end

            loadObjectImpl@matlab.System(obj,s,wasLocked);

        end
        
        % nrTDLChannel isInactivePropertyImpl method
        function flag = isInactivePropertyImpl(obj,prop)
            
            if (any(strcmp(prop,{'PathDelays','AveragePathGains','FadingDistribution'})))
                flag = ~strcmp(obj.DelayProfile,'Custom');
            elseif (strcmp(prop,'KFactorFirstTap'))
                flag = ~strcmp(obj.DelayProfile,'Custom') || ~strcmp(obj.FadingDistribution,'Rician');
            elseif (strcmp(prop,'KFactorScaling'))
                flag = ~any(strcmp(obj.DelayProfile,{'TDL-D','TDL-E','NTN-TDL-C','NTN-TDL-D'}));
            elseif (any(strcmp(prop,{'TransmitCorrelationMatrix','ReceiveCorrelationMatrix'})))
                flag = ~strcmp(obj.MIMOCorrelation,'Custom') || strcmp(obj.Polarization,'Custom');
            elseif (strcmp(prop,'NumTransmitAntennas'))
                flag = strcmp(obj.MIMOCorrelation,'Custom') && ~strcmp(obj.Polarization,'Custom');
            elseif (any(strcmp(prop,{'NumReceiveAntennas','TransmissionDirection'})))
                flag = strcmp(obj.MIMOCorrelation,'Custom');
            elseif (any(strcmp(prop,{'TransmitPolarizationAngles','ReceivePolarizationAngles','XPR'})))
                flag = ~(strcmp(obj.MIMOCorrelation,'Custom') && strcmp(obj.Polarization,'Cross-Polar'));
            elseif (strcmp(prop,'SpatialCorrelationMatrix'))
                flag = ~(strcmp(obj.MIMOCorrelation,'Custom') && strcmp(obj.Polarization,'Custom'));
            elseif (strcmp(prop,'KFactor'))
                flag = strcmp(obj.DelayProfile,'Custom') || ~(obj.KFactorScaling);
            elseif (strcmp(prop,'Seed'))
                flag = ~strcmp(obj.RandomStream,'mt19937ar with seed');
            elseif (strcmp(prop,'DelaySpread'))
                flag = ~any(strcmp(obj.DelayProfile,{'TDL-A','TDL-B','TDL-C','TDL-D','TDL-E','NTN-TDL-A','NTN-TDL-B','NTN-TDL-C','NTN-TDL-D'}));
            elseif (any(strcmp(prop,{'NumTimeSamples','OutputDataType'})))
                flag = obj.ChannelFiltering;
            elseif (strcmp(prop,'SatelliteDopplerShift'))
                flag = ~(any(strcmp(obj.DelayProfile, ...
                    {'NTN-TDL-A','NTN-TDL-B','NTN-TDL-C','NTN-TDL-D','NTN-TDLA100','NTN-TDLC5'})));
            else
                flag = false;
            end
            
        end
        
        % nrTDLChannel validatePropertiesImpl method
        function validatePropertiesImpl(obj)
            
            nrTDLChannel.validation(obj);
            
        end

        function processTunedPropertiesImpl(obj)
            % Perform actions when tunable properties change
            % between calls to the System object

            if isChangedProperty(obj,'SatelliteDopplerShift')
                nrTDLChannel.validateSatelliteDoppler(obj);
            end
        end

    end

% =========================================================================
%   private

    properties (Access = private, Nontunable)
        
        % The underlying System object used to perform the channel modeling
        theChannel;
        
        % The underlying transmit and receive antenna properties. These are
        % presented on the public interface via similarly-named dependent
        % properties (drop the "the" from the start of the names below) and
        % transmit/receive property pairs (or elements within a property)
        % are switched around when TransmitAndReceiveSwapped = true
        theNumTransmitAntennas = 1;
        theNumReceiveAntennas = 2;
        theTransmitCorrelationMatrix = 1;
        theReceiveCorrelationMatrix = eye(2);
        theTransmitPolarizationAngles = [45 -45];
        theReceivePolarizationAngles = [90 0];
        theSpatialCorrelationMatrix = eye(2);
        
        % Channel filters
        channelFilter;
        channelFilterReciprocal;

        % Path filters
        pathFilters;

        % Control the type of channel response output: OFDM response or path gains
        OutputOFDMResponse = false;

    end

    properties (Access = private)

        % Clock of the channel (This property only takes effect when
        % PathGainSampleRate='auto')
        theTime;

        % The number of samples processed by the channel
        theNumSamplesProcessed = 0;

        % Initial channel matrix for LOS component
        % TS 38.101-4 Section B.2, TS 38.101-5 Section B.2, TS 38.521-5
        % Section B.2
        theInitialChannelLOS = [];

        % The accumulated phase due to satellite Doppler shift
        theSatelliteDopplerPhase = 0;

    end
    
    methods (Access = private)
        
        % validate input signal
        function validateInputSignal(obj,in)
        
            [Nt,Nr] = nrTDLChannel.getAntennaCount(obj);
            if (obj.TransmitAndReceiveSwapped)
                ntxants = Nr;
            else
                ntxants = Nt;
            end
            coder.internal.errorIf(size(in,2) ~= ntxants,'nr5g:nrTDLChannel:SignalInputNotMatchNumTx',size(in,2),ntxants);
            validateattributes(in,{'double','single'},{'2d','finite'},class(obj),'signal input');
            
        end

        % validate sample rate from carrier
        function validateCarrierSampleRate(obj,carrier)
            ofdmInfo = nr5g.internal.OFDMInfo(carrier,[]);
            carrierSR = ofdmInfo.SampleRate;
            coder.internal.errorIf(carrierSR ~= obj.SampleRate,'nr5g:nrTDLChannel:CarrierSampleRateMismatch',obj.SampleRate,carrierSR);
        end

        % get channel info
        function s = getInfo(obj,doValidation,swapped)

            if (~isempty(coder.target) || ~isLocked(obj))
                if (doValidation)
                    nrTDLChannel.validation(obj);
                end
                channel = nrTDLChannel.makeMIMOChannel(obj);
            else
                channel = obj.theChannel;
            end
            [~,channelFilterDelay] = nrTDLChannel.makePathFilters(obj);
            
            s.ChannelFilterDelay = channelFilterDelay;
            % In case PathGainSampleRate='auto' and channel has a different
            % sample rate, always use the signal sample rate to calculate
            % the maximum channel delay
            s.MaximumChannelDelay = ceil(max(channel.PathDelays*obj.SampleRate)) + channelFilterDelay;
            s.PathDelays = channel.PathDelays;
            s.AveragePathGains = channel.AveragePathGains;
            if (nrTDLChannel.hasLOSPath(obj))
                s.KFactorFirstTap = 10*log10(channel.KFactor);
            else
                s.KFactorFirstTap = -Inf;
            end
            Nt = channel.NumTransmitAntennas;
            s.NumTransmitAntennas = Nt;
            s.NumReceiveAntennas = size(channel.SpatialCorrelationMatrix,1) / Nt;
            s.SpatialCorrelationMatrix = channel.SpatialCorrelationMatrix;

            % The number of transmit and receive antennas in 's' are
            % applicable to TransmitAndReceiveSwapped = false, so swap them
            % if TransmitAndReceiveSwapped = true. The spatial correlation
            % matrix also has its transmit and receive antennas swapped
            if (swapped)
                s.NumTransmitAntennas = s.NumReceiveAntennas;
                s.NumReceiveAntennas = Nt;
                s.SpatialCorrelationMatrix = nrTDLChannel.makeReciprocalSpatialCorrelationMatrix(s.SpatialCorrelationMatrix,Nt);
            end

        end

        % validate number of input samples or NumTimeSamples
        function validateNumTimeSamples(obj,numTimeSamples,carrier)
            % Calculate symbol lengths
            ofdmInfo = nr5g.internal.OFDMInfo(carrier,[]);
            symbolLengths = ofdmInfo.CyclicPrefixLengths + ofdmInfo.Nfft;
            slotInSubframe = mod(carrier.NSlot,carrier.SlotsPerSubframe);
            numSamplesInCurrentSlot = sum(symbolLengths(slotInSubframe*ofdmInfo.SymbolsPerSlot + (1:ofdmInfo.SymbolsPerSlot)));
            
            if obj.ChannelFiltering
                coder.internal.errorIf(numTimeSamples< numSamplesInCurrentSlot,'nr5g:nrTDLChannel:NumInputSamplesNotASlotChFiltOn',numTimeSamples,numSamplesInCurrentSlot,carrier.NSlot);
            else
                coder.internal.errorIf(numTimeSamples< numSamplesInCurrentSlot,'nr5g:nrTDLChannel:NumTimeSamplesNotASlotChFiltOff',numTimeSamples,numSamplesInCurrentSlot,carrier.NSlot);
            end
        end
        
        % calculate the number of path gain samples per path per antenna to
        % generate
        function numPathGainSamples = getNumPathGainSamples(obj,numTimeSamples)

            numPathGainSamples = numTimeSamples;
            if strcmpi(obj.PathGainSampleRate,'auto')
                % Try to get fewer path gain samples in this mode to speed
                % up the simulation
                sampleRatio = obj.SampleRate/obj.theChannel.SampleRate;
                if obj.MaximumDopplerShift==0
                    % For static channel, one path gain sample is enough
                    numPathGainSamples = 1;
                else
                    % Calculated the number of samples to get based on the
                    % input signal length, including at least an extra half
                    % sample period beyond the end of the waveform to allow
                    % for zero order hold balancing and capping at input
                    % signal size in case sampleRatio is 1
                    numPathGainSamples = min(numTimeSamples,ceil(numTimeSamples/sampleRatio+0.5));
                end
            end

        end

        % get preferred channel filtering policy for a given input
        function p = getFilterPolicy(obj,in)

            outputtype = class(in);
            T = size(in,1);
            Ncs = obj.getNumPathGainSamples(T);
            Np = numel(obj.theChannel.PathDelays);
            Nt = obj.theNumTransmitAntennas;
            Nr = obj.theNumReceiveAntennas;
            sizg = [Ncs Np Nt Nr];
            p = wireless.internal.channelmodels.getFilterPolicy(outputtype,[sizg T]);

        end

    end
    
    methods (Static, Access = private)
        
        function [pathFilters,filterDelay] = makePathFilters(obj)
            
            if (isempty(coder.target) && obj.ChannelFiltering && isLocked(obj))
                if (~obj.TransmitAndReceiveSwapped)
                    channelInfo = info(obj.channelFilter);
                else
                    channelInfo = info(obj.channelFilterReciprocal);
                end
            else
                nrTDLChannel.validation(obj);
                % Note: optimizeScalarGainFilter is an optimization option,
                % it does not affect the path filters or filter delay
                optimizeScalarGainFilter = false;
                channelFilter = nrTDLChannel.setupChannelFilter(obj,optimizeScalarGainFilter);
                channelInfo = info(channelFilter);
            end
            
            pathFilters = channelInfo.ChannelFilterCoefficients.';
            filterDelay = channelInfo.ChannelFilterDelay;
            
        end
        
        % Construct the MIMOChannel. Note that 'c' is applicable to
        % TransmitAndReceiveSwapped = false, with TransmitAndReceiveSwapped
        % = true being handled by permuting the path gains after executing
        % the underlying channel
        function c = makeMIMOChannel(obj,varargin)
            
            c = comm.MIMOChannel;
            if strcmpi(obj.PathGainSampleRate,'auto') && obj.MaximumDopplerShift>0
                c.SampleRate = min(2*64*obj.MaximumDopplerShift, obj.SampleRate);
            else
                c.SampleRate = obj.SampleRate;
            end
            [pathDelays,averagePathGains,K_1dB] = nrTDLChannel.getDelayProfile(obj);
            c.PathDelays = pathDelays;
            c.AveragePathGains = averagePathGains;
            c.NormalizePathGains = obj.NormalizePathGains;
            if (nrTDLChannel.hasLOSPath(obj))
                c.FadingDistribution = 'Rician';
                c.KFactor = 10^(K_1dB/10); % convert dB to linear
                c.DirectPathDopplerShift = 0.7*obj.MaximumDopplerShift;
                c.DirectPathInitialPhase = 0.0;
            else
                c.FadingDistribution = 'Rayleigh';
            end
            c.MaximumDopplerShift = obj.MaximumDopplerShift;
            c.DopplerSpectrum = doppler('Jakes');
            c.SpatialCorrelationSpecification = 'Combined';
            c.NumTransmitAntennas = nrTDLChannel.getAntennaCount(obj);
            % NumReceiveAntennas is not used as SpatialCorrelationSpecification = 'Combined'
            % TransmitCorrelationMatrix is not used as SpatialCorrelationSpecification = 'Combined'
            % ReceiveCorrelationMatrix is not used as SpatialCorrelationSpecification = 'Combined'
            c.SpatialCorrelationMatrix = nrTDLChannel.getSpatialCorrelationMatrix(obj);
            c.AntennaSelection = 'Off';
            c.NormalizeChannelOutputs = obj.NormalizeChannelOutputs;
            c.FadingTechnique = 'Sum of sinusoids';
            c.NumSinusoids = obj.NumSinusoids;
            if strcmpi(obj.PathGainSampleRate,'auto')
                c.InitialTimeSource = 'Input port';
                c.setCheckInitialTime(false);
            else
                c.InitialTimeSource = 'Property';
                c.InitialTime = obj.InitialTime;
            end
            c.RandomStream = obj.RandomStream;
            c.Seed = obj.Seed;
            c.PathGainsOutputPort = true;
            c.Visualization = 'Off';
            c.ChannelFiltering = false;
            if ~isempty(varargin)
                c.OutputDataType = varargin{1};
            end
            
        end
        
        % Create channel filter from nrTDLChannel properties
        function channelFilter = setupChannelFilter(obj,optimizeScalarGainFilter)
            
            pathDelays = nrTDLChannel.getDelayProfile(obj);
            channelFilter = comm.ChannelFilter('SampleRate',obj.SampleRate,'PathDelays',pathDelays,'NormalizeChannelOutputs',false,'OptimizeScalarGainFilter',optimizeScalarGainFilter);

        end
        
        function Rspat = getSpatialCorrelationMatrix(obj)
            
            coder.extrinsic('nr5g.internal.nrTDLChannel.calculateSpatialCorrelationMatrix')
            
            if (strcmp(obj.MIMOCorrelation,'Custom') && strcmp(obj.Polarization,'Custom'))
            
                % Use SpatialCorrelationMatrix property as the overall
                % spatial correlation matrix Rspat
                Rspat = obj.SpatialCorrelationMatrix;
                
            else
                
                % Note: where the text "(-by-Np)" appears after a matrix
                % size here, this means that the matrix may instead be a
                % 3-D array with 3rd dimension size Np, equal to the number
                % of paths (each path can have its own different matrix)
                
                % Get permutation matrix according to TS 36.101 Annex
                % B.2.3A.1 / TS 36.104 Annex B.5A.1, size
                % (Nt*Nr)-by-(Nt*Nr) for cross-polar antennas. For co-polar
                % antennas, P is 1 and therefore no permutation is
                % performed
                P = nrTDLChannel.getPermutationMatrix(obj);
                
                % Get transmit and receive side spatial correlation
                % matrices. For co-polar antennas, Rt and Rr are of size
                % Nt-by-Nt(-by-Np) and Nr-by-Nr(-by-Np) respectively. For
                % cross-polar antennas, they are of size
                % (Nt/2)-by-(Nt/2)(-by-Np) and (Nr/2)-by-(Nr/2)(-by-Np)
                % unless Nt=1 or Nr=1 in which case the size is
                % 1-by-1(-by-Np)
                Rt = nrTDLChannel.getTransmitCorrelationMatrix(obj);
                Rr = nrTDLChannel.getReceiveCorrelationMatrix(obj);
                
                % Get polarization covariance matrix. For co-polar antennas
                % Gamma is unity (size 1-by-1) and for cross-polar antennas
                % it is of size 4-by-4(-by-Np) unless Nt=1 or Nr=1 in which
                % case it is of size 2-by-2(-by-Np). (If both Nt and Nr are
                % 1, Gamma is of size 1-by-1(-by-Np))
                Gamma = nrTDLChannel.getPolarizationCorrelationMatrix(obj);
                
                % Compute overall spatial correlation matrix. Rspat is of
                % size (Nt*Nr)-by-(Nt*Nr)(-by-Np)
                a = nrTDLChannel.getRoundoffScalingFactor(obj);
                Rspat = coder.const(double(nr5g.internal.nrTDLChannel.calculateSpatialCorrelationMatrix(P,Rt,Rr,Gamma,a)));       
                
            end
        
        end
        
        % TS 36.101 Annex B.2.3A.1
        % TS 36.104 Annex B.5A.1
        function P = getPermutationMatrix(obj)
            
            [Nt,Nr] = nrTDLChannel.getAntennaCount(obj);
            
            if (strcmp(obj.Polarization,'Co-Polar'))
                % In co-polar cases, return 1 instead of identity matrix to
                % make spatial correlation calculation faster
                
                P = 1;
                
            else
                
                P = zeros(Nr*Nt);

                for i = 1:Nr
                    for j = 1:Nt

                        a = ((j-1)*Nr) + i;

                        if (j <= Nt/2)
                            b = (2*(j-1)*Nr) + i;
                        else
                            b = (2*(j-(Nt/2))*Nr) - Nr + i;
                        end

                        P(a,b) = 1;

                    end
                end
                
            end
            
        end
        
        function [Nt,Nr] = getAntennaCount(obj)
            
            if (strcmp(obj.MIMOCorrelation,'Custom'))
                if (strcmp(obj.Polarization,'Custom'))
                    Nt = obj.theNumTransmitAntennas;
                    Nr = size(obj.SpatialCorrelationMatrix,1) / Nt;
                else
                    Nt_in = size(obj.theTransmitCorrelationMatrix,1);
                    Nr_in = size(obj.theReceiveCorrelationMatrix,1);
                    if (strcmp(obj.Polarization,'Cross-Polar'))
                        [txPolAngles,rxPolAngles] = nrTDLChannel.getPolarizationAngles(obj);
                        Npt = numel(txPolAngles);
                        Npr = numel(rxPolAngles);
                        Nt = Nt_in * Npt;
                        Nr = Nr_in * Npr;
                    else
                        Nt = Nt_in;
                        Nr = Nr_in;
                    end
                end
            else
                Nt = obj.theNumTransmitAntennas;
                Nr = obj.theNumReceiveAntennas;
            end
            
        end
        
        % TS 36.101 Table B.2.3.2-1
        % TS 36.101 Table B.2.3A.3-1
        % TS 36.104 Table B.5.2-1
        % TS 36.104 Table B.5A.3-1
        function Rt = getTransmitCorrelationMatrix(obj)
            
            if (strcmpi(obj.MIMOCorrelation,'Custom'))
                Rt = obj.theTransmitCorrelationMatrix;
            else
                switch (nrTDLChannel.getMIMOCorrelationVersusLink(obj))
                    case 'Low'
                        r = 0;
                    case {'Medium','Medium-A','UplinkMedium'}
                        r = 0.3;
                    case 'High'
                        r = 0.9;
                end
                Rt = nrTDLChannel.getCorrelationMatrix(obj,obj.theNumTransmitAntennas,r);
            end

        end
        
        % TS 36.101 Table B.2.3.2-1
        % TS 36.101 Table B.2.3A.3-1
        % TS 36.104 Table B.5.2-1
        % TS 36.104 Table B.5A.3-1
        function Rr = getReceiveCorrelationMatrix(obj)

            if (strcmpi(obj.MIMOCorrelation,'Custom'))
                Rr = obj.theReceiveCorrelationMatrix;
            else
                switch (nrTDLChannel.getMIMOCorrelationVersusLink(obj))
                    case 'Low'
                        r = 0;
                    case {'Medium','UplinkMedium'}
                        r = 0.9;
                    case 'Medium-A'
                        if (strcmpi(obj.Polarization,'Co-Polar'))
                            r = 0.3874;
                        else
                            r = 0.6;
                        end
                    case 'High'
                        r = 0.9;
                end
                Rr = nrTDLChannel.getCorrelationMatrix(obj,obj.theNumReceiveAntennas,r);
            end

        end
        
        function mimoCorrelation = getMIMOCorrelationVersusLink(obj)
            
            if (strcmpi(obj.TransmissionDirection,'Uplink') && strcmpi(obj.MIMOCorrelation,'Medium'))
                mimoCorrelation = 'UplinkMedium';
            else
                mimoCorrelation = obj.MIMOCorrelation;
            end
            
        end
                
        function Gamma = getPolarizationCorrelationMatrix(obj)
            
            coder.extrinsic('nr5g.internal.nrTDLChannel.calculateGammaMatrix');
            
            switch (obj.Polarization)
                
                case 'Co-Polar'
                    
                    % For co-polar antennas, correlation is defined only in
                    % terms of spatial positions so polarization
                    % correlation matrix is unity (size 1-by-1)
                    Gamma = 1;
                    
                case 'Cross-Polar'
                    
                    % Transmit and receive polarization angles
                    [txPolAngles,rxPolAngles] = nrTDLChannel.getPolarizationAngles(obj);
                    
                    % Cross polarization power ratio in dB
                    if (strcmp(obj.MIMOCorrelation,'Custom'))
                        XPR = -obj.XPR;
                    else
                        gamma = nrTDLChannel.getPolarizationCorrelation(obj);
                        XPR = 10*log10((1-gamma)/(1+gamma));
                    end
                    
                    % Calculate polarization correlation matrix according
                    % to IEEE 802.16m-08/004r5 Appendix B
                    Gamma = coder.const(double(nr5g.internal.nrTDLChannel.calculateGammaMatrix(txPolAngles,rxPolAngles,XPR)));
                    
            end
            
        end
        
        function [txPolAnglesOut,rxPolAnglesOut] = getPolarizationAngles(obj)
            
            if (strcmp(obj.MIMOCorrelation,'Custom'))
                txPolAnglesOut = obj.theTransmitPolarizationAngles;
                rxPolAnglesOut = obj.theReceivePolarizationAngles;
            else
                if (strcmp(obj.Polarization,'Co-Polar'))
                    % TS 36.101 Annex B.2.3
                    % TS 36.104 Annex B.5
                    txPolAnglesOut = 90;
                    rxPolAnglesOut = 90;
                else
                    % TS 36.101 Annex B.2.3A
                    % TS 36.104 Annex B.5A
                    if (strcmp(obj.TransmissionDirection,'Downlink'))
                        txPolAngles = [45 -45];
                        rxPolAngles = [90 0];
                    else
                        txPolAngles = [90 0];
                        rxPolAngles = [45 -45];
                    end
                    if (obj.theNumTransmitAntennas==1)
                        txPolAnglesOut = txPolAngles(1);
                    else
                        txPolAnglesOut = txPolAngles;
                    end
                    if (obj.theNumReceiveAntennas==1)
                        rxPolAnglesOut = rxPolAngles(1);
                    else
                        rxPolAnglesOut = rxPolAngles;
                    end
                end
            end

        end
            
        % TS 36.101 Table B.2.3A.3-1
        % TS 36.104 Table B.5A.3-1
        function gamma = getPolarizationCorrelation(obj)
           
            switch (nrTDLChannel.getMIMOCorrelationVersusLink(obj))
                case 'Low'
                    gamma = 0;
                case {'Medium','Medium-A','UplinkMedium'}
                    gamma = 0.2;
                case 'High'
                    gamma = 0.3;
            end
            
        end

        % TS 36.101 Annex B.2.3.2
        % TS 36.101 Annex B.2.3A.3
        % TS 36.104 Annex B.5.2
        function a = getRoundoffScalingFactor(obj)
            
            [Nt,Nr] = nrTDLChannel.getAntennaCount(obj);
            
            a = 0.0;
            if (~strcmpi(obj.MIMOCorrelation,'Custom'))
                if (Nt==4 && Nr==4)
                    a = 0.00012;
                elseif (strcmpi(obj.MIMOCorrelation,'High') && ((((Nt==2 && Nr==4) || (Nt==4 && Nr==2)) && strcmpi(obj.Polarization,'Co-Polar')) || (Nt==8 && Nr==2)))
                    a = 0.00010;
                end
            end
            
        end
        
        % TS 36.101 Annex B.2.3.1
        % TS 36.101 Annex B.2.3A.2
        % TS 36.104 Annex B.5.1
        % TS 36.104 Annex B.5A.2
        function R = getCorrelationMatrix(obj,N,r)

            if (strcmp(obj.Polarization,'Cross-Polar') && N~=1)
                N = N/2;
            end
                
            switch N
                case 1
                    R = 1;
                case 2
                    R = [1 r; ...
                         r 1];
                case 4
                    R = [1          r^(1/9)    r^(4/9)    r;       ...
                         (r^(1/9))  1          r^(1/9)    r^(4/9); ...
                         (r^(4/9))  (r^(1/9))  1          r^(1/9); ...
                         r          (r^(4/9))  (r^(1/9))  1];
                otherwise
                    R = eye(N);
            end

        end

        function [pathDelaysOut,pathGainsOut,K_1dB] = getDelayProfile(obj)
            
            coder.extrinsic('nr5g.internal.nrTDLChannel.getTDLProfile');
            coder.extrinsic('wireless.internal.channelmodels.scaleDelaysAndKFactor');
            
            if strcmpi(obj.DelayProfile,'Custom')
                pathDelaysIn = obj.PathDelays;
                pathGainsIn = obj.AveragePathGains;
                if (nrTDLChannel.hasLOSPath(obj))
                    % Split the first path into a LOS part and Rayleigh
                    % part according to K_1
                    K_1dB = obj.KFactorFirstTap;
                    K_1 = 10^(K_1dB/10);
                    P_1dB = pathGainsIn(1);
                    P_1 = 10^(P_1dB/10);
                    pathDelays = [pathDelaysIn(1) pathDelaysIn(1) pathDelaysIn(2:end)];
                    pathGains = [(10*log10(P_1 * K_1 / (1 + K_1)) + [0 -K_1dB]) pathGainsIn(2:end)];
                    
                else
                    pathDelays = pathDelaysIn;
                    pathGains = pathGainsIn;
                end
            else
                desiredKFactor = NaN;
                if (nrTDLChannel.hasLOSPath(obj))
                    if (obj.KFactorScaling)
                        desiredKFactor = obj.KFactor;
                    end
                end
                pdp = coder.const(double(nr5g.internal.nrTDLChannel.getTDLProfile(obj.DelayProfile)));
                % Perform delay and K-factor scaling, unless the delay
                % profile is a tabulated simplified delay profile
                if (~any(strcmp(obj.DelayProfile,{'TDLA30','TDLB100','TDLC300','TDLC60','TDLD30','TDLA10','TDLD10','NTN-TDLA100','NTN-TDLC5'})))
                    pdp = coder.const(double(wireless.internal.channelmodels.scaleDelaysAndKFactor(pdp,desiredKFactor,obj.DelaySpread)));
                end
                pathDelays = pdp(:,1).'; % 1st column is delay
                pathGains = pdp(:,2).';  % 2nd column is power
            end
            
            % At this point in the code, if a Rician path is present it is
            % split into a LOS part and a Rayleigh part, whether the delay
            % profile was from the standard tables or is custom
            
            if nrTDLChannel.hasLOSPath(obj)
                % Remove 2nd entry of delay profile, corresponding to the
                % Rayleigh part of the Rician path. The first entry and
                % K_1dB now capture the Rician path
                K_1dB = pathGains(1) - pathGains(2);
                pathGainsOut = [10*log10(sum(10.^(pathGains(1:2)/10))) pathGains(3:end)];
                pathDelaysOut = pathDelays([1 3:end]);
            else
                K_1dB = -Inf;
                pathGainsOut = pathGains;
                pathDelaysOut = pathDelays;
            end
            
        end

        function has = hasLOSPath(obj)

            if strcmpi(obj.DelayProfile,'Custom')
                has = strcmpi(obj.FadingDistribution,'Rician');
            else
                has = wireless.internal.channelmodels.hasLOSPath(obj.DelayProfile);
            end

        end
        
        function validation(obj)
            
            coder.internal.errorIf( ...
                strcmp(obj.Polarization,'Custom') && ~strcmp(obj.MIMOCorrelation,'Custom'), ...
                'nr5g:nrTDLChannel:InvalidCorrelationForPolarization');
            
            [txPolAngles,rxPolAngles] = nrTDLChannel.getPolarizationAngles(obj);
            Npt = numel(txPolAngles);
            Npr = numel(rxPolAngles);
            
            coder.internal.errorIf( ...
                ~strcmp(obj.MIMOCorrelation,'Custom') && ...
                strcmp(obj.Polarization,'Cross-Polar') && ...
                (mod(obj.theNumTransmitAntennas,Npt)~=0 || mod(obj.theNumReceiveAntennas,Npr)~=0), ...
                'nr5g:nrTDLChannel:InvalidAntsForCrossPolar');
            
            coder.internal.errorIf( ...
                any(strcmp(obj.MIMOCorrelation, {'Medium', 'Medium-A', 'High'})) && ...
                (all(obj.theNumTransmitAntennas ~= [1 2 4]*Npt) || ...
                 all(obj.theNumReceiveAntennas  ~= [1 2 4]*Npr)), ...
                'nr5g:nrTDLChannel:InvalidAntsForCorrelation');
            
            Np = size(nrTDLChannel.getDelayProfile(obj),2);
            nrTDLChannel.validateCorrelationMatrixSize(nrTDLChannel.getTransmitCorrelationMatrix(obj),Np,'TransmitCorrelationMatrix');
            nrTDLChannel.validateCorrelationMatrixSize(nrTDLChannel.getReceiveCorrelationMatrix(obj),Np,'ReceiveCorrelationMatrix');
            if (strcmp(obj.MIMOCorrelation,'Custom') && strcmp(obj.Polarization,'Cross-Polar'))
                coder.internal.errorIf(all(size(obj.XPR,2)~=[1 Np]),'nr5g:nrTDLChannel:XPRDimNotMatchNP');
            end
            
        end
        
        function validateCorrelationMatrixSize(R,Np,name)
            
            coder.internal.errorIf(ndims(R)==3 && size(R,3)~=Np,'nr5g:nrTDLChannel:CorrMtxDimNotMatchNP',name);
            
        end
        
        function validateSatelliteDoppler(obj)
            if nrTDLChannel.isNTN(obj)
                % Check the maximum Doppler and sample rate of the channel,
                % in the presence of satellite Doppler shift
                coder.internal.errorIf( ...
                    (abs(obj.SatelliteDopplerShift)+obj.MaximumDopplerShift) > obj.SampleRate/10, ...
                    'nr5g:nrTDLChannel:MaxDopplerShiftTooLarge')
            end
        end

        function flag = isNTN(obj)
            flag = any(strcmpi(obj.DelayProfile, {'NTN-TDL-A' 'NTN-TDL-B' ...
                    'NTN-TDL-C' 'NTN-TDL-D' 'NTN-TDLA100' 'NTN-TDLC5'}));
        end

        function D = makeReciprocalSpatialCorrelationMatrix(C,Nt)
        
            Nr = size(C,1) / Nt;
            D = zeros(Nr*Nt,Nr*Nt);
            for m = 1:Nr
                for n = 1:Nr
                    X = C(m:Nr:Nt*Nr,n:Nr:Nt*Nr);
                    D((m-1)*Nt+(1:Nt),(n-1)*Nt+(1:Nt)) = X(1:Nt,1:Nt);
                end
            end
            
        end
        
        function val = swapTxRx(txval,rxval,txrxSwapped)
            
            if (~txrxSwapped)
                val = txval;
            else
                val = rxval;
            end
            
        end

    end
    
end
