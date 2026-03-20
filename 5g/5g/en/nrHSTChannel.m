classdef nrHSTChannel< matlab.System
%nrHSTChannel TS 38.101-4 Annex B.3 High-speed train channel
%   CHAN = nrHSTChannel creates a high-speed train (HST) MIMO channel
%   System object, CHAN. This object filters an input signal through the
%   HST channel to obtain the channel-impaired signal. This object
%   implements the following channel profiles of TS 38.101-4:
%   * Annex B.1 Static Propagation Condition
%   * Annex B.3.1 HST Single-Tap Channel Profile
%   * Annex B.3.2 HST-SFN Channel Profile
%   * Annex B.3.3 HST-DPS Channel Profile
%   This object also implements the profile defined in TS 38.104 Annex G.3,
%   which is equivalent to that in TS 38.101-4 Annex B.3.1.
%
%   CHAN = nrHSTChannel(Name,Value) creates an HST channel object, CHAN,
%   with the specified property Name set to the specified Value. You can
%   specify additional name-value pair arguments in any order as
%   (Name1,Value1,...,NameN,ValueN).
%
%   Step method syntax for ChannelFiltering set to true:
%
%   Y = step(CHAN,X) filters the input signal X through an HST channel and
%   returns the result in Y. The input X can be a double or single
%   precision data type scalar, vector, or 2-D matrix. X is of size
%   Ns-by-Nt, where Ns is the number of samples and Nt is the number of
%   transmit antennas. Nt must remain fixed across step calls. Y is the
%   output signal of size Ns-by-Nr, where Nr is the number of receive
%   antennas. Y is of the same data type as the input signal X.
% 
%   [Y,PATHGAINS] = step(CHAN,X) returns the channel path gains in
%   PATHGAINS. PATHGAINS is of size Ns-by-Np-by-Nt-by-Nr, where Np is the
%   number of paths. PATHGAINS is of the same data type as the input signal
%   X.
%
%   [Y,PATHGAINS,SAMPLETIMES] = step(CHAN,X) also returns the sample times
%   of the channel snapshots (1st dimension elements) of PATHGAINS.
%   SAMPLETIMES is of size Ns-by-1 and is of double precision data type
%   with real values.
%
%   Step method syntax for ChannelFiltering set to false:
%
%   [PATHGAINS,SAMPLETIMES] = step(CHAN) produces path gains PATHGAINS and
%   sample times SAMPLETIMES as described above, where the duration of the
%   channel process is given by the NumTimeSamples property. In this case
%   the object acts as a source of path gains and sample times without
%   filtering an input signal. The property NumTransmitAntennas specifies
%   the number of transmit antennas Nt. The data type of PATHGAINS is
%   specified by the OutputDataType property.
% 
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj,x) and y = obj(x) are
%   equivalent.
%
%   nrHSTChannel methods:
%
%   step                   - Filter input signal through an HST MIMO fading channel
%                            (see above)
%   release                - Allow property value and input characteristics changes
%   clone                  - Create an HST channel object with same property values
%   isLocked               - Locked status (logical)
%   <a href="matlab:help nrHSTChannel/reset">reset</a>                  - Reset states of filters
%   <a href="matlab:help nrHSTChannel/infoImpl">info</a>                   - Return characteristic information about the HST 
%                            channel
%   getPathFilters         - Get filter impulse responses for the filters 
%                            which apply the path delays to the input waveform
%
%   nrHSTChannel properties:
%
%   ChannelProfile             - HST channel profile
%   Ds                         - Distance between gNodeBs (m)
%   Dmin                       - Minimum distance between gNodeB and railway track (m)
%   Velocity                   - Velocity of the train (km/h)
%   MaximumDopplerShift        - Maximum Doppler shift (Hz)
%   NumTaps                    - Number of channel taps
%   SampleRate                 - Input signal sample rate (Hz)
%   NumTransmitAntennas        - Number of transmit antennas
%   NumReceiveAntennas         - Number of receive antennas
%   InitialTime                - Start time of the channel (s)
%   NormalizeChannelOutputs    - Normalize channel outputs (logical)
%   ChannelFiltering           - Perform filtering of input signal (logical)
%   NumTimeSamples             - Number of time samples
%   OutputDataType             - Path gain output data type
% 
%   % Example 1:
%   % Configure an HST single-tap channel and filter an input signal. Use 
%   % a distance between gNodeBs of 300 m, a minimum distance between
%   % gNodeBs and railway track of 2 m, a train velocity of 300 km/h and a
%   % maximum Doppler shift of 750 Hz.
%
%   hst = nrHSTChannel;
%   hst.ChannelProfile = 'HST';     
%   hst.Ds = 300;                   
%   hst.Dmin = 2;                   
%   hst.Velocity = 300;             
%   hst.MaximumDopplerShift = 750; 
%   
%   % Create a random waveform of 1 subframe duration with 1 antenna and
%   % pass it through the channel.
%   Nt = 1;                     % Number of transmit antennas
%   hst.SampleRate = 30.72e6;   % Sample rate
%   T = hst.SampleRate * 1e-3;  % Time span of the waveform
%
%   txWaveform = complex(randn(T,Nt),randn(T,Nt));
%
%   % Pass waveform through channel
%   rxWaveform = hst(txWaveform);
%
%   % Example 2: 
%   % Configure an HST-SFN channel profile and estimate the delay
%   % introduced by the channel.
% 
%   hst = nrHSTChannel;
%   hst.ChannelProfile = 'HST-SFN';
%   hst.Ds = 700;
%   hst.Dmin = 150;
%   hst.Velocity = 500;
%   hst.MaximumDopplerShift = 870;
%   hst.SampleRate = 30.72e6;
%    
%   % Set the initial time of the channel. This time configures the
%   % starting position of the train. The train position relative to the 
%   % remote radio heads (RRH) determines the delay of each RRH signal.
%   hst.InitialTime = (hst.Ds/3)/(hst.Velocity/3.6);
%
%   % Disable channel filtering and configure the number of channel samples
%   % to calculate 1 ms worth of path gains samples.
%   hst.ChannelFiltering = false;
%   hst.NumTimeSamples = hst.SampleRate*1e-3;
% 
%   % Retrieve the path gains from the channel.
%   pathGains = hst();
%   
%   % The delays of each RRH signal change over time. Calculate the channel
%   % path filter responses relative to the previous channel call. 
%   pathFilters = getPathFilters(hst); 
%   
%   % Estimate the channel delay
%   offset = nrPerfectTimingEstimate(pathGains,pathFilters)
% 
%   % Display the path filters and estimated channel delay
%   plot(0:size(pathFilters,1)-1,pathFilters);
%   hold on
%   stem(repmat(offset,1,hst.NumTaps),pathFilters(1+offset,:),'k')
%   legend(["Tap "+ (1:hst.NumTaps) "Timing offset"])
%   xlabel('Filter delay (samples)') 
%   ylabel('Amplitude')
%   title('Impulse Response of the Channel Filters')
%
%   See also nrTDLChannel, nrCDLChannel, nrPerfectChannelEstimate,
%   nrPerfectTimingEstimate.

 
%   Copyright 2022-2024 The MathWorks, Inc.

    methods
        function out=nrHSTChannel
            % Set property values from any name-value pairs input to the
            % constructor
        end

        function out=getNumInputsImpl(~) %#ok<STOUT>
        end

        function out=getNumOutputsImpl(~) %#ok<STOUT>
        end

        function out=getPathFilters(~) %#ok<STOUT>
            %getPathFilters Get time-varying path filter impulse responses
            %   H = getPathFilters(CHAN) returns a double precision real matrix
            %   H of size Nh-by-Np where Nh is the number of impulse response
            %   samples and Np is the number of paths. Each column of H
            %   contains the channel filter impulse response for each path of
            %   the channel. This information facilitates reconstruction of a
            %   perfect channel estimate when used in conjunction with the
            %   PATHGAINS output of the step method. These filters are
            %   time-varying and depend on the state of the channel for the
            %   HST-SFN channel profile. The path filters are relevant to the
            %   latest step call.
        end

        function out=infoImpl(~) %#ok<STOUT>
            %info Returns characteristic information about the HST channel
            %   S = info(CHAN) returns a structure containing characteristic
            %   information, S, about the HST channel. A description of
            %   the fields and their values is as follows:
            % 
            %   PathDelays          - Ns-by-Np matrix providing the absolute
            %                         propagation delays of the discrete
            %                         channel paths at the input signal sample
            %                         times, in seconds. Ns is the number of
            %                         input samples and Np the number of paths.
            %   DopplerShifts       - Ns-by-Np matrix providing Doppler shifts
            %                         of the discrete paths at the input
            %                         signal sample times, in Hz.
            %   PowerLevels         - Ns-by-Np matrix providing path gains
            %                         of the discrete paths at the input signal
            %                         sample times, in dB.
            %   CarrierFrequency    - Carrier frequency in Hz. The carrier
            %                         frequency (f) depends on the maximum
            %                         Doppler shift (fd), train velocity (v)
            %                         and the speed of light (c) as f = fd*c/v.
            %                         For static propagation conditions (fd=0,
            %                         v=0), the carrier frequency is returned
            %                         as NaN.
            %   NumTransmitAntennas - Number of transmit antennas. When the
            %                         property ChannelFiltering = true, the
            %                         second dimension of input signal
            %                         determines the value of this field.
            %                         Otherwise, this value is equal to the
            %                         NumTransmitAntennas property value.
            %   NumReceiveAntennas  - Number of receive antennas.
            %   ChannelFilterDelay  - Channel filter delay in samples.
            %   MaximumChannelDelay - Maximum channel delay in samples. This 
            %                         delay consists of the ChannelFilterDelay
            %                         and the maximum propagation delay
            %                         relative to the minimum propagation delay
            %                         Dmin/c. This field is calculated as
            %                         MaximumChannelDelay = ChannelFilterDelay
            %                         + (MaxPropDelay-MinPropDelay).
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
        end

        function out=releaseImpl(~) %#ok<STOUT>
            % reset the time to the last InitialTime property value set
        end

        function out=resetImpl(~) %#ok<STOUT>
            % reset the time to the last InitialTime property value set
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
        end

        function out=setupImpl(~) %#ok<STOUT>
            % Construct channel filter if needed
        end

        function out=stepImpl(~) %#ok<STOUT>
            % Determine the input size and output data type.
        end

        function out=validatePropertiesImpl(~) %#ok<STOUT>
        end

    end
    properties
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

        %ChannelProfile High-speed train (HST) channel profile
        %   Specify the HST channel profile as one of 'HST', 'HST-SFN',
        %   'HST-DPS'. These channel profiles are defined in TS 38.101-4
        %   Annex B.3.
        %
        %   The default value of this property is 'HST'.
        ChannelProfile;

        %Dmin Minimum distance between gNodeB and railway track (m)
        %   Specify the minimum distance between gNodeB and the railway
        %   track in meters as a double precision, real, positive scalar.
        %
        %   The default value of this property is 2 m.
        Dmin;

        %Ds Distance between gNodeBs (m)
        %   Specify the distance between gNodeBs in meters as a double
        %   precision, real, positive scalar. When ChannelProfile = 'HST',
        %   Ds/2 is the initial distance from the train to the gNodeB.
        %
        %   The default value of this property is 300 m.
        Ds;

        %InitialTime Start time of the channel (s)
        %   Specify the time offset of the channel as a real nonnegative
        %   scalar. This property is tunable.
        %
        %   The default value of this property is 0.0.
        InitialTime;

        %MaximumDopplerShift Maximum Doppler shift (Hz) 
        %   Specify the maximum Doppler shift in Hertz as a double
        %   precision, real, nonnegative scalar. The maximum Doppler shift
        %   applies to all the paths of the channel. For static propagation
        %   conditions as defined in TS 38.101-4 Annex B.1, set the
        %   Velocity and MaximumDopplerShift properties to 0.
        %
        %   The default value of this property is 750 Hz.
        MaximumDopplerShift;

        %NormalizeChannelOutputs Normalize channel outputs by the number of receive antennas (logical)
        %   Set this property to true to normalize the channel outputs by
        %   the number of receive antennas. When you set this property to
        %   false, there is no normalization for channel outputs.
        %
        %   The default value of this property is true.
        NormalizeChannelOutputs;

        %NumReceiveAntennas Number of receive antennas
        %   Specify the number of receive antennas as 1, 2, or 4.
        %
        %   The default value of this property is 2.
        NumReceiveAntennas;

        %NumTaps Number of channel taps for HST-SFN channel profile
        %   Specify the number of channel taps. This property applies when
        %   ChannelProfile is set to 'HST-SFN'.
        %   
        %   The default value of this property is 4.
        NumTaps;

        %NumTimeSamples Number of time samples
        %   Specify the number of time samples used to set the duration of
        %   the fading process realization as a positive integer scalar.
        %   This property applies when ChannelFiltering is false. This
        %   property is tunable.
        %
        %   The default value of this property is 30720.
        NumTimeSamples;

        %NumTransmitAntennas Number of transmit antennas
        %   Specify the number of transmit antennas as 1, 2, 4, or 8. This
        %   property applies when ChannelFiltering is set to false.
        %
        %   The default value of this property is 1.
        NumTransmitAntennas;

        %OutputDataType Path gain output data type
        %   Specify the path gain output data type as one of 'double' or
        %   'single'. This property applies when ChannelFiltering is false.
        % 
        %   The default value of this property is 'double'.
        OutputDataType;

        %SampleRate Sample rate (Hz)
        %   Specify the sample rate of the input signal in Hz as a double
        %   precision, real, positive scalar. The default value of this
        %   property is 30720000 Hz.
        SampleRate;

        %Velocity Velocity of the train in km/h
        %   Specify the velocity of the train in kilometers per hour as a
        %   double precision, real, nonnegative scalar. For static
        %   propagation conditions as defined in TS 38.101-4 Annex B.1, set
        %   the Velocity and MaximumDopplerShift properties to 0.
        %
        %   The default value of this property is 300 km/h.
        Velocity;

    end
end
