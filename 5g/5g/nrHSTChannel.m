classdef (StrictDefaults) nrHSTChannel < matlab.System
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
    
%#codegen

% =========================================================================
%   public interface

    methods (Access = public)

        % nrHSTChannel constructor
        function obj = nrHSTChannel(varargin)

            % Set property values from any name-value pairs input to the
            % constructor
            setProperties(obj,nargin,varargin{:});
            
        end

        function h = getPathFilters(obj)
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

            % The path filters are available only after the channel has
            % been locked in the case of HST-SFN profile.
            mustBeLocked = strcmpi(obj.ChannelProfile,'HST-SFN');
            coder.internal.errorIf(mustBeLocked && ~isLocked(obj),'nr5g:nrHSTChannel:UnavailablePathFilters');

            % If static propagation conditions or single-tap profiles
            % (HST and HST-DPS), there is no filtering and path filters is
            % equal to 1.
            if ~hasChannelDelay(obj)

                h = 1;

            else % Multi-tap profiles (HST-SFN) in non-static propagation conditions 
                
                % Fractional delay filter coefficients corresponding to the
                % path delays associated to the input samples of the last
                % step call.
                sampleDelays = obj.pPathDelays*obj.SampleRate + channelFilterDelay(obj);

                % Get the path filters based on the integer delay offsets
                % and fractional delay filter coefficients.
                numCoeff = size(obj.pFracDelayCoeff,1);
                numTaps = obj.pNumTaps;
                h = zeros(obj.pChannelFilter.MaximumDelay,numTaps);
                for tap = 1:numTaps
                    d = sampleDelays(1,tap); % First sample delay
                    delayIdx = 1 + floor(d) + (1:numCoeff) - numCoeff/2;
                    h(delayIdx,tap) = relativeFracDelayCoeff(obj,d);
                end

                % If the delay difference between the max and min delays
                % associated to the input waveform to the last step call
                % exceeds a threshold, warn about old path filters. The
                % maximum number of samples that can be filtered without
                % getPathFilters warning is approximately threshold*(c/v),
                % with c the speed of light and v the train velocity.
                maxSampleDelay = max(sampleDelays,[],1);
                minSampleDelay = min(sampleDelays,[],1);
                threshold = 0.1;
                if any(abs(maxSampleDelay-minSampleDelay) > threshold)
                    coder.internal.warning('nr5g:nrHSTChannel:OldPathFilters',num2str(threshold));
                end

            end
        end

     end

    properties (Access = public, Nontunable)

        %ChannelProfile High-speed train (HST) channel profile
        %   Specify the HST channel profile as one of 'HST', 'HST-SFN',
        %   'HST-DPS'. These channel profiles are defined in TS 38.101-4
        %   Annex B.3.
        %
        %   The default value of this property is 'HST'.
        ChannelProfile = 'HST'

        %Ds Distance between gNodeBs (m)
        %   Specify the distance between gNodeBs in meters as a double
        %   precision, real, positive scalar. When ChannelProfile = 'HST',
        %   Ds/2 is the initial distance from the train to the gNodeB.
        %
        %   The default value of this property is 300 m.
        Ds = 300
        
        %Dmin Minimum distance between gNodeB and railway track (m)
        %   Specify the minimum distance between gNodeB and the railway
        %   track in meters as a double precision, real, positive scalar.
        %
        %   The default value of this property is 2 m.
        Dmin = 2

        %Velocity Velocity of the train in km/h
        %   Specify the velocity of the train in kilometers per hour as a
        %   double precision, real, nonnegative scalar. For static
        %   propagation conditions as defined in TS 38.101-4 Annex B.1, set
        %   the Velocity and MaximumDopplerShift properties to 0.
        %
        %   The default value of this property is 300 km/h.
        Velocity = 300

        %MaximumDopplerShift Maximum Doppler shift (Hz) 
        %   Specify the maximum Doppler shift in Hertz as a double
        %   precision, real, nonnegative scalar. The maximum Doppler shift
        %   applies to all the paths of the channel. For static propagation
        %   conditions as defined in TS 38.101-4 Annex B.1, set the
        %   Velocity and MaximumDopplerShift properties to 0.
        %
        %   The default value of this property is 750 Hz.
        MaximumDopplerShift = 750        

        %NumTaps Number of channel taps for HST-SFN channel profile
        %   Specify the number of channel taps. This property applies when
        %   ChannelProfile is set to 'HST-SFN'.
        %   
        %   The default value of this property is 4.
        NumTaps = 4;

        %SampleRate Sample rate (Hz)
        %   Specify the sample rate of the input signal in Hz as a double
        %   precision, real, positive scalar. The default value of this
        %   property is 30720000 Hz.
        SampleRate = 30720000

        %NumTransmitAntennas Number of transmit antennas
        %   Specify the number of transmit antennas as 1, 2, 4, or 8. This
        %   property applies when ChannelFiltering is set to false.
        %
        %   The default value of this property is 1.
        NumTransmitAntennas {mustBeMember(NumTransmitAntennas,[1,2,4,8])} = 1;

        %NumReceiveAntennas Number of receive antennas
        %   Specify the number of receive antennas as 1, 2, or 4.
        %
        %   The default value of this property is 2.
        NumReceiveAntennas {mustBeMember(NumReceiveAntennas,[1,2,4])} = 2;

    end

    properties (Access = public)

        %InitialTime Start time of the channel (s)
        %   Specify the time offset of the channel as a real nonnegative
        %   scalar. This property is tunable.
        %
        %   The default value of this property is 0.0.
        InitialTime = 0.0;
        
    end

    properties (Access = public, Nontunable)

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

    % public property setters for validation
    methods

        function set.Ds(obj,val)
            propName = 'Ds';
            validateattributes(val,{'double'},{'real','scalar','positive','finite'},[class(obj) '.' propName],propName);
            obj.Ds = val;
        end

        function set.Dmin(obj,val)
            propName = 'Dmin';
            validateattributes(val,{'double'},{'real','scalar','positive','finite'},[class(obj) '.' propName],propName);
            obj.Dmin = val;
        end

        function set.Velocity(obj,val)
            propName = 'Velocity';
            validateattributes(val,{'double'},{'real','scalar','nonnegative','finite'},[class(obj) '.' propName],propName);
            obj.Velocity = val;
        end

        function set.MaximumDopplerShift(obj,val)
            propName = 'MaximumDopplerShift';
            validateattributes(val,{'double'},{'real','scalar','nonnegative','finite'},[class(obj) '.' propName],propName);
            obj.MaximumDopplerShift = val;
        end
        
        function set.NumTaps(obj,val)
            propName = 'NumTaps';
            validateattributes(val,{'double'},{'integer','scalar','positive','finite'},[class(obj) '.' propName], propName);
            obj.NumTaps = val;
        end

        function set.SampleRate(obj,val)
            propName = 'SampleRate';
            validateattributes(val,{'double'},{'real','scalar','positive','finite'},[class(obj) '.' propName],propName);
            obj.SampleRate = val;
        end

        function set.InitialTime(obj,val)
            propName = 'InitialTime';
            validateattributes(val,{'double'},{'real','scalar','nonnegative','finite'},[class(obj) '.' propName],propName);
            obj.InitialTime = val;
        end

        function set.NumTimeSamples(obj,val)
            propName = 'NumTimeSamples';
            validateattributes(val,{'numeric'},{'scalar','integer','positive'},[class(obj) '.' propName],propName);
            obj.NumTimeSamples = val;
        end
        
    end

    % property value sets for enumerated properties
    properties(Hidden,Transient)
        
        ChannelProfileSet = matlab.system.StringSet({'HST','HST-SFN','HST-DPS'});
        OutputDataTypeSet = matlab.system.StringSet({'double','single'});
        
    end    

% =========================================================================
%   protected interface

    methods (Access = protected)

        % nrHSTChannel setupImpl method
        function setupImpl(obj,in)

            % Construct channel filter if needed
            if hasChannelDelay(obj)

                obj.pChannelFilter = constructChannelFilter(obj);
                obj.pFracDelayCoeff = nrHSTChannel.getAllFractionalDelayFilterCoefficients(clone(obj.pChannelFilter));
                
                % Set the configured number of taps.
                obj.pNumTaps = obj.NumTaps;
                
            end

            if (obj.ChannelFiltering)
                validateInputSignal(obj,in);
            else
                obj.pNumTxAnts = obj.NumTransmitAntennas;
            end

        end

        % nrHSTChannel stepImpl method
        function varargout = stepImpl(obj,varargin)

            % Determine the input size and output data type.
            if (obj.ChannelFiltering)
                in = varargin{1};
                validateInputSignal(obj,in);
                insize = [size(in,1) obj.pNumTxAnts];
                outputtype = class(in);
            else
                insize = [obj.NumTimeSamples obj.pNumTxAnts];
                outputtype = obj.OutputDataType;
            end

            Ns = insize(1);  % Number of input signal samples    
            Nt = insize(2);  % Number of Tx antennas

            % Calculate path gains, delays, and Doppler shifts associated
            % to the position of the train.
            if strcmpi(obj.ChannelProfile,'HST')
                [gains,powers,relativeDelays,dopplerShifts] = hstChannel(obj,Ns,Nt,outputtype);
            else
                [gains,powers,relativeDelays,dopplerShifts] = sfndpsChannel(obj,Ns,Nt,outputtype);
            end

            % Scale path gains by the number of receive antennas.
            if (obj.NormalizeChannelOutputs)
                gains = gains/sqrt(obj.NumReceiveAntennas);
            end

            % Filter input signal
            if (obj.ChannelFiltering)
                % Delay associated to each input sample, measured in samples.
                sampleDelays = relativeDelays*obj.SampleRate + channelFilterDelay(obj);
                out = nrHSTChannel.filterSignal(obj.pChannelFilter,complex(in),gains,sampleDelays(1:Ns,1:obj.pNumTaps));
            end

            % Update path delays, power levels, and Doppler shifts.
            obj.pPathDelays = relativeDelays;
            obj.pPowerLevels = powers;
            obj.pDopplerShifts = dopplerShifts;

            % Calculate sample times
            sampleTimes = obj.pTime + (0:Ns-1).'/obj.SampleRate;

            % Assign outputs
            if (obj.ChannelFiltering)
                varargout = {out gains sampleTimes};
            else
                varargout = {gains sampleTimes};
            end

            % Advance the time according to the input length, or
            % NumTimeSamples if channel filtering is disabled
            obj.pTime = obj.pTime + (Ns/obj.SampleRate);

        end

        % nrHSTChannel resetImpl method
        function resetImpl(obj)

            % reset the time to the last InitialTime property value set
            obj.pTime = obj.InitialTime;

            if (obj.ChannelFiltering) && ~isempty(obj.pChannelFilter)
                reset(obj.pChannelFilter);
            end

        end

        % nrHSTChannel releaseImpl method
        function releaseImpl(obj)

            % reset the time to the last InitialTime property value set
            obj.pTime = obj.InitialTime;

            % Reset the number of tx antennas.
            obj.pNumTxAnts = -1;

            if (obj.ChannelFiltering) && ~isempty(obj.pChannelFilter)
                release(obj.pChannelFilter);
            end

        end

        % nrHSTChannel getNumInputsImpl method
        function num = getNumInputsImpl(obj)
            
            num = double(obj.ChannelFiltering);
            
        end

        % nrHSTChannel getNumOutputsImpl method
        function num = getNumOutputsImpl(obj)
            
            num = 2 + obj.ChannelFiltering;
            
        end

        % nrHSTChannel isInputComplexityMutableImpl method
        function flag = isInputComplexityMutableImpl(obj,~) %#ok<INUSD> 
            flag = true;            
        end

        % nrHSTChannel isInputSizeMutableImpl method
        function flag = isInputSizeMutableImpl(obj,~)       %#ok<INUSD> 
            flag = true;
        end

        % nrHSTChannel infoImpl method
        function out = infoImpl(obj)
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

            validate(obj);

            out = struct();
            
            if hasChannelDelay(obj)
                out.PathDelays = obj.pPathDelays + obj.Dmin/obj.LightSpeed;
            else
                out.PathDelays = zeros(size(obj.pPathDelays));
            end

            out.DopplerShifts = obj.pDopplerShifts;
            out.PowerLevels = 10*log10(obj.pPowerLevels);
            out.CarrierFrequency = obj.MaximumDopplerShift*obj.LightSpeed/(obj.Velocity/3.6); % v/3.6 -> km/h to m/s.
            out.NumTransmitAntennas = repmat(obj.pNumTxAnts,double(obj.pNumTxAnts~=-1));      % Empty if object is not locked
            out.NumReceiveAntennas = obj.NumReceiveAntennas;
            out.ChannelFilterDelay = channelFilterDelay(obj); 
            out.MaximumChannelDelay = maxChannelDelay(obj);

        end

        % nrHSTChannel saveObjectImpl method
        function s = saveObjectImpl(obj)
            
            s = saveObjectImpl@matlab.System(obj);
            s.pTime = obj.pTime;
            s.pPathDelays = obj.pPathDelays;
            s.pDopplerShifts = obj.pDopplerShifts;
            s.pPowerLevels = obj.pPowerLevels;
            s.pFracDelayCoeff = obj.pFracDelayCoeff;
            s.pNumTaps = obj.pNumTaps;
            s.pNumTxAnts = obj.pNumTxAnts;

            if (obj.ChannelFiltering)
                s.pChannelFilter = matlab.System.saveObject(obj.pChannelFilter);
            else
                s.pChannelFilter = obj.pChannelFilter;
            end
            
        end

        % nrHSTChannel loadObjectImpl method
        function loadObjectImpl(obj,s,wasLocked)
            
            obj.pTime = s.pTime;
            obj.pPathDelays = s.pPathDelays;
            obj.pDopplerShifts = s.pDopplerShifts;
            obj.pPowerLevels = s.pPowerLevels;
            obj.pFracDelayCoeff = s.pFracDelayCoeff;
            obj.pNumTaps = s.pNumTaps;
            obj.pNumTxAnts = s.pNumTxAnts;

            if s.ChannelFiltering
                obj.pChannelFilter = matlab.System.loadObject(s.pChannelFilter);
            else
                obj.pChannelFilter = s.pChannelFilter;
            end

            loadObjectImpl@matlab.System(obj,s,wasLocked);

        end

        % nrHSTChannel isInactivePropertyImpl method
        function flag = isInactivePropertyImpl(obj,prop)
            
            if (any(strcmp(prop,{'NumTimeSamples','OutputDataType','NumTransmitAntennas'})))
                flag = obj.ChannelFiltering;
            elseif strcmpi(prop,'NumTaps')
                flag = ~strcmpi(obj.ChannelProfile,'HST-SFN');
            else
                flag = false;
            end
            
        end

        % nrHSTChannel validatePropertiesImpl method
        function validatePropertiesImpl(obj)
        
            validate(obj);
            
        end

        % nrHSTChannel processTunedPropertiesImpl method
        function processTunedPropertiesImpl(obj)

            if (isChangedProperty(obj,'InitialTime'))

                % If the tuned InitialTime changes the current time (to the
                % same tolerance as used to design fractional delay
                % filters)
                if (abs(obj.pTime - obj.InitialTime) > (obj.MaxFractionalDelayError/obj.SampleRate))
                    % set the time to the InitialTime property value
                    obj.pTime = obj.InitialTime;

                    % reset channel filters
                    if (obj.ChannelFiltering) && ~isempty(obj.pChannelFilter)
                        reset(obj.pChannelFilter);
                    end
                end

            end

        end

    end

% =========================================================================
%   private

    properties (Access = private, Constant)

        % Speed of light
        LightSpeed = physconst('Lightspeed');

        % Fractional delay FIR filter length
        ChannelFilterLength = 16;

        % Maximum fractional delay error used to specify the resolution of
        % the channel fractional delay filtering. The maximum fractional
        % delay error along with geometric and kinematic properties of the
        % channel determine the rate of change of fractional delay filters.
        % The level of intercarrier interference (ICI) introduced by HST
        % channels to OFDM waveforms is dominated by the Doppler spreads at
        % high speeds and high carrier frequencies. At lower speeds and
        % carrier frequencies, the change of fractional delay filters can
        % have a significant impact on the ICI when compared to Doppler
        % spread. The value of this property attempts to minimize such
        % impact in low mobility scenarios.
        MaxFractionalDelayError = 0.001;

    end

    properties (Access = private, Nontunable)

        % Fractianal delay filter coefficients as an Nc-by-Nf matrix. The
        % number of coefficients Nc depends on the maximum delay of the
        % channel. The number of fractional delay filters is Nf =
        % 1/(2*MaxFractionalDelayError).
        pFracDelayCoeff

        % Number of channel taps
        pNumTaps = 1;

    end

    properties (Access = private)

        % Current time, advanced according to the input length (or
        % NumTimeSamples if ChannelFiltering is set to false) on each step
        % call.
        pTime = 0;

        % Variable fractional delay filter
        pChannelFilter;

        % Channel path delays in samples
        pPathDelays;

        % Channel power levels
        pPowerLevels;
        
        % Channel Doppler shifts
        pDopplerShifts;

        % Number of transmit antennas based on the input signal dimensions
        % or NumTransmitAntennas property
        pNumTxAnts = -1;

    end

    methods (Access = private)

        % Calculate the channel path gains, power levels, path delays and
        % Doppler frequencies according to TS 38.101-4 Annex B.3.1
        function [pathGains,powerLevels,pathDelays,dopplerFreqs] = hstChannel(obj,Ns,Nt,outputtype)

            % Time vector of the input waveform.
            t = cast(obj.pTime + (0:Ns-1)'/obj.SampleRate,outputtype);

            % Extract some channel properties.
            Ds = obj.Ds;                 %#ok<*PROPLC>
            Dmin = obj.Dmin;
            v = obj.Velocity/3.6+realmin;% Velocity in m/s. realmin avoids division by zero.

            % Calculate channel matrix. For static propagation conditions,
            % H is defined in TS 38.101-4 Annex B.1. Otherwise, it is all
            % ones matrix.
            H = channelMatrix(obj,Nt);

            % Doppler frequency shift (B.3.1.1-4). Equation (B.3.1.3) is
            % implemented by time reversal (2*Ds/v-t) on (B.3.1.2).
            ts = Ds/v;                                  % Travel time between gNodeBs
            tm = mod(t,2*ts);                           % Time modulo 2*Ds/v (B.3.1.4)
            late = tm>ts;                               % Time of the second time period (B.3.1.3)
            tm(late) = 2*ts-tm(late);                   % Time reversal of the second time period
            distance = sqrt(Dmin^2 + (Ds/2 - tm*v).^2); % Distance between gNB and train
            cosTh = (Ds/2-v*tm)./distance;              % B.3.1.2 and B.3.1.3
            dopplerFreqs = obj.MaximumDopplerShift*cosTh;

            % Instantaneous phase of the channel tap. The following
            % expression is equivalent to -2*pi/lambda*d, where lambda is
            % the carrier wavelength. Use distance reversal to keep
            % consistency between the phases and Doppler frequencies above.
            distance(late) = 2*sqrt(Dmin^2 + Ds^2/4) - distance(late);
            phase = -2*pi*obj.MaximumDopplerShift*distance/v;

            % Path gains of size Ns-by-1-by-Nt-by-Nr.
            pathGains = H.*exp(1i*phase);

            % Set path delays to 0 and power levels to 1.
            pathDelays = zeros(Ns,1,outputtype);
            powerLevels = ones(Ns,1,outputtype);

        end

        % Calculate the channel path gains, power levels, path delays and
        % Doppler frequencies according to TS 38.101-4 Annex B.3.2 and
        % Annex B.3.3.
        function [pathGains,powerLevels,pathDelays,dopplerFreqs] = sfndpsChannel(obj,Ns,Nt,outputtype)

            % Time vector of the input waveform.
            time = cast(obj.pTime + (0:Ns-1)'/obj.SampleRate,outputtype);

            % Extract some channel properties.
            Ds = obj.Ds;                        % Inter RRH distance
            Dmin = obj.Dmin;                    % RRH-railway distance
            v = obj.Velocity/3.6+realmin;       % Velocity in m/s. realmin avoids division by zero
            Np = cast(obj.pNumTaps,outputtype); % Number of simultaneous taps (RRHs)
            Nr = obj.NumReceiveAntennas;        % Number of receive antennas

            % Calculate channel matrix. For static propagation conditions,
            % H is defined in TS 38.101-4 Annex B.1. Otherwise, it is all
            % ones matrix.
            H = channelMatrix(obj,Nt);

            % Indices of the transmission points (RRHs) switching sample
            % times. These indices allow for processing of the input
            % waveform in time blocks where visible RRHs do not change. For
            % HST-SFN, the switching occurs when the train is closest to an
            % RRH. For HST-DPS, the switching occurs in the middle point
            % between two RRHs (Ds/2) according to (B.3.3.4). These indices
            % also contain the first and last input sample times.
            offset = Ds/2*strcmpi(obj.ChannelProfile,'HST-DPS');    % Switching point adjustment for HST-DPS
            refRRH = floor((v*time+offset)/Ds);                     % Reference RRH
            rrhSwitchSamples = 1 + [0; find(diff(refRRH)>0); Ns-1]; % Indices of switching samples

            % Initialize channel output parameters for block processing.
            pathGains = complex(zeros(Ns,Np,Nt,Nr,outputtype));
            pathDelays = zeros(Ns,Np,outputtype);
            dopplerFreqs = zeros(Ns,Np,outputtype);
            powerLevels = zeros(Ns,Np,outputtype);

            % Calculate the channel outputs for each time block of the
            % input waveform where the visible RRHs do not change.
            for b = 1:length(rrhSwitchSamples)-1

                % Local sample indices and time to this block.
                s = rrhSwitchSamples(b):rrhSwitchSamples(b+1);
                t = time(s);

                % Indices of the visible RRHs in this time block. The
                % circular shift allows for continuity of the channel
                % ouputs for each tap across multiple time blocks.
                kOffset = refRRH(rrhSwitchSamples(b));
                k = kOffset + (floor(-Np/2)+1:floor(Np/2));
                k = circshift(k,kOffset,2);

                % Position of the visible RRHs in this time block
                % (B.3.2.1 and B.3.3.1)
                xk = k*Ds + 1i*Dmin;

                % Positon of the train at the input sample times
                % (B.3.2.2 and B.3.3.2).
                y = v*t;

                % Position vector of the train with respect to each RRH and
                % corresponding distances.
                pt = y-xk;
                d = abs(pt);

                % Linear path power normalized by the total power of all
                % paths (B.3.2.4). This is equal to 1 for HST-DPS.
                dsq = d.^2;
                powerLev = 1./(dsq.*sum(1./dsq,2));

                % Doppler shifts (B.3.2.5 and B.3.3.5) and path delays
                % (B.3.2.6)
                Fdk = obj.MaximumDopplerShift*real(-pt./d);     % Doppler shift of each tap
                delays = (d-Dmin)/obj.LightSpeed;               % Delays relative to the minimum delay.

                % Instantaneous phase of each tap. The following expression
                % is equivalent to -2*pi/lambda*d, where lambda is the
                % carrier wavelength.
                phase = -2*pi*obj.MaximumDopplerShift*d/v;

                % Path gains of size Ns-by-Np-by-Nt-by-Nr.
                g = H.*powerLev.*exp(1i*phase);

                % Store this block of channel output parameters.
                bind = 1+(0:(rrhSwitchSamples(b+1)-rrhSwitchSamples(b)));
                timeIndex = s(bind);
                pathGains(timeIndex,:) = g(bind,:);
                pathDelays(timeIndex,:) = delays(bind,:);
                dopplerFreqs(timeIndex,:) = Fdk(bind,:);
                powerLevels(timeIndex,:) = powerLev(bind,:);

            end

        end

        function H = channelMatrix(obj,Nt)

            % Channel matrices defined in TS 38.101-4 Annex B.1.
            Hmat = nr5g.internal.staticChannelMatrix(Nt,obj.NumReceiveAntennas);
            H = permute(Hmat,[4 3 2 1]);

        end

        % Establish whether a channel profile has a delay ~= 0. Only
        % non-static HST-SFN profiles introduce delay.
        function out = hasChannelDelay(obj)

            out = double(strcmpi(obj.ChannelProfile,'HST-SFN') && (obj.MaximumDopplerShift ~= 0));

        end

        % Calculate the maximum delay of the channel in samples including
        % the channel fractional delay filter lengths to avoid clipping of
        % delays.
        function maxDelaySamples = maxChannelDelay(obj)

            if hasChannelDelay(obj)

                % Calculate maximum and minimum distances between railway and gNodeBs.
                maxDistance = sqrt((2*obj.Ds)^2 + obj.Dmin^2);
                minDistance = obj.Dmin;
                delayRange = (maxDistance - minDistance) / obj.LightSpeed;
    
                % The maximum delay in samples is the sum of the propagation
                % delay and the fractional delay filter length.
                maxDelaySamples = 1 + obj.ChannelFilterLength + ceil(delayRange*obj.SampleRate);
                
            else
                % No delay
                maxDelaySamples = 0;
            end
            
        end

        % Construct channel filter based on dsp.VariableFractionalDelay
        function channelFilt = constructChannelFilter(obj)
            
            channelFilt = dsp.VariableFractionalDelay(...
                'InterpolationMethod','FIR',...
                'FilterHalfLength',obj.ChannelFilterLength/2,...
                'InterpolationPointsPerSample',1/(2*obj.MaxFractionalDelayError),...
                'MaximumDelay',maxChannelDelay(obj));

        end

        % Get the fractional delay filter coefficients associated to a
        % delay in samples.
        function coeff = relativeFracDelayCoeff(obj,sampleDelays)

            % Fractional delays in samples.
            fracDelay = mod(sampleDelays,1);
            
            % Number of interpolation points per sample.
            L = obj.pChannelFilter.InterpolationPointsPerSample;
            delayIntervals = [-1,(0:L-1)/L + 1/(2*L)];
            % Index of the fractional delay filter (ranges from 1 to L+1).
            filtIdx = quantiz(fracDelay,delayIntervals);

            % Fractional delay filter coefficients.
            coeff = obj.pFracDelayCoeff(:,filtIdx);

        end

        % Determine channel filter delay. It is zero for static propagation
        % conditions and for HST and HST-DPS channel profiles as they are
        % single tap profiles and filtering does not occur.
        function filterDelay = channelFilterDelay(obj)
            
            filterDelay = (obj.ChannelFilterLength/2-1) * hasChannelDelay(obj);

        end

        % Validate object
        function validate(obj)

            coder.internal.errorIf( xor(~obj.Velocity,~obj.MaximumDopplerShift) ,'nr5g:nrHSTChannel:IncompatibleVelocityDoppler')

        end

        % Validate input signal
        function validateInputSignal(obj,in)

            numInputColumns = size(in,2);
            if obj.pNumTxAnts == -1 % If number of antennas not initialized yet

                % Validate the number of antennas (columns of the input
                % signal) is supported.
                coder.internal.errorIf(~ismember(numInputColumns, [0 1 2 4 8]),'nr5g:nrHSTChannel:InvalidNumTx',numInputColumns);

                % Lock the number of antennas to the number of columns of the
                % input signal
                obj.pNumTxAnts = numInputColumns;
            else
                % Validate the number of antennas (columns of the input
                % signal) is consistent with previous calls.
                coder.internal.errorIf(numInputColumns ~= obj.pNumTxAnts,'nr5g:nrHSTChannel:SignalInputNotMatchNumTx',numInputColumns,obj.pNumTxAnts);
            end

            validateattributes(in,{'double','single'},{'2d','finite'},class(obj),'signal input');
            
        end

    end

    methods (Access = private, Static)

        % Filter input signal using channel filter, path gains and delays.
        function outputSignal = filterSignal(chanFilt,in,pathGains,sampleDelays)
            
            if ~isempty(chanFilt)
                filtOut = chanFilt(in,permute(sampleDelays,[1 3 2]));
            else
                filtOut = in;
            end

            % Multiply filtered signal by path gains.
            outputSignal = permute( sum( permute(filtOut,[1 3 2 4]).*pathGains ,[2 3]), [1 4 2 3]);

        end
        
        % Calculate the coefficients of all fractional delay filters in the
        % channel filter. The filter coefficients are obtained by filtering
        % an impulse sequence with all possible fractional delays, limited
        % to the resolution (number of interpolation points per sample) of
        % the channel filter.
        function filterCoeff = getAllFractionalDelayFilterCoefficients(varFracDelayFilter)
            
            % Create an impulse signal that covers the full response of the
            % fractional delay filters.
            filterHalfLength = varFracDelayFilter.FilterHalfLength;
            imp = [1;zeros(filterHalfLength*2-1,1)];

            % Filter impulse signal to get channel impulse response for
            % each possible fractional delay filter.
            L = varFracDelayFilter.InterpolationPointsPerSample;
            filterCoeff = zeros(length(imp),L+1);
            delayBinCenter = (0:L)/L;
            for i=1:L+1
                d = filterHalfLength - 1 + delayBinCenter(i);
                filterCoeff(:,i) = varFracDelayFilter(imp,d);
            end

        end

    end
    
end
