classdef (Abstract) networkTraffic < handle & comm.internal.ConfigBase
    %networkTraffic Base class for network traffic models
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   networkTraffic methods:
    %
    %   generate - Generate application traffic packet
    %
    %   Note that generate is an abstract method, and you must implement it
    %   in the custom traffic model (derived class). For more information
    %   about this, see the provided example.
    %
    %   % Example: Implement a custom traffic model, and create an object
    %   % for the same traffic model.
    %
    %   % Define a custom traffic model (derived class).
    %   classdef customTraffic < wirelessnetwork.internal.networkTraffic
    %       properties % Configurable
    %           Prop1 = true;
    %       end
    %
    %       % You must implement these methods in the custom traffic model.
    %       methods
    %           function obj = customTraffic(varargin) % Constructor
    %               % To enable support for configurable properties through
    %               % name-value arguments in the derived class, call the
    %               % constructor of base class (within the constructor of
    %               % your derived class) by using the name-value arguments
    %               % as variable inputs.
    %               obj@wirelessnetwork.internal.networkTraffic(varargin{:});
    %
    %               % Additional code in constructor
    %           end
    %
    %           % Generate a packet every 100 milliseconds.
    %           function [dt, packetSize, varargout] = generate(obj, varargin)
    %               dt = 100; % Time left for the next packet generation (in milliseconds)
    %               packetSize = 1500; % In bytes
    %
    %               % Generate a random application packet for the custom traffic model.
    %               if nargout == 3
    %                   varargout{1} = randi([0 255],packetSize,1);
    %               end
    %           end
    %       end
    %   end
    %
    %   % Create an object of customTraffic class. This creates
    %   % "myTrafficObj" object with "Prop1" set to "false".
    %   myTrafficObj = customTraffic(Prop1=false);

    %   Copyright 2023 The MathWorks, Inc.

    %#codegen

    properties (Hidden)
        %ObjectAdded Flag indicating whether the traffic object is added to
        %an existing node
        %
        %   Once a traffic object is added to a node, this property is
        %   internally set to true. Once set to true, this flag cannot be
        %   reset to false. Note that this flag is used only for internal
        %   purposes.
        ObjectAdded (1,1) logical = false;
    end

    methods
        function obj = networkTraffic(varargin)
            % To enable support for configurable properties through
            % name-value arguments in the derived class, call the
            % constructor of base class (within the constructor of your
            % derived class) by using the name-value arguments as variable
            % inputs. For more understanding, refer the example given in <a
            % href="matlab:help('wirelessnetwork.internal.networkTraffic')">networkTraffic</a> help.
            obj@comm.internal.ConfigBase(varargin{:});
        end

        function set.ObjectAdded(obj, value)

            % Once you have added the traffic object to a node, refrain
            % from adding the same traffic source object to any other nodes.
            coder.internal.errorIf((obj.ObjectAdded && ~value), "wirelessnetwork:networkTraffic:UsedTrafficSource");
            obj.ObjectAdded = value;
        end
    end

    methods (Abstract)
        %generate Generate application traffic packet
        %
        %   [DT, PACKETSIZE] = generate(OBJ) returns DT and PACKETSIZE,
        %   where DT is the time left for the next packet generation (in
        %   milliseconds) and PACKETSIZE is the size of the current packet
        %   (in bytes).
        %
        %   [DT, PACKETSIZE] = generate(OBJ, ELAPSEDTIME) additionally
        %   specifies the time elapsed (in milliseconds) since the last
        %   call to the generate method, ELAPSEDTIME. When using this
        %   syntax, the generate function calculates DT based on ELAPSEDTIME.
        %
        %   [..., PACKET] = generate(...) returns the application packet,
        %   PACKET, for any of the input argument combinations in the
        %   previous syntaxes. PACKET contains a column vector of integer
        %   values in the range [0, 255]. Each value in PACKET represents a
        %   decimal octet. The function returns PACKET only when you
        %   specify the third output argument.
        [dt, packetSize, varargout] = generate(obj, varargin)
    end
end