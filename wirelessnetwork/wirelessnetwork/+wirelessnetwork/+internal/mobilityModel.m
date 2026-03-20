classdef (Abstract) mobilityModel < matlab.mixin.Copyable
    %mobilityModel Base class for mobility models
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   mobilityModel methods:
    %
    %   position - Returns the current position in 3-D Cartesian coordinates,
    %              representing the [x y z] position in meters.
    %   velocity - Returns the current velocity in 3-D Cartesian coordinates,
    %              representing the [x y z] velocity in meters per second.

    %   Copyright 2023-2024 The MathWorks, Inc.

    properties (SetAccess = protected)
        %LatestPosition Position of the node when the mobility model was invoked last
        %time
        %   Position of the node in 3-D Cartesian coordinates, representing the [x y z]
        %   position in meters.
        LatestPosition

        %LatestVelocity Velocity of the node when the mobility model was invoked last
        %time
        %   Velocity of the node in 3-D Cartesian coordinates, representing the [x y z]
        %   velocity in meters per second.
        LatestVelocity
    end

    methods (Abstract)
        %position Return the position of the node at current time
        pos = position(obj, currentTime)

        %Velocity Return the velocity of the node at current time
        vel = velocity(obj, currentTime)
    end
end