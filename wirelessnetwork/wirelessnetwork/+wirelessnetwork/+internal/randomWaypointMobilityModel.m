classdef randomWaypointMobilityModel < wirelessnetwork.internal.mobilityModel
    %randomWaypointMobilityModel Implement random waypoint mobility model
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   randomWaypointMobilityModel properties (configurable through constructor only):
    %
    %   Position      - Position of the node.
    %   SpeedRange    - Speed range [minSpeed, maxSpeed] used for setting the
    %                   speed of a node according to continuous uniform
    %                   distribution.
    %   PauseDuration - Time duration (in seconds) that a node pauses after
    %                   reaching a target position.
    %   BoundaryShape - Mobility boundary shape.
    %   Bounds        - Center location and size of the mobility boundary.
    %
    %   randomWaypointMobilityModel methods:
    %
    %   position - Returns the current position in 3-D Cartesian coordinates,
    %              representing the [x y z] position in meters.
    %   velocity - Returns the current velocity in 3-D Cartesian coordinates,
    %              representing the [x y z] velocity in meters per second.

    %   Copyright 2023-2024 The MathWorks, Inc.

    properties (SetAccess=private)
        %SpeedRange Speed limits from which the speed is randomly selected for
        %traveling from the current position to the target position
        %   Specify the speed range as a numeric vector of the form [minSpeed
        %   maxSpeed], where each element is greater than 0 and maxSpeed is greater
        %   than minSpeed. The minSpeed and maxSpeed indicate the minimum speed and
        %   maximum speed of the mobility model, respectively.
        SpeedRange

        %PauseDuration Time duration that a node pauses after reaching a target position
        %   Specify the pause duration (in seconds) as a nonnegative numeric scalar.
        PauseDuration

        %BoundaryShape Shape of the mobility boundary
        %   Specify the mobility boundary shape as "rectangle" or "circle".
        BoundaryShape

        %Bounds Center location and size of the mobility boundary shape
        %   Specify the center location and size of the mobility boundary shape
        %   according to the specified boundary shape. If the boundary shape is
        %   "rectangle", it is a vector of length 4 and its format is [center's
        %   X-coordinate, center's Y-coordinate, length, breadth]. If the boundary
        %   shape is "circle", it is a vector of length 3 and its format is
        %   [center's X-coordinate, center's Y-coordinate, radius].
        Bounds

        %Waypoint Randomly selected target position
        Waypoint

        %WaypointSpeed Randomly selected speed for moving towards the target
        %position
        WaypointSpeed
    end

    properties (Access = private)
        %PauseState Flag that indicates whether the mobility model is in pause
        %state. The values true and false indicate pause state and not in pause
        %state, respectively
        PauseState

        %PauseEndTime Time (in seconds) at which pause duration ends
        PauseEndTime

        %NextWaypointReachTime Time (in seconds) at which target position is reached
        NextWaypointReachTime

        %LastUpdateTime Most recent update time of mobility context
        LastUpdateTime = 0

        %DistanceTraveled Distance traveled (in meters) towards the target position
        %from the previous target position
        DistanceTraveled = 0

        %TotalDistance Total distance (in meters) from the current position to the
        %target position
        TotalDistance = 0

        %LowerLeftCorner X and Y coordinates of the lower left corner of the
        %rectangular boundary
        LowerLeftCorner

        %RectangularBoundary Flag that indicates whether the boundary shape is
        %rectangular
        RectangularBoundary
    end

    methods
        % Constructor
        function obj = randomWaypointMobilityModel(param)
            %randomWaypointMobilityModel Create a random waypoint mobility object
            %
            %   OBJ = randomWaypointMobilityModel(PARAM) creates a random waypoint
            %   mobility object.
            %
            %   OBJ is an object of type randomWaypointMobilityModel.
            %
            %   PARAM is a structure with the fields.
            %       Position      - Position of the node.
            %       SpeedRange   - Speed limits of the mobility model.
            %       PauseDuration - Time duration (in seconds) that a node pauses after
            %                       reaching a target position.
            %       BoundaryShape - Mobility boundary shape.
            %       Bounds        - Center location and size of the mobility boundary.

            inputParam = {"SpeedRange", "PauseDuration", "BoundaryShape", "Bounds"};
            for idx=1:numel(inputParam)
                obj.(char(inputParam{idx})) = param.(char(inputParam{idx}));
            end
            obj.LatestPosition = param.Position;
            obj.LatestVelocity = param.Velocity;
            obj.LastUpdateTime = param.CurrentTime;
            % Set the lower left corner value if the boundary shape is rectangle
            if strcmpi(obj.BoundaryShape, "rectangle")
                obj.RectangularBoundary = true;
                obj.LowerLeftCorner = [obj.Bounds(1)-obj.Bounds(3)/2 obj.Bounds(2)-obj.Bounds(4)/2];
            else
                obj.RectangularBoundary = false;
            end
            % Set the initial position as the current waypoint
            obj.Waypoint = obj.LatestPosition;
            if obj.PauseDuration > 0
                obj.PauseState = true;
                obj.PauseEndTime = obj.PauseDuration;
            else
                obj.PauseState = false;
                [obj.Waypoint, obj.LatestVelocity] = randomWaypoint(obj);
                obj.NextWaypointReachTime = obj.TotalDistance/obj.WaypointSpeed;
            end
        end

        function pos = position(obj, currentTime)
            %position Get position of node at current time
            %
            %   POS = position(OBJ, CURRENTTIME) returns the position of the
            %   node at current time.
            %
            %   POS is the 3-D Cartesian position vector.
            %
            %   CURRENTTIME is the current simulation time, in seconds.

            if currentTime == obj.LastUpdateTime
                pos = obj.LatestPosition;
            else
                % Update node position if current time is different from the previous time
                % the position was calculated
                updateMobilityContext(obj, currentTime);
                pos = obj.LatestPosition;
            end
        end

        function vel = velocity(obj, currentTime)
            %velocity Get velocity of the node at current time
            %
            %   vel = velocity(OBJ, CURRENTTIME) returns the velocity of the node at
            %   current time.
            %
            %   VEL is the 3-D Cartesian velocity vector.
            %
            %   CURRENTTIME is the current simulation time, in seconds.

            if currentTime == obj.LastUpdateTime
                vel = obj.LatestVelocity;
            else
                % Update node velocity if current time is different from the previous time
                % the velocity was calculated
                updateMobilityContext(obj, currentTime);
                vel = obj.LatestVelocity;
            end
        end
    end

    methods (Access = private)
        function updateMobilityContext(obj, currentTime)
            %updateMobilityContext Update position and velocity based on its current time

            % Find the current position and velocity by iterating from the last update
            % time to the current time
            while obj.LastUpdateTime < currentTime
                % Check if the mobility model reaches pause state during this entire time
                if obj.PauseState
                    if obj.PauseEndTime < currentTime
                        % If the pause time finishes before the current time, select the next random
                        % waypoint
                        obj.PauseState = false;
                        obj.LastUpdateTime = obj.PauseEndTime;
                        [obj.Waypoint, obj.LatestVelocity] = randomWaypoint(obj);
                        obj.NextWaypointReachTime = obj.LastUpdateTime + (obj.TotalDistance/obj.WaypointSpeed);
                    else
                        % If the pause time does not finish before the current time, just update
                        % the previous time to current time
                        obj.LastUpdateTime = currentTime;
                    end
                else % In mobile state
                    if obj.NextWaypointReachTime < currentTime
                        % If the mobile state finishes before the current time, move to the pause
                        % state
                        obj.PauseState = true;
                        obj.LastUpdateTime = obj.NextWaypointReachTime;
                        obj.PauseEndTime = obj.LastUpdateTime + obj.PauseDuration;
                        obj.LatestPosition = obj.Waypoint;
                    else
                        % If the mobile state does not finish before the current time, update the
                        % current position and velocity accordingly
                        elapsedTime = currentTime - obj.LastUpdateTime;
                        obj.DistanceTraveled = obj.DistanceTraveled + (obj.WaypointSpeed * elapsedTime);
                        obj.LatestPosition = obj.LatestPosition + (obj.LatestVelocity*elapsedTime);
                        obj.LastUpdateTime = currentTime;
                    end
                end
            end
        end

        function [waypoint, waypointVelocity] = randomWaypoint(obj)
            %randomWaypoint Get the next random waypoint within the specified boundary
            %and the selected velocity for reaching the waypoint

            pz = obj.LatestPosition(3);
            if obj.RectangularBoundary % Rectangle
                % Get a random point within the rectangular boundary
                px = obj.LowerLeftCorner(1) + obj.Bounds(3)*rand;
                py = obj.LowerLeftCorner(2) + obj.Bounds(4)*rand;
            else % Circle
                % Get a random point within the circular boundary
                r = obj.Bounds(3)*rand;
                theta = 2*pi*rand;
                px = r*cos(theta) + obj.Bounds(1);
                py = r*sin(theta) + obj.Bounds(2);
            end
            waypoint = [px py pz];
            obj.TotalDistance = norm(waypoint - obj.LatestPosition);
            obj.DistanceTraveled = 0;

            % Get a random value within the specified speed limits. This will act as
            % the magnitude of velocity
            obj.WaypointSpeed = obj.SpeedRange(1) + (obj.SpeedRange(2) - obj.SpeedRange(1)) * rand;
            % Find the velocity along each axis (X, Y, and Z) direction
            waypointVelocity = (obj.WaypointSpeed/obj.TotalDistance) * (waypoint - obj.LatestPosition);
        end
    end
end