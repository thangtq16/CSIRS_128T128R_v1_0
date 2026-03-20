classdef queue < handle
    %queue Implements packet queueing functionality
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   QUEUEOBJ = queue(MAXPACKETS) creates a first-in first-out (FIFO) queue
    %   to buffer MAXPACKETS packets.
    %
    %   MAXPACKETS indicates the maximum number of packets can be stored in the
    %   queue.
    %
    %   queue properties:
    %
    %   MaxPackets  - Maximum number of packets that can be stored in the queue
    %   CurrentSize - Current size of the queue
    %
    %   queue methods:
    %
    %   enqueue  - Add packet to queue
    %   dequeue  - Remove packet from queue
    %   peek     - Return the oldest packet from queue without removing the
    %              packet
    %   isEmpty  - Check if queue is empty
    %   isFull   - Check if queue is full

    %   Copyright 2023 The MathWorks, Inc.

    properties (SetAccess = private)
        %MaxPackets Maximum number of packets that can be stored in FIFO queue
        %   Specified as a positive integer scalar
        MaxPackets

        %CurrentSize Current size indicates the number of packets stored in the
        %FIFO queue
        CurrentSize = 0
    end

    properties (Hidden)
        %Queue Cell array of size N-by-1 where N is the maximum number of packets
        %that can be stored
        Queue
    end

    properties (Access = private)
        %Rear Rear index of queue
        Rear = 0

        %Front Front index of queue
        Front = 1
    end

    methods
        % Constructor
        function obj = queue(maxPackets)

            obj.MaxPackets = maxPackets;
            % Create a queue using cell array
            obj.Queue = cell(maxPackets, 1);
        end

        function isQueued = enqueue(obj, packet)
            %enqueue Add packet to the end of queue
            %
            %   ISQUEUED = enqueue(OBJ, PACKET) adds packet to the end of queue.
            %
            %   ISQUEUED indicates the packet is queued or not. Its values true and
            %   false indicate queued and not queued, respectively.
            %
            %   OBJ is an object of type wirelessnetwork.internal.queue.
            %
            %   PACKET indicates the data to be enqueued. This can be any MATLAB data
            %   type.

            % Check if queue is not full
            if obj.CurrentSize < obj.MaxPackets
                % Add packet into the queue
                obj.Rear = obj.Rear + 1;
                obj.Queue{obj.Rear} = packet;
                if obj.Rear == obj.MaxPackets
                    obj.Rear = 0;
                end
                obj.CurrentSize = obj.CurrentSize + 1;
                isQueued = true;
            else
                % Unable to add packet into the queue because the queue is full
                isQueued = false;
            end
        end

        function packet = dequeue(obj)
            %dequeue Remove the oldest packet from the start of queue
            %
            %   PACKET = dequeue(OBJ) removes the oldest packet from the start of
            %   queue.
            %
            %   PACKET indicates the data to be dequeued from the queue. If the queue
            %   is empty, empty ([]) will be returned.
            %
            %   OBJ is an object of type wirelessnetwork.internal.queue.

            % Check if queue is not empty
            if obj.CurrentSize > 0
                % Remove the oldest packet from the queue
                packet = obj.Queue{obj.Front};
                obj.Front = obj.Front + 1;
                if obj.Front == obj.MaxPackets + 1
                    obj.Front = 1;
                end
                obj.CurrentSize = obj.CurrentSize - 1;
            else
                packet = [];
            end
        end

        function packet = peek(obj)
            %peek Return the oldest packet from queue without removing the packet
            %
            %   PACKET = PEEK(OBJ) returns the oldest packet from the start of queue
            %   without removing the packet from the queue.
            %
            %   PACKET indicates the data to be returned. If the queue is empty, empty
            %   ([]) will be returned.
            %
            %   OBJ is an object of type wirelessnetwork.internal.queue.

            % Check if queue is not empty
            if obj.CurrentSize > 0
                % Return the oldest packet from the queue
                packet = obj.Queue{obj.Front};
            else
                % Return empty packet
                packet = [];
            end
        end

        function queueEmpty = isEmpty(obj)
            %isEmpty Check if the queue is empty
            %
            %   QUEUEEMPTY = isEmpty(OBJ) checks if the queue is empty.
            %
            %   QUEUEEMPTY indicates whether the queue is empty. Its values true and
            %   false to indicate empty queue and non-empty queue, respectively.
            %
            %   OBJ is an object of type wirelessnetwork.internal.queue.

            queueEmpty = (obj.CurrentSize == 0);
        end

        function queueFull = isFull(obj)
            %isFull Check if the queue is full
            %
            %   QUEUEFULL = isFull(OBJ) checks whether the queue is full.
            %
            %   QUEUEFULL indicates whether the queue is full. Its values true and
            %   false to indicate full and not full queue, respectively.
            %
            %   OBJ is an object of type wirelessnetwork.internal.queue.

            queueFull = (obj.CurrentSize == obj.MaxPackets);
        end
    end
end