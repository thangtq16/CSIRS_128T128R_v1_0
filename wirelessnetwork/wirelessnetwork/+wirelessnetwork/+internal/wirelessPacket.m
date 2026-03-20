function packet = wirelessPacket()
    %wirelessPacket Return the wireless packet structure
    %
    %   Note: This is an internal undocumented function and its API and/or
    %   functionality may change in subsequent releases
    %
    %PACKET - It is a structure with these fields
    %   Type                - Type of the signal. Accepted values are
    %                         0, 1, 2, 3, and 4, which represent invalid
    %                         packet, WLAN, 5G, Bluetooth LE, and Bluetooth
    %                         BR/EDR packets, respectively. The default
    %                         value is 0.
    %   TransmitterID       - Transmitter node identifier. It is a
    %                         positive scalar integer. The default value is [].
    %   TransmitterPosition - Position of transmitter, specified as a
    %                         real-valued vector in cartesian coordinates
    %                         [x y z] in meters. The default value is [].
    %   TransmitterVelocity - Velocity (v) of transmitter in the x- y-, and
    %                         z-directions, specified as a real-valued
    %                         vector of the form [vx vy vz] in meters per
    %                         second. The default value is [0 0 0].
    %   NumTransmitAntennas - Number of antennas at the transmitter. The
    %                         default value is [].
    %   StartTime           - Packet transmission start time at the
    %                         transmitter or packet arrival time at the
    %                         receiver in seconds. The default value is [].
    %   Duration            - Duration of the packet in seconds. The
    %                         default value is [].
    %   Power               - Average power of the packet in dBm. The
    %                         default value is [].
    %   CenterFrequency     - Center frequency of the carrier in Hz. The
    %                         default value is [].
    %   Bandwidth           - Carrier bandwidth in Hz. It is the
    %                         bandwidth around the center frequency.The default
    %                         value is [].
    %   Abstraction         - A logical scalar representing the abstraction
    %                         type. It takes a value of true or false which
    %                         represents abstracted PHY, or full PHY
    %                         respectively. The default value is false.
    %   SampleRate          - Sample rate of the packet, in samples per
    %                         second. It is only applicable when
    %                         Abstraction value is set to false. The
    %                         default value is [].
    %   DirectToDestination - A numeric integer scalar. A value of 0 indicates
    %                         it is a normal packet and is transmitted over
    %                         the channel. A nonzero value represents a
    %                         destination node ID and also indicates that
    %                         it is a special packet, where the channel
    %                         model is bypassed and transmitted directly to
    %                         the destination node. The default value is 0.
    %   Data                - Contains time samples (full PHY) or frame
    %                         information (abstracted PHY). If Abstraction
    %                         is set to false, this field contains
    %                         time-domain samples of the packet represented
    %                         as a T-by-R matrix of complex values. T is
    %                         the number of time-domain samples. R is the
    %                         number of transmitter antennas if the packet
    %                         represents the transmitted packet or number
    %                         of receiver antennas if the packet represents
    %                         the received packet. If Abstraction is set to
    %                         true, this field contains the standard specific
    %                         information.
    %   Metadata            - A structure representing the technology-specific,
    %                         abstraction-specific, and channel information.
    %                         It contains the following fields.
    %             Channel - It is a structure representing the channel
    %             information with these fields:
    %                 PathGains - Complex path gains at each snapshot in
    %                 time. It is a matrix of size Ncs-by-Np-by-Nt-by-Nr. The
    %                 default value is [].
    %                 PathDelays - Delays in seconds corresponding to each
    %                 path. It is a vector of size 1-by-Np. The default value
    %                 is [].
    %                 PathFilters - Filter coefficients for each path. It is a
    %                 matrix of size Np-by-Nf. The default value is [].
    %                 SampleTimes - Simulation time in seconds corresponding to
    %                 each path gains snapshot. It is a vector of size
    %                 Ncs-by-1. The default value is [].
    %             Here Ncs, Np, Nt, Nr, and Nf represents number of channel
    %             snapshots, number of paths, number of transmit antennas,
    %             number of receive antennas, and number of filter coefficients
    %             respectively.
    %   Tags                - Array of structures where each structure contains these
    %                         fields.
    %                         Name      - Name of the tag.
    %                         Value     - Data associated with the tag.
    %                         ByteRange - Specific range of bytes within the packet to
    %                         which the tag applies.

    %   Copyright 2021-2024 The MathWorks, Inc.

    packet = struct('Type', 0, ...
        'TransmitterID', [], ....
        'TransmitterPosition', [], ...
        'TransmitterVelocity', [0 0 0], ...
        'NumTransmitAntennas', [], ...
        'StartTime', [], ...
        'Duration', [], ...
        'Power', [], ...
        'CenterFrequency', [], ...
        'Bandwidth', [], ...
        'Abstraction', false, ...
        'SampleRate', [], ...
        'DirectToDestination', 0, ...
        'Data', [], ...
        'Metadata', struct('Channel', ...
         struct('PathGains', [], ...
        'PathDelays', [], ...
        'PathFilters', [], ...
        'SampleTimes', [])), ...
        'Tags', []);
end
