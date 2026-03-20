classdef Formats01Common < nr5g.internal.pucch.Formats0134Common ...
        & nr5g.internal.interlacing.InterlacingConfig
    %Formats01Common Common configuration object for PUCCH formats 0 and 1
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   Formats01Common properties (configurable):
    %
    %   InitialCyclicShift - Initial cyclic shift (0...11) (default 0)
    %   GroupHopping       - Group hopping configuration
    %                        ('neither' (default), 'enable', 'disable')
    %   HoppingID          - Hopping identity (0...1023) (default [])

    %   Copyright 2020-2023 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %InitialCyclicShift Initial cyclic shift
        %   Specify the initial cyclic shift (m_0) as a scalar nonnegative
        %   integer. It must be in range 0...11, provided by higher-layer
        %   parameter initialCyclicShift. The default value is 0.
        InitialCyclicShift (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(InitialCyclicShift, 11)} = 0;
    end

end