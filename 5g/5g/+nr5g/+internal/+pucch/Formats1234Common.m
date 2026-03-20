classdef Formats1234Common
    %Formats1234Common Common configuration object for PUCCH formats 1 to 4
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   Formats1234Common properties (configurable):
    %
    %   OCCI - Orthogonal cover code index (default 0)

    %   Copyright 2021-2023 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %OCCI Orthogonal cover code index
        %   Specify the orthogonal cover code index (OCCI) as a scalar
        %   nonnegative integer. The value must be in the range 0...6 for
        %   format 1 and in the range 0...3 for formats 2, 3, and 4. The
        %   default value is 0. For formats 2 and 3, OCCI only applies when
        %   InterlaceIndex is a scalar.
        OCCI (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(OCCI, 6)} = 0;
    end
end

