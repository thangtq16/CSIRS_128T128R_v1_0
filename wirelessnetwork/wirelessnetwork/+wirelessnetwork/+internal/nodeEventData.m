classdef (ConstructOnLoad) nodeEventData < event.EventData
%nodeEventData Data object passed to event listeners
%
%   Note: This is an internal undocumented class and its API and/or
%   functionality may change in subsequent releases
%
%   nodeEventData creates a data object passed to event listeners. It is
%   used to encapsulate information about an event which can then be passed
%   to event listeners via NOTIFY.
%
%   nodeEventData properties (configurable):
%
%   Data - Structure input containing event information to be passed to
%   registered listener callback. The object that defines the event
%   fills the data.
%
%   nodeEventData properties (read-only):
%
%   EventName - Name of the event described by this object
%   Source    - The object that defines the event described by this object

%   Copyright 2022 The MathWorks, Inc.

properties
    Data
end
end