classdef nr5G_PDSCHPTRS_Dialog < wirelessWaveformApp.nr5G_PXSCHPTRS_Dialog
    % Interface to PDSCH PTRS properties

    %   Copyright 2020-2024 The MathWorks, Inc.

    properties (Dependent)
        % List of properties with custom GETers/SETers. All others from
        % displayOrder() get generic GETer/SETer from Dialog.m
    end

    properties (Dependent, GetAccess = protected)
        ChannelName % Needed by UpdateAppStatus
    end

    properties (Hidden)
        TitleString = 'PDSCH 1 PT-RS'
    end

    % Constructor and public methods defined in the base app
    methods (Access = public)
        function obj = nr5G_PDSCHPTRS_Dialog(parent, fig, invisibleProps)
            obj@wirelessWaveformApp.nr5G_PXSCHPTRS_Dialog(parent, fig, invisibleProps, wirelessWaveformApp.nr5G_PDSCHAdvanced_Dialog.DefaultCfg); % call base constructor
        end

        function props = displayOrder(~)
            props = {'EnablePTRS'; 'Power'; 'TimeDensity'; 'FrequencyDensity'; 'REOffset'; 'PTRSPortSet'};
        end

        function restoreDefaults(obj)
            % This method is called in the baseclass constructor to set the
            % default values of the properties
            % Get the same defaults with programmatic objects
            c = nrWavegenPDSCHConfig;
            obj.Power             = c.PTRSPower;
            obj.EnablePTRS        = c.EnablePTRS;

            c = nrPDSCHPTRSConfig;
            obj.TimeDensity       = c.TimeDensity;
            obj.FrequencyDensity  = c.FrequencyDensity;
            obj.REOffset          = c.REOffset;
            obj.PTRSPortSet       = c.PTRSPortSet;
        end
    end

    % Custom getters/setters
    methods
        function out = get.ChannelName(~)
            out = 'PDSCH';
        end
    end
end
