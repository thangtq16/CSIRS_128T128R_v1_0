classdef Formats34Common < nr5g.internal.pucch.Formats234Common ...
        & nr5g.internal.pucch.Formats0134Common
    %Formats34Common Common configuration object for PUCCH formats 3 and 4
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   Formats34Common properties (configurable):
    %
    %   Modulation       - Modulation scheme ('QPSK' (default), 'pi/2-BPSK')
    %   AdditionalDMRS   - Additional demodulation reference signal (DM-RS)
    %                      configuration flag (0 (default), 1)
    %   DMRSUplinkTransformPrecodingR16 - Enable low PAPR DM-RS sequence for
    %                      DFT-s-OFDM (0 (default), 1). To enable this
    %                      property, set the Modulation property to 'pi/2-BPSK'.
    %   NID              - Data scrambling identity (0...1023) (default [])
    %   RNTI             - Radio network temporary identifier (0...65535)
    %                      (default 1)
    %   GroupHopping     - Group hopping configuration
    %                      ('neither' (default), 'enable', 'disable')
    %   HoppingID        - Hopping identity (0...1023) (default [])

    %   Copyright 2020-2023 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties

        %Modulation Modulation scheme
        %   Specify the modulation scheme as one of {'pi/2-BPSK', 'QPSK'}.
        %   The default value is 'QPSK'.
        Modulation = 'QPSK';

        %AdditionalDMRS Additional DM-RS configuration flag
        %   Specify the additional demodulation reference signal (DM-RS)
        %   configuration as a numeric or logical scalar. The value is
        %   either true or false, provided by higher-layer parameter,
        %   additionalDMRS. This property is used only if the number of
        %   OFDM symbols allocated for physical uplink control channel is
        %   greater than 9. If the value is set to true, there are 4 DM-RS
        %   OFDM symbols. If the value is set to false, there are 2 DM-RS
        %   OFDM symbols. The default value is false.
        AdditionalDMRS (1,1) logical = false;

        %DMRSUplinkTransformPrecodingR16 Low PAPR DM-RS for DFT-s-OFDM
        %   Specify the use of low PAPR DM-RS for DFT-s-OFDM. To enable
        %   this property, set the Modulation property to 'pi/2-BPSK'. When
        %   the property is set to 1, the DM-RS sequence generation uses
        %   type 2 low PAPR sequences, otherwise type 1 low PAPR sequences
        %   are used. The default value is 0.
        DMRSUplinkTransformPrecodingR16 (1,1) logical = false;

    end

    properties(Constant, Hidden)
        Modulation_Values = {'QPSK', 'pi/2-BPSK'};
    end

    methods

        % Self-validate and set properties
        function obj = set.Modulation(obj,val)
            prop = 'Modulation';
            val = validatestring(val,obj.Modulation_Values,...
                [class(obj) '.' prop],prop);
            obj.(prop) = '';
            obj.(prop) = val;
        end

    end

end
