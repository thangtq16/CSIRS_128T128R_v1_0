function [ind,info] = nrPRACHIndices(carrier,prach,varargin)
%nrPRACHIndices Physical random access channel indices
%   [IND,INFO] = nrPRACHIndices(CARRIER,PRACH) returns the matrix IND
%   containing physical random access channel (PRACH) resource element
%   indices, as defined in TS 38.211 Section 5.3.2, for the specified
%   carrier and PRACH configurations. IND is empty if the current PRACH
%   preamble is not active in the current subframe or 60 kHz slot, as
%   described in TS 38.211 Section 6.3.3.2. The function also returns
%   additional information, INFO, as a structure with the field:
%
%   PRBSet              - Physical resource block (PRB) indices occupied by
%                         PRACH preamble for PUSCH (0-based)
%
%   CARRIER is a carrier configuration object, <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a>.
%   Only these object properties are relevant for this function:
%
%   SubcarrierSpacing     - Subcarrier spacing in kHz
%                           (15, 30, 60, 120, 240, 480, 960)
%   NSizeGrid             - Number of resource blocks in carrier resource
%                           grid (1...275)
%
%   PRACH is a PRACH configuration object, <a href="matlab:help('nrPRACHConfig')">nrPRACHConfig</a>.
%   Only these object properties are relevant for this function:
%
%   FrequencyRange       - Frequency range (used in combination with
%                          DuplexMode to select a configuration table from
%                          TS 38.211 Table 6.3.3.2-2 to 6.3.3.2-4)
%                          ('FR1', 'FR2')
%   DuplexMode           - Duplex mode (used in combination with
%                          FrequencyRange to select a configuration table
%                          from TS 38.211 Table 6.3.3.2-2 to 6.3.3.2-4)
%                          ('FDD', 'SUL', 'TDD')
%   ConfigurationIndex   - Configuration index, as defined in TS 38.211
%                          Tables 6.3.3.2-2 to 6.3.3.2-4 (0...262)
%   SubcarrierSpacing    - PRACH subcarrier spacing in kHz
%                          (1.25, 5, 15, 30, 60, 120, 480, 960)
%   LRA                  - Length of the Zadoff-Chu preamble sequence
%                          (139, 571, 839, 1151)
%   RBOffset             - Starting RB index of PRACH allocation relative
%                          to carrier resource grid (0...274)
%   FrequencyStart       - Frequency offset of lowest PRACH transmission
%                          occasion in frequency domain with respect to
%                          PRB 0 (0...274)
%   FrequencyIndex       - Index of the PRACH transmission occasions in
%                          frequency domain (0...7)
%   RBSetOffset          - Starting RB index of the uplink RB set for this
%                          PRACH transmission occasion (0...274)
%   ActivePRACHSlot      - Active PRACH slot number within a subframe or a
%                          60 kHz slot (0, 1, 3, 7, 15)
%   NPRACHSlot           - PRACH slot number
%
%   [IND,INFO] = nrPRACHIndices(...,NAME,VALUE) specifies additional
%   options as NAME,VALUE pairs to allow control over the format of the
%   indices:
%
%   'IndexStyle'           - 'index' for linear indices (default)
%                            'subscript' for [subcarrier, symbol, antenna] 
%                            subscript row form
%
%   'IndexBase'            - '1based' for 1-based indices (default) 
%                            '0based' for 0-based indices
%
%   Example:
%   % Generate PRACH indices for the default configurations of
%   % nrCarrierConfig and nrPRACHConfig.
%
%   prach = nrPRACHConfig;
%   carrier = nrCarrierConfig;
%   [ind,info] = nrPRACHIndices(carrier,prach);
%
%   See also nrCarrierConfig, nrPRACHConfig, nrPRACH, nrPRACHGrid.

%   Copyright 2019-2022 The MathWorks, Inc.

    %#codegen
    narginchk(2,6);
    
    % Input validation
    [prach,opts] = parseAndValidateInputs(carrier,prach,varargin{:});

    % Get PRACH indices and additional info
    [ind,info] = nr5g.internal.prach.getIndices(carrier,prach,opts);

end

% Parse and validate inputs
function [prach,opts] = parseAndValidateInputs(carrier,prach,varargin)
    
    % Validate carrier and PRACH inputs
    fcnName = 'nrPRACHIndices';
    prach = nr5g.internal.prach.validatePRACHAndCarrier(carrier,prach,fcnName);

    % Parse options
    optNames = {'IndexStyle','IndexBase'};
    opts = nr5g.internal.parseOptions(fcnName,optNames,varargin{:});
end
