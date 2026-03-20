classdef nrPDSCHMCSTables < comm.internal.ConfigBase
    %nrPDSCHMCSTables MCS lookup tables for PDSCH
    %   PDSCHMCSTables = nrPDSCHMCSTables creates a physical downlink
    %   shared channel (PDSCH) modulation and coding scheme (MCS) lookup
    %   tables object that provides the tables related to TS 38.214 Section
    %   5.1.3.1.
    %
    %   nrPDSCHMCSTables constant properties:
    %
    %   QAM64Table      - MCS index table 1,
    %                     corresponding to TS 38.214 Table 5.1.3.1-1
    %   QAM256Table     - MCS index table 2,
    %                     corresponding to TS 38.214 Table 5.1.3.1-2
    %   QAM64LowSETable - MCS index table 3,
    %                     corresponding to TS 38.214 Table 5.1.3.1-3
    %   QAM1024Table    - MCS index table 4,
    %                     corresponding to TS 38.214 Table 5.1.3.1-4
    %
    %   The table columns are MCSIndex, Modulation, Qm, TargetCodeRate, and
    %   SpectralEfficiency. A table value NaN corresponds to the value
    %   "Reserved" from the technical specification.
    %
    %   Example 1: 
    %   % Create an nrPDSCHMCSTables object, get the code rate from an MCS
    %   % index.
    %
    %   iMCS = 1; 
    %   pdschmcsTables = nrPDSCHMCSTables;
    %   pdschmcsTable64QAM = pdschmcsTables.QAM64Table;
    %   tcr = pdschmcsTable64QAM.TargetCodeRate(pdschmcsTable64QAM.MCSIndex == iMCS);
    %
    %   See also nrPUSCHMCSTables, nrCQITables, nrPDSCHConfig.

    %   Copyright 2023 The MathWorks, Inc.

    %#codegen

    % Read-only properties 
    properties (SetAccess=private)
        %QAM64Table - MCS index table 1, corresponding to TS 38.214 Table 5.1.3.1-1
        %   The table contains a column for the modulation scheme in
        %   addition to the columns from the specification. The
        %   TargetCodeRate column contains the fractional target code
        %   rates, which are obtained by dividing the target code rate
        %   values from the specification by 1024.
        QAM64Table;

        %QAM256Table - MCS index table 2, corresponding to TS 38.214 Table 5.1.3.1-2
        %   The table contains a column for the modulation scheme in
        %   addition to the columns from the specification. The
        %   TargetCodeRate column contains the fractional target code
        %   rates, which are obtained by dividing the target code rate
        %   values from the specification by 1024.
        QAM256Table;

        %QAM64LowSETable - MCS index table 3, corresponding to TS 38.214 Table 5.1.3.1-3
        %   The table contains a column for the modulation scheme in
        %   addition to the columns from the specification. The
        %   TargetCodeRate column contains the fractional target code
        %   rates, which are obtained by dividing the target code rate
        %   values from the specification by 1024.
        QAM64LowSETable;
        
        %QAM1024Table - MCS index table 4, corresponding to TS 38.214 Table 5.1.3.1-4
        %   The table contains a column for the modulation scheme in
        %   addition to the columns from the specification. The
        %   TargetCodeRate column contains the fractional target code
        %   rates, which are obtained by dividing the target code rate
        %   values from the specification by 1024.
        QAM1024Table;
    end

    methods
        % Constructor
        function obj = nrPDSCHMCSTables()
            obj.QAM64Table = nr5g.internal.getMCSTable(1);
            obj.QAM256Table = nr5g.internal.getMCSTable(2);
            obj.QAM64LowSETable = nr5g.internal.getMCSTable(3);
            obj.QAM1024Table = nr5g.internal.getMCSTable(4);
        end
    end

end