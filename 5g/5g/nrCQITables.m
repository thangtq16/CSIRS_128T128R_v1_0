classdef nrCQITables  < comm.internal.ConfigBase
    %nrCQITables CQI lookup tables
    %   CQITables = nrCQITables creates a channel quality indicator (CQI)
    %   lookup tables object that contains the 4-bit CQI tables, as defined
    %   in TS 38.214 Section 5.2.2.1
    %
    %   nrCQITables constant properties:
    %
    %   CQITable1 - Table containing the 4-bit CQI Table,
    %               corresponding to TS 38.214 Table 5.2.2.1-2
    %   CQITable2 - Table containing the 4-bit CQI Table 2,
    %               corresponding to TS 38.214 Table 5.2.2.1-3
    %   CQITable3 - Table containing the 4-bit CQI Table 3,
    %               corresponding to TS 38.214 Table 5.2.2.1-4
    %   CQITable4 - Table containing the 4-bit CQI Table 4,
    %               corresponding to TS 38.214 Table 5.2.2.1-5
    %
    %   The columns in each table are CQIIndex, Modulation, Qm,
    %   TargetCodeRate, and SpectralEfficiency. A table value NaN
    %   corresponds to the "out of range" value from the technical
    %   specification.
    %
    %   Example 1: 
    %   % Create an nrCQITables object, get the code rate from a CQI
    %   % index.
    %
    %   iCQI = 1; 
    %   cqiTables = nrCQITables;
    %   cqiTableOne = cqiTables.CQITable1;
    %   tcr = cqiTableOne.TargetCodeRate(cqiTableOne.CQIIndex == iCQI);
    %
    %   See also nrPDSCHMCSTables, nrPUSCHMCSTables.

    %   Copyright 2023 The MathWorks, Inc.

    %#codegen 

    % Read-only properties  
    properties (SetAccess=private)
        %CQITable1 - 4-bit CQI table, corresponding to TS 38.214 Table 5.2.2.1-2
        %   The table contains a column Qm for the number of bits per
        %   modulation symbol in addition to the columns from the
        %   specification. The TargetCodeRate column contains the
        %   fractional target code rates, which are obtained by dividing
        %   the target code rate values from the specification by 1024.
        CQITable1;

        %CQITable2 - CQI table 2, corresponding to TS 38.214 Table 5.2.2.1-3
        %   The table contains a column Qm for the number of bits per
        %   modulation symbol in addition to the columns from the
        %   specification. The TargetCodeRate column contains the
        %   fractional target code rates, which are obtained by dividing
        %   the target code rate values from the specification by 1024.
        CQITable2;

        %CQITable3 - CQI table 3, corresponding to TS 38.214 Table 5.2.2.1-4
        %   The table contains a column Qm for the number of bits per
        %   modulation symbol in addition to the columns from the
        %   specification. The TargetCodeRate column contains the
        %   fractional target code rates, which are obtained by dividing
        %   the target code rate values from the specification by 1024.
        CQITable3;

        %CQITable4 - CQI table 4, corresponding to TS 38.214 Table 5.2.2.1-5
        %   The table contains a column Qm for the number of bits per
        %   modulation symbol in addition to the columns from the
        %   specification. The TargetCodeRate column contains the
        %   fractional target code rates, which are obtained by dividing
        %   the target code rate values from the specification by 1024.
        CQITable4;
    end

    methods
        % Constructor
        function obj = nrCQITables()
            obj.CQITable1 = nr5g.internal.getCQITable(1);
            obj.CQITable2 = nr5g.internal.getCQITable(2);
            obj.CQITable3 = nr5g.internal.getCQITable(3);
            obj.CQITable4 = nr5g.internal.getCQITable(4);
        end
   end
    
end