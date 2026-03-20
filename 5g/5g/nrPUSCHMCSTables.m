classdef nrPUSCHMCSTables < comm.internal.ConfigBase
    %nrPUSCHMCSTables MCS lookup tables for PUSCH
    %   PUSCHMCSTables = nrPUSCHMCSTables creates a physical uplink shared
    %   channel (PUSCH) modulation and coding scheme (MCS) lookup tables
    %   object that provides the tables related to TS 38.214 Section
    %   5.1.3.1 and Section 6.1.4.1
    %
    %   nrPUSCHMCSTables properties:
    %
    %   TransformPrecodingPi2BPSK         - logical value representing
    %                                       whether the higher-layer
    %                                       parameter 'tp-pi2BPSK' is
    %                                       configured (default false)
    %   TransformPrecodingQAM64Table      - MCS index table, corresponding to
    %                                       TS 38.214 Table 6.1.4.1-1
    %   TransformPrecodingQAM64LowSETable - MCS index table, corresponding to
    %                                       TS 38.214 Table 6.1.4.1-2
    %
    %   nrPUSCHMCSTables constant properties:
    %
    %   QAM64Table      - MCS index table 1,
    %                     corresponding to TS 38.214 Table 5.1.3.1-1
    %   QAM256Table     - MCS index table 2,
    %                     corresponding to TS 38.214 Table 5.1.3.1-2
    %   QAM64LowSETable - MCS index table 3,
    %                     corresponding to TS 38.214 Table 5.1.3.1-3
    %
    %   The table columns are MCSIndex, Modulation, Qm, TargetCodeRate, and
    %   SpectralEfficiency. A table value NaN corresponds to the value
    %   "Reserved" from the technical specification.
    %
    %   Example 1:
    %   % Create an nrPUSCHMCSTables object, get the code rate from an MCS
    %   % index.
    %
    %   iMCS = 1;
    %   puschmcsTables = nrPUSCHMCSTables;
    %   puschmcsTable64QAM = puschmcsTables.QAM64Table;
    %   tcr = puschmcsTable64QAM.TargetCodeRate(puschmcsTable64QAM.MCSIndex == iMCS);
    %
    %   Example 2:
    %   % Create an nrPUSCHMCSTables object where the higher-layer
    %   % parameter 'tp-pi2BPSK' is configured and get the code rate from
    %   % an MCS index.
    %
    %   iMCS = 1;
    %   puschmcsTables = nrPUSCHMCSTables;
    %   puschmcsTables.TransformPrecodingPi2BPSK = true;
    %   puschmcsTableTP64QAM = puschmcsTables.TransformPrecodingQAM64Table;
    %   tcr = puschmcsTableTP64QAM.TargetCodeRate(puschmcsTableTP64QAM.MCSIndex == iMCS);
    %
    %   See also nrPDSCHMCSTables, nrCQITables, nrPUSCHConfig.

    %   Copyright 2023 The MathWorks, Inc.

    %#codegen


    % Public, tunable properties
    properties
        %TransformPrecodingPi2BPSK - Logical indicator for 'tp-pi2BPSK'
        %   Represents the higher-layer parameter 'tp-pi2BPSK'. If true the
        %   UE is capable of pi/2 BPSK modulation.
        TransformPrecodingPi2BPSK (1,1) logical = false;
    end

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
    end

    % Public, dependent properties
    properties (Dependent)

        %TransformPrecodingQAM64Table - MCS index table for PUSCH with transform precoding and 64QAM corresponding to TS 38.214 Table 6.1.4.1-1
        %   The table contains a column for the modulation scheme in
        %   addition to the columns from the specification. The
        %   TargetCodeRate column contains the fractional target code
        %   rates, which are obtained by dividing the target code rate
        %   values from the specification by 1024. The table values depend
        %   on the value of TransformPrecodingPi2BPSK.
        TransformPrecodingQAM64Table;

        %TransformPrecodingQAM64LowSETable - MCS index table 2 for PUSCH with transform precoding and 64QAM corresponding to TS 38.214 Table 6.1.4.1-2
        %   The table contains a column for the modulation scheme in
        %   addition to the columns from the specification. The
        %   TargetCodeRate column contains the fractional target code
        %   rates, which are obtained by dividing the target code rate
        %   values from the specification by 1024. The table values depend
        %   on the value of TransformPrecodingPi2BPSK.
        TransformPrecodingQAM64LowSETable;
    end

    methods

        %Constructor
        function obj = nrPUSCHMCSTables(varargin)
            obj@comm.internal.ConfigBase(varargin{:});
            obj.QAM64Table = nr5g.internal.getMCSTable(1);
            obj.QAM256Table = nr5g.internal.getMCSTable(2);
            obj.QAM64LowSETable = nr5g.internal.getMCSTable(3);
        end

        %Method getting Table 6.1.4.1-1 dependent on value of TransformPrecodingPi2BPSK;
        function TransformPrecodingQAM64Table = get.TransformPrecodingQAM64Table(obj)
            q=2-obj.TransformPrecodingPi2BPSK;
            if q > 1
                qMod = 'QPSK';
            else
                qMod = 'pi/2-BPSK';
            end
            MCSIndex = (0:31)';
            Qm = [repmat(q,2,1);repmat(2,8,1);repmat(4,7,1);repmat(6,11,1);q;2;4;6];
            TargetCodeRate=[240/q;314/q;193;251;308;379;449;526;602;679;340;378;434;490;553;616;658;466;517;567;616;666;719;772;822;873;910;948;NaN;NaN;NaN;NaN]/1024;
            SpectralEfficiency= round(TargetCodeRate.*Qm*1e4)/1e4;
            Modulation = [repmat({qMod},2,1);repmat({'QPSK'},8,1);repmat({'16QAM'},7,1);repmat({'64QAM'},11,1);{qMod};{'QPSK'};{'16QAM'};{'64QAM'}];
            TransformPrecodingQAM64Table = table(MCSIndex,Modulation,Qm,TargetCodeRate,SpectralEfficiency,'VariableNames',{'MCSIndex', 'Modulation', 'Qm', 'TargetCodeRate', 'SpectralEfficiency'});
        end

        %Method getting Table 6.1.4.1-2 dependent on value of TransformPrecodingPi2BPSK;
        function TransformPrecodingQAM64LowSETable = get.TransformPrecodingQAM64LowSETable(obj)
            q=2-obj.TransformPrecodingPi2BPSK;
            if q > 1
                qMod = 'QPSK';
            else
                qMod = 'pi/2-BPSK';
            end
            MCSIndex = (0:31)';
            Qm = [repmat(q,6,1);repmat(2,10,1);repmat(4,8,1);repmat(6,4,1);q;2;4;6];
            TargetCodeRate=[60/q;80/q;100/q;128/q;156/q;198/q;120;157;193;251;308;379;449;526;602;679;378;434;490;553;616;658;699;772;567;616;666;772;NaN;NaN;NaN;NaN]/1024;
            SpectralEfficiency= round(TargetCodeRate.*Qm*1e4)/1e4;
            Modulation =[repmat({qMod},6,1);repmat({'QPSK'},10,1);repmat({'16QAM'},8,1);repmat({'64QAM'},4,1);{qMod};{'QPSK'};{'16QAM'};{'64QAM'}];
            TransformPrecodingQAM64LowSETable = table(MCSIndex,Modulation,Qm,TargetCodeRate,SpectralEfficiency,'VariableNames',{'MCSIndex', 'Modulation', 'Qm', 'TargetCodeRate', 'SpectralEfficiency'});
        end
    end
end