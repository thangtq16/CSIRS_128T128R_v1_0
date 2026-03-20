function out = getVariablesFromConfigTable(frequencyRange,duplexMode,ConfigurationIndex)
% getVariablesFromConfigTable gets the value of the desired variable from
% TS 38.211 Tables 6.3.3.2-2 to 6.3.3.2-4, given the value of
% FrequencyRange and DuplexMode.
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%    OUT = getVariablesFromConfigTable(FREQUENCYRANGE,DUPLEXMODE,CONFIGURATIONINDEX)
%    returns a structure containing the value of the element corresponding
%    to CONFIGURATIONINDEX from the correct table, amongst TS 38.211
%    Tables 6.3.3.2-2 to 6.3.3.2-4, given the value of FrequencyRange and
%    DuplexMode.
%    The structure OUT has the following fields:
%
%       * ConfigurationIndex
%       * Format
%       * x
%       * y
%       * sfn
%       * StartingSymbol
%       * SlotsPerSF
%       * NumTimeOccasions
%       * PRACHDuration

%  Copyright 2019-2020 The MathWorks, Inc.

%#codegen
    
    coder.varsize('format',[1 5],[0 1]);
    coder.varsize('y',[1 2],[0 1]);
    coder.varsize('sfn',[1 40],[0 1]);

    if strcmpi(frequencyRange,'FR1')
        if strcmpi(duplexMode,'TDD') % TS 38.211 Table 6.3.3.2-3
            [configIdx,formats,x,ys,sfns,startingSymbol,slotsPerSF,numTimeOccasions,duration] = ...
                nr5g.internal.prach.getConfigurationTables(3);
        else % TS 38.211 Table 6.3.3.2-2
            [configIdx,formats,x,ys,sfns,startingSymbol,slotsPerSF,numTimeOccasions,duration] = ...
                nr5g.internal.prach.getConfigurationTables(2);
        end
    else % TS 38.211 Table 6.3.3.2-4
        [configIdx,formats,x,ys,sfns,startingSymbol,slotsPerSF,numTimeOccasions,duration] = ...
            nr5g.internal.prach.getConfigurationTables(4);
    end
    
    format = formats{ConfigurationIndex+1};
    y = ys{ConfigurationIndex+1};
    sfn = sfns{ConfigurationIndex+1};
    
    out = struct('ConfigurationIndex',configIdx(ConfigurationIndex+1),...
                 'Format',format,...
                 'x',x(ConfigurationIndex+1),...
                 'y',y,...
                 'sfn',sfn,...
                 'StartingSymbol',startingSymbol(ConfigurationIndex+1),...
                 'SlotsPerSF',slotsPerSF(ConfigurationIndex+1),...
                 'NumTimeOccasions',numTimeOccasions(ConfigurationIndex+1),...
                 'PRACHDuration',duration(ConfigurationIndex+1));
    
end