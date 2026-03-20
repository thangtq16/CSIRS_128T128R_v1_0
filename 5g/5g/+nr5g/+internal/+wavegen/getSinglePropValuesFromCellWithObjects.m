 function vals = getSinglePropValuesFromCellWithObjects(cellWithObjs, propName, varargin)
% This is an internal, undocumented function that can change anytime.   
% This is a codegen-friendly version of such a cellfun call:
% >> bwpID = cellfun(@(x) [x.BandwidthPartID], obj.BandwidthParts);

%   Copyright 2020 The MathWorks, Inc.

%#codegen
  
  % type spec needed to help codegen. zeros initialization with 'like' doesn't
  % suffice, because cell array may be empty
  if nargin > 2
    type = varargin{1};
  else
    type = 'double';
  end

  vals = zeros(1, numel(cellWithObjs), type);
  
  for idx = 1:numel(cellWithObjs)
    vals(idx) = cellWithObjs{idx}.(propName);
  end      
end