function mat = addRowAndColumn(row, column)
% This is an internal, undocumented function that can change anytime. It is
% currently used to perform this command in a codegen-friendly manner:
% >> mat = row+column;

%   Copyright 2020 The MathWorks, Inc.

%#codegen

[a, b] = meshgrid(row, column);
mat = a + b;
