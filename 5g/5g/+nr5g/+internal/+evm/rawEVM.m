function evm = rawEVM(varargin)
%rawEVM Error vector magnitude (EVM) calculation
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   EVM = rawEVM(...) returns a structure EVM containing error vector
%   magnitude information. EVM is a structure with the fields:
%   RMS  - Root Mean Square (RMS) EVM, the square root of the mean square
%          of the EVM across all input values
%   Peak - Peak EVM, the largest EVM value calculated across all
%          input values
%   EV   - The normalized error vector
%
%   EVM = rawEVM(X,R) returns a structure EVM for the input array X given
%   the reference signal array R. The EVM is defined using the error
%   (difference) between the input values X and the reference signal R.
%
%   EVM = rawEVM(EV) returns a structure EVM for the input array EV which
%   is taken to be the normalized error vector
%   EV=(X-R)/sqrt(mean(abs(R.^2))). This allows for peak and RMS EVM
%   calculation for pre-existing normalized error vectors. This can be used
%   for example to calculate the EVM across an array of previous EVM
%   results, by extracting and concatenating the EV fields from the array
%   to form the EV input.

% Copyright 2024-2025 The MathWorks, Inc.

%#codegen

    % Create a persistent comm.EVM object to avoid the re-initialization of
    % the object for each call.
    persistent evmComm
    if isempty(evmComm)
        evmComm = comm.EVM(AveragingDimensions=[1 2]);
        % Enable peak EVM output
        evmComm.MaximumEVMOutputPort = 1;
    end

    if (nargin == 2)
        x = varargin{1};
        r = varargin{2};
        errorVector = x-r;
        expDataType = class(r);
        p = sqrt(mean(abs(r(:).^2)));
        % Get the normalized error vector
        evnorm = errorVector/p;

        % Get the RMS and Peak EVM in %. Convert the inputs to the double
        % data type to accommodate both double and single input types,
        % using a persistent comm.EVM variable.
        [rmsEVM,peakEVM] = evmComm(cast(r,like=1i),cast(x,like=1i));

        % Convert EVM percentages to decimal values and assign to output
        % fields
        evm.EV = evnorm;
        evm.RMS = cast(rmsEVM,expDataType)/100;
        evm.Peak = cast(peakEVM,expDataType)/100;
    else
        % Error vector is the input
        ev = varargin{1};
        evnorm = ev;
        evmsignal = abs(evnorm(:));
        evm.EV = evnorm;
        if isempty(evmsignal)
            expDataType = class(ev);
            evm.RMS = nan(1,expDataType);
            evm.Peak = nan(1,expDataType);
        else
            evm.RMS = sqrt(mean(evmsignal.^2));
            evm.Peak = max(evmsignal);
        end
    end

end
