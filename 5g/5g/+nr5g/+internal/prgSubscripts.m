%prgSubscripts get PRG subscripts
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2020-2023 The MathWorks, Inc.

%#codegen

function [prgsubs,crbsubs,sizout,allplanes] = prgSubscripts(siz,nstartgrid,portind,W)

    % Establish the dimensionality of the grid of port indices
    portsiz = siz;
    ndims = max(length(portsiz),3);
    portsiz(ndims) = size(W,1);

    % Using the port subscripts, establish if the input 'siz' is missing a
    % trailing singleton dimension (applies when precoding a channel
    % estimate with one transmit port)
    K = siz(1);
    L = siz(2);
    if (isempty(portind))
        if (ndims==3 && portsiz(ndims)==1)
            sizout = [siz 1];
        else
            sizout = siz;
        end
        maxp_ind = 1;
    else
        maxportind = max(portind,[],'all');
        if (ndims==3)
            maxp_ind = floor((maxportind-1)/(K*L)) + 1;
        else % ndims==4
            R = siz(3);
            maxp_ind = floor((maxportind-1)/(K*L*R)) + 1;
        end
        if (maxp_ind > portsiz(ndims))
            if (ndims==4)
                v = portsiz;
                coder.internal.error( ...
                    'nr5g:nrPDSCHPrecode:indexExceedsDimsLinear', ...
                    maxportind,prod(portsiz),v(1),v(2),v(3),v(4));
            else
                sizout = [siz 1];
            end
        else
            sizout = siz;
        end
    end

    % Get the maximum port number, the largest port number in the indices
    % or the number of layers in the precoder, whichever is larger
    maxp = max(maxp_ind,size(W,1));

    % Establish if the RE indices refer to the same locations in all
    % planes; this is assumed to be false if all indices for multiple
    % planes are in one column
    allplanes = nr5g.internal.precodeIsAllPlanes(siz,portind,maxp);

    % Calculate 1-based RE subscripts from port indices, including reducing
    % port indices to a single column if the RE indices refer to the same
    % locations in all planes
    if (allplanes)
        repdims = [1 size(portind,[2 3])];
        portind = portind(:,1);
    else
        repdims = [1 1 1];
    end
    resubs = mod(portind-1,K) + 1;

    % Calculate 0-based CRB subscripts from RE subscripts
    crbsubs = floor((resubs - 1) / 12);

    % Get the number of precoder resource groups NPRG from the precoder
    % array W
    NPRG = size(W,3);

    if (NPRG==1)

        % For NPRG=1, PRG subscripts are all 1
        prgsubs = ones(size(portind));

    else

        % Get the number of resource blocks from the grid size
        NRB = K / 12;

        % Get the PRG numbers (1-based) for each CRB in the whole carrier
        prgset = nr5g.internal.prgSet(NRB,nstartgrid,NPRG);

        % Calculate 1-based PRG subscripts from CRB subscripts
        prgsubs = prgset(crbsubs + 1);

    end

    % If the RE indices refer to the same locations in all planes, repeat
    % the CRB and PRG subscripts across all planes
    if (allplanes)
        crbsubs = repmat(crbsubs,repdims);
        prgsubs = repmat(prgsubs,repdims);
    end

end
