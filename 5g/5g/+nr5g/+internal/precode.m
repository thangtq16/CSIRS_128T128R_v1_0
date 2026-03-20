%precode Precoding of symbols and projection of the corresponding indices
%    
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2020-2025 The MathWorks, Inc.

%#codegen

function [symout,indout] = precode(siz,symin,indin,W,bandsubs,allplanes)

    if nargin<6
        % By default, assume allplanes is false
        allplanes = 0;
        if nargin<5
            % By default, assume no PDSCH PRG bundling or it is not
            % applicable
            bandsubs = ones(size(indin));
        end
    end

    % Dimensionality information
    ndims = max(length(siz),3);
    outplanedim = size(W,2);
    nBands = size(W,3);
    outdims = ones(1,4);
    if (ndims==3)
        outdims(1:3) = [siz(1:2) outplanedim];
    else % ndims==4
        outdims(1:4) = [siz(1:3) outplanedim];
    end

    % Get the number of resource elements in the symbols (assuming that
    % the indices are the same size)
    NRE = size(symin,1);

    if (allplanes && (nBands==1 || NRE==0))

        % Create antenna indices
        indout = allPlanesIndices(outdims,indin);

        if (NRE==0)

            % Create antenna symbols with the same size as the antenna
            % indices and the same type as the input symbols
            symout = zeros(size(indout),'like',symin);

        else

            if (ndims==3)

                % Perform matrix multiplication
                symout = symin * W;

            else

                % Reshape the 3-D symbol array into a matrix, perform
                % matrix multiplication, and reshape the output back to a
                % 3-D array
                symmat = reshape(symin,[],siz(4));
                symout = reshape(symmat * W,[],outdims(3),outdims(4));

            end

        end

    elseif (allplanes && all(bandsubs==bandsubs(:,1),'all'))

        % The band subscripts are the same in all planes, so keep just the
        % first plane
        bandsubs = bandsubs(:,1);

        % Create antenna indices
        indout = allPlanesIndices(outdims,indin);

        % Create antenna symbols with the same size as the antenna indices
        % and the same type as the input symbols
        symout = zeros(size(indout),'like',symin);

        % For each band
        for band = 1:nBands

            % Create array of logical indices indicating which indices and
            % symbols correspond to this band
            thisband = (bandsubs==band);

            % If no indices or symbols correspond to this band, move to the
            % next band
            if (~any(thisband))
                continue;
            end

            % Perform precoding for this band
            if (ndims==3)

                % Perform matrix multiplication and assign into this band
                % of the output
                symout(thisband,:) = symin(thisband,:) * W(:,:,band);
                
            else

                % Reshape the 3-D symbol array for this band into a matrix,
                % perform matrix multiplication, reshape the output back to
                % a 3-D array, and assign into this band of the output
                bandsymmat = reshape(symin(thisband,:,:),[],siz(4));
                bandsymout = bandsymmat * W(:,:,band);
                bandsymout = reshape(bandsymout,[],outdims(3),outdims(4));
                symout(thisband,:,:) = bandsymout;

            end

        end

    else

        % Get port grid size
        portgridsiz = [prod(outdims(1:ndims-1)) size(W,1)];

        P = size(W,2);
        bfgrid = zeros([portgridsiz(1),P],'like',symin);
        
        % For each band
        for band = 1:nBands

            % Create array of logical indices indicating which indices
            % and symbols correspond to this band
            thisband = (bandsubs==band);

            % If no indices or symbols correspond to this band, move to
            % the next band
            if (~any(thisband))
                continue;
            end

            % Create empty port grid
            portgrid = complex(zeros(portgridsiz,'like',symin));

            % Assign symbols for this band into the port grid
            portgrid(indin(thisband)) = symin(thisband);

            s = unique(mod(indin(thisband)-1,portgridsiz(1))+1);
            bfgrid(s,:) = portgrid(s,:) * W(:,:,band);

        end

        antgrid = reshape(bfgrid,outdims);

        % Extract antenna symbols and antenna indices
        [symout,indout] = nrExtractResources(indin,antgrid);

    end

end

function indout = allPlanesIndices(outdims,indin)

    K = outdims(1);
    L = outdims(2);
    P = prod(outdims(3:4));
    indout = indin(:,1) + (0:(P-1))*K*L;
    indout = reshape(indout,[size(indin,1) outdims(3:4)]);

end
