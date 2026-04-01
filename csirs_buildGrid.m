function [csirs, txGrid_2slots, allCsirsInd, allCsirsSym, slotAssign, cdmLengths] = ...
    csirs_buildGrid(carrier, nTxAntennas)
%CSIRS_BUILDGRID  Configure 4 NZP-CSI-RS resources (Row 18, CDM8) and
%  generate the 2-slot TX resource grid.
%
%  Inputs:
%    carrier      - nrCarrierConfig (NSizeGrid, SubcarrierSpacing set by caller)
%    nTxAntennas  - total TX antennas (128)
%
%  Outputs:
%    csirs         - {1x4} cell of nrCSIRSConfig objects
%    txGrid_2slots - [K x 28 x nTx] combined 2-slot resource grid
%    allCsirsInd   - {1x4} cell of CSI-RS indices per resource
%    allCsirsSym   - {1x4} cell of CSI-RS symbols per resource
%    slotAssign    - [1x4] slot index (0 or 1) per resource
%    cdmLengths    - [1x2] CDM lengths for nrChannelEstimate ([2 4])
%
%  Layout (Row 18, CDM8 = FD-CDM2 x TD-CDM4):
%    Slot 0: R0 l0=2 (ports 0-31),  R1 l0=8 (ports 32-63)
%    Slot 1: R2 l0=2 (ports 64-95), R3 l0=8 (ports 96-127)

nPortsPerRes = nTxAntennas / 4;   % 32
nResources   = 4;

l0_values      = [2, 8, 2, 8];
slotAssign     = [0, 0, 1, 1];
subcarrierLoc  = [0, 2, 4, 6];    % Row 18: FD-CDM2, density 0.5
cdmLengths     = [2, 4];           % [FD-CDM2, TD-CDM4]
powerScale     = db2mag(0);        % 0 dB power offset

% ── Build CSI-RS config objects ──────────────────────────────────────────
csirs = cell(1, nResources);
for r = 1:nResources
    csirs{r} = nrCSIRSConfig;
    csirs{r}.CSIRSType           = {'nzp'};
    csirs{r}.CSIRSPeriod         = 'on';
    csirs{r}.RowNumber           = 18;
    csirs{r}.Density             = {'one'};
    csirs{r}.SymbolLocations     = {l0_values(r)};
    csirs{r}.SubcarrierLocations = {subcarrierLoc};
    csirs{r}.NumRB               = carrier.NSizeGrid;
    csirs{r}.NID                 = carrier.NCellID;
end

% ── Generate symbols & indices, map to per-slot grids ────────────────────
carrier.NSlot = 0;
grid_slot0 = nrResourceGrid(carrier, nTxAntennas);
carrier.NSlot = 1;
grid_slot1 = nrResourceGrid(carrier, nTxAntennas);

allCsirsInd = cell(1, nResources);
allCsirsSym = cell(1, nResources);

gridSizeFull = [carrier.NSizeGrid*12, carrier.SymbolsPerSlot, nTxAntennas];

for r = 1:nResources
    slotNum        = slotAssign(r);
    carrier.NSlot  = slotNum;
    portOffset     = (r-1) * nPortsPerRes;

    sym = nrCSIRS(carrier, csirs{r}) * powerScale;
    ind = nrCSIRSIndices(carrier, csirs{r});

    % Remap port dimension: local 32-port → full 128-port index space
    gridSizePerRes = [carrier.NSizeGrid*12, carrier.SymbolsPerSlot, nPortsPerRes];
    [scInd, symInd, portInd] = ind2sub(gridSizePerRes, ind);
    portInd  = portInd + portOffset;
    fullInd  = sub2ind(gridSizeFull, scInd, symInd, portInd);

    if slotNum == 0
        grid_slot0(fullInd) = sym;
    else
        grid_slot1(fullInd) = sym;
    end

    allCsirsInd{r} = ind;
    allCsirsSym{r} = sym;
end

% ── Concatenate into 2-slot grid [K x 28 x 128] ─────────────────────────
txGrid_2slots = [grid_slot0, grid_slot1];

end
