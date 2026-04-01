function channel = csirs_channelSetup(carrier, gnbArraySize, ueArraySize, channelCfg)
%CSIRS_CHANNELSETUP  Create and configure nrCDLChannel object.
%
%  Inputs:
%    carrier      - nrCarrierConfig (used to set SampleRate)
%    gnbArraySize - [Nv Nh Npol Mg Ng] e.g. [4 16 2 1 1] → 128 ports
%    ueArraySize  - [Nv Nh Npol Mg Ng] e.g. [1  2 2 1 1] →   4 ports
%    channelCfg   - struct with fields:
%                     .DelayProfile        (e.g. 'CDL-B')
%                     .DelaySpread         (s,  e.g. 450e-9)
%                     .MaximumDopplerShift (Hz, e.g. 136)
%                     .CarrierFrequency    (Hz, e.g. 4.9e9)
%
%  Output:
%    channel - configured nrCDLChannel object (not yet run)
%
%  Note: Call release(channel) before each new realization in Monte Carlo.

channel = nrCDLChannel;
channel.DelayProfile        = channelCfg.DelayProfile;
channel.DelaySpread         = channelCfg.DelaySpread;
channel.MaximumDopplerShift = channelCfg.MaximumDopplerShift;
channel.CarrierFrequency    = channelCfg.CarrierFrequency;

channel.TransmitAntennaArray.Size           = gnbArraySize;
channel.TransmitAntennaArray.ElementSpacing = [0.5 0.5 1 1];
channel.ReceiveAntennaArray.Size            = ueArraySize;
channel.ReceiveAntennaArray.ElementSpacing  = [0.5 0.5 1 1];

ofdmInfo           = nrOFDMInfo(carrier);
channel.SampleRate = ofdmInfo.SampleRate;

end
