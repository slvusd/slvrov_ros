const mockState = {
  streamConnected: false,
  controlsEnabled: false,
};

document.documentElement.dataset.stream = mockState.streamConnected ? "connected" : "offline";
document.documentElement.dataset.controls = mockState.controlsEnabled ? "enabled" : "disabled";
