
import { AppStreamer, StreamType, LogLevel } from '@nvidia/omniverse-webrtc-streaming-library';

const server = '127.0.0.1';          // hardcode to avoid the library parsing window.location
const signalingPort = 49100;          // Isaac Sim signaling (TCP)
const mediaPort = 47998;              // Isaac Sim media (UDP)


await AppStreamer.connect({
  streamSource: StreamType.DIRECT,  
  logLevel: LogLevel.DEBUG,
  streamConfig: {
    server,
    signalingPort,
    mediaServer: server,
    mediaPort,
    videoElementId: 'remote-video',
    audioElementId: 'remote-audio',
  },
});