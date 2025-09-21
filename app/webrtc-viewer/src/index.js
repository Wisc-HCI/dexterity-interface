// import { AppStreamer, StreamType, LogLevel } from '@nvidia/omniverse-webrtc-streaming-library';

// await AppStreamer.connect({
//   streamSource: StreamType.DIRECT,           // <— THIS
//   logLevel: LogLevel.INFO,
//   streamConfig: {
//     server: location.hostname || '127.0.0.1',
//     signalingPort: 49100,                    // default Isaac Sim signaling (TCP)
//     mediaServer: location.hostname || '127.0.0.1',
//     mediaPort: 47998,                        // default media (UDP)
//     width: 1280, height: 720, fps: 30,
//     videoElementId: 'remote-video',
//     audioElementId: 'remote-audio',
//     // DO NOT include messageElementId unless you actually have it
//   },
// });
import { AppStreamer, StreamType, LogLevel } from '@nvidia/omniverse-webrtc-streaming-library';

const server = '127.0.0.1';          // hardcode to avoid the library parsing window.location
const signalingPort = 49100;          // Isaac Sim signaling (TCP)
const mediaPort = 47998;              // Isaac Sim media (UDP)

console.log('StreamType.DIRECT =', StreamType.DIRECT); // sanity check

// Ensure the elements exist
if (!document.getElementById('remote-video')) throw new Error('Missing #remote-video');
if (!document.getElementById('remote-audio')) throw new Error('Missing #remote-audio');

await AppStreamer.connect({
  streamSource: StreamType.DIRECT,     // <- IMPORTANT
  logLevel: LogLevel.DEBUG,
  // DO NOT pass urlLocation, backendUrl, or anything mentioning /v2/session
  streamConfig: {
    server,
    signalingPort,
    mediaServer: server,
    mediaPort,
    videoElementId: 'remote-video',
    audioElementId: 'remote-audio',
  },
  onStart:  (m) => console.log('onStart', m),
  onUpdate: (m) => console.log('onUpdate', m),
  onError:  (e) => console.error('onError', e),
  onStop:   (m) => console.log('onStop', m),
});