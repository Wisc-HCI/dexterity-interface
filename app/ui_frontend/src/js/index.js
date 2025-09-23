
import { AppStreamer, StreamType, LogLevel } from '@nvidia/omniverse-webrtc-streaming-library';



(async () => {
  await AppStreamer.connect({
    streamSource: StreamType.DIRECT,
    logLevel: LogLevel.DEBUG,
    streamConfig: {
      server: '127.0.0.1',
      signalingPort: 49100,
      mediaServer: '127.0.0.1',
      mediaPort: 47998,
      videoElementId: 'remote-video',
      audioElementId: 'remote-audio',
    },
  });
})();