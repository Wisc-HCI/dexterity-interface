import { AppStreamer } from '@nvidia/omniverse-webrtc-streaming-library';

const serverIP = location.hostname || '127.0.0.1';
const signalingPort = 49100; // TCP
const mediaPort = 47998;     // UDP
const sessionId = 'local';   // any string for local dev

const url = `signalingserver=${serverIP}&signalingport=${signalingPort}` +
            `&mediaserver=${serverIP}&mediaport=${mediaPort}` +
            `&sessionid=${sessionId}&mic=0&cursor=free&resolution=1280x720&fps=30&autolaunch=true`;

const streamer = new AppStreamer();
await streamer.setup({
  streamConfig: {
    source: 'local',
    videoElementId: 'remote-video',
    audioElementId: 'remote-audio',
    messageElementId: 'remote-msg',
    urlLocation: { search: url },
  },
  onStart: () => console.log('stream started'),
});
