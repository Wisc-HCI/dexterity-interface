import {IP} from "/src/js/constants.js"
import { post_objects } from "/src/js/helpers/api.js";
import { AppStreamer, StreamType, LogLevel } from '@nvidia/omniverse-webrtc-streaming-library';

/**
 * Connect to Issacsim stream
 */
export async function start_isaacsim_stream() {

    await AppStreamer.connect({
            streamSource: StreamType.DIRECT,
            logLevel: LogLevel.DEBUG,
            streamConfig: {
            server: IP,
            signalingPort: 49100,
            mediaServer: IP,
            mediaPort: 47998,
            videoElementId: 'remote-video',
            audioElementId: 'remote-audio',
            },
    });
}


/*
TODO
*/
export async function load_objects() {
    await post_objects();
}