import {IP} from "/src/js/constants.js"
import { post_objects, post_scene_freeze, post_scene_unfreeze } from "/src/js/helpers/api.js";
import { AppStreamer, StreamType, LogLevel } from '@nvidia/omniverse-webrtc-streaming-library';

/**
 * Connect to Issacsim stream and display in remote-video
 * and remote-audio element.
 */
export async function start_isaacsim_stream() {

    // TODO: Prevent multiple clients from connecting to livestream
    await AppStreamer.connect({
            streamSource: StreamType.DIRECT,
            logLevel: LogLevel.WARN,
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


/**
 * Spawn the objects in simulation.
 * @param {boolean} force If true, spawns all objects regardless of position change. Defaults to true.
 */
export async function load_objects(force = true) {
    await post_objects(force);
}


/**
 * Freeze the scene: clears the polling interval and locks the current YOLO scene on the backend.
 * @param {number|null} interval The active scene tracking interval to clear.
 * @returns {null} Always returns null to assign back to the interval variable.
 */
export async function freeze_scene(interval) {
    clearInterval(interval);
    await post_scene_freeze();
    return null;
}


/**
 * Unfreeze the scene: unlocks the backend scene and resumes polling for object updates.
 * @returns {number} The new interval ID for the scene tracking poll.
 */
export async function unfreeze_scene() {
    await post_scene_unfreeze();
    let spawning = false;
    return setInterval(async () => {
        if (spawning) return;
        spawning = true;
        try { await load_objects(false); } finally { spawning = false; }
    }, 500);
}