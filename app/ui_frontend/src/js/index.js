import { subscribe_state, get_state, set_state} from "/src/js/state.js";
import { start_isaacsim_stream, load_objects} from "/src/js/helpers/simulation.js";
import { populate_timeline, load_latest_timeline, handle_plan_play, init_timeline_scrubber, move_scrubber_to_index} from "/src/js/helpers/timeline.js";
import { open_primitive_editor, save_primitive_edit, close_primitive_editor } from "/src/js/helpers/primitive_editor.js";
import { handle_task_submit } from "/src/js/helpers/task_editor.js";
import {post_plan_cancel} from "/src/js/helpers/api.js"
import pause_icon from "url:/src/assets/svgs/pause.svg";
import play_icon from "url:/src/assets/svgs/play.svg";

document.addEventListener("DOMContentLoaded", async () => {

    const task_submit_btn = document.getElementById("submit_task");
    const play_btn = document.getElementById("play");
    const play_img = document.getElementById("play_img");
    const execute_on_robot_btn = document.getElementById("execute_on_robot");

    await start_isaacsim_stream();

    load_objects();
    // Load most recent prims
    load_latest_timeline();

    init_timeline_scrubber("timeline_viewport", "timeline", "scrubber");

    // Init state listeners
    subscribe_state((state) => {
        populate_timeline(state.plan, "timeline");

        if (state.editing_index !== null) {
            open_primitive_editor(state.editing_index, "primitive_modal", "primitive_modal_content")
        }

        if (state.executing_index) {
            move_scrubber_to_index(state.executing_index,"timeline_viewport", "timeline", "scrubber")
        }

        // End of plan execution
        if (state.executing_index === null || state.pause) {
            play_img.src = play_icon;
        }
    });


    task_submit_btn.addEventListener("click", () => {
        handle_task_submit("task_input");
    });

    // TODO: MOVE TO OTHER FILE??
    play_btn.addEventListener("click", () => {
        if (get_state().pause) {
            set_state({pause: false})
            handle_plan_play(false);
            play_img.src = pause_icon;
        } else {
            set_state({pause: true})
            post_plan_cancel();
            play_img.src = play_icon;
        }
        
    });

    execute_on_robot_btn.addEventListener("click", () => {
        handle_plan_play(true);
    });

    // Primitive Edit Modal
    document.getElementById("cancel_edit").addEventListener("click", () => {
        close_primitive_editor("primitive_modal", );
    });

    // Primitive Edit Modal
    document.getElementById("save_edit").addEventListener("click", async () => {
        await save_primitive_edit();
    });

});
