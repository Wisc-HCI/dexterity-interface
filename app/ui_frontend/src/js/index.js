import { subscribe_state_with_prev, get_state, set_state} from "/src/js/state.js";
import { start_isaacsim_stream, load_objects} from "/src/js/helpers/simulation.js";
import { populate_timeline, load_latest_timeline, handle_plan_play, init_timeline_scrubber, move_scrubber_to_index} from "/src/js/helpers/timeline.js";
import { open_primitive_editor, save_primitive_edit, close_primitive_editor, open_add_primitive_editor, delete_primitive} from "/src/js/helpers/primitive_editor.js";
import {populate_task_history, handle_task_submit } from "/src/js/helpers/task_editor.js";
import {post_plan_cancel, post_scene_freeze, post_scene_unfreeze} from "/src/js/helpers/api.js"
import pause_icon from "url:/src/assets/svgs/pause.svg";
import play_icon from "url:/src/assets/svgs/play.svg";

document.addEventListener("DOMContentLoaded", async () => {
    const TIMELINE_ID = "timeline";
    const TASK_HISTORY_ID = 'task_history';
    
    const task_submit_btn = document.getElementById("submit_task");
    const play_btn = document.getElementById("play");
    const play_img = document.getElementById("play_img");
    const execute_on_robot_btn = document.getElementById("execute_on_robot");
    const add_btn = document.getElementById("add_primitive");
    const freeze_scene_btn = document.getElementById("freeze_scene");

    let scene_tracking_interval = setInterval(load_objects, 300);

    await start_isaacsim_stream();

    // Load everything that comes from backend
    load_objects();
    load_latest_timeline(); 

    // Init state listeners
    subscribe_state_with_prev((state, prev_state) => {
        
        populate_timeline(state.primitive_plan, TIMELINE_ID);
        

        if (state.editing_index !== null) {
            open_primitive_editor(state.editing_index, "primitive_modal", "primitive_modal_content");
        }


        // End of plan execution
        if ((prev_state && prev_state.executing_index != null && state.executing_index == null) || state.pause) {
            play_img.src = play_icon;
        }

        if (!prev_state || state.id != prev_state.id) {
            populate_task_history(TASK_HISTORY_ID);
        }

        // Update freeze button label
        freeze_scene_btn.textContent = state.scene_frozen ? "Track Scene" : "Freeze Scene";
    });


    task_submit_btn.addEventListener("click", () => {
        handle_task_submit("task_input");
    });

    play_btn.addEventListener("click", () => {
        if (get_state().pause) {
            set_state({pause: false});
            handle_plan_play(false);
            play_img.src = pause_icon;
        } else {
            set_state({pause: true})
            post_plan_cancel();
            play_img.src = play_icon;
        }
        
    });

    add_btn.addEventListener("click", () => {
        open_add_primitive_editor("add_primitive_modal", "add_primitive_modal_content");
    })


    execute_on_robot_btn.addEventListener("click", () => {
        handle_plan_play(true);
    });

    document.getElementById("cancel_add").addEventListener("click", () => {
        close_primitive_editor("add_primitive_modal");
    })


    // Primitive Edit Modal
    document.getElementById("cancel_edit").addEventListener("click", () => {
        close_primitive_editor("primitive_modal");
    });

    // Primitive Edit Modal
    document.getElementById("save_edit").addEventListener("click", async () => {
        await save_primitive_edit();
    });

    // Primitive Edit Modal
    document.getElementById("delete_primitive").addEventListener("click", async () => {
        delete_primitive("primitive_modal");

    });

    freeze_scene_btn.addEventListener("click", async () => {
        if (get_state().scene_frozen) {
            await post_scene_unfreeze();
            set_state({ scene_frozen: false });
            scene_tracking_interval = setInterval(load_objects, 300);
        } else {
            clearInterval(scene_tracking_interval);
            scene_tracking_interval = null;
            await post_scene_freeze();
            set_state({ scene_frozen: true });
        }
    });

});
