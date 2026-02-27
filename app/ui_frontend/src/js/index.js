import { subscribe_state_with_prev, get_state, set_state} from "/src/js/state.js";
import { start_isaacsim_stream, load_objects, freeze_scene, unfreeze_scene } from "/src/js/helpers/simulation.js";
import { populate_timeline, load_latest_timeline, handle_plan_play} from "/src/js/helpers/timeline.js";
import { open_primitive_editor, save_primitive_edit, close_primitive_editor, open_add_primitive_editor, delete_primitive} from "/src/js/helpers/primitive_editor.js";
import {populate_task_history, handle_task_submit } from "/src/js/helpers/task_editor.js";
import {post_plan_cancel} from "/src/js/helpers/api.js"
import pause_icon from "url:/src/assets/svgs/pause.svg";
import play_icon from "url:/src/assets/svgs/play.svg";

document.addEventListener("DOMContentLoaded", async () => {
    
    // Set conditions via url
    const p = new URLSearchParams(window.location.search);
    const SHOW_PLAN = p.get('show_plan') !== 'false';  // Default to true
    const ALLOW_PLAN_INTERACTION = p.get('plan_interaction') !== 'false'; // default to ture

    const TIMELINE_ID = "timeline";
    const TASK_HISTORY_ID = "task_history";
    const PRIMITIVE_MODAL_ID = "primitive_modal";
    const SAVE_BTN_ID = "save_edit";

    
    const task_submit_btn = document.getElementById("submit_task");
    const play_btn = document.getElementById("play");
    const play_img = document.getElementById("play_img");
    const execute_on_robot_btn = document.getElementById("execute_on_robot");
    const add_btn = document.getElementById("add_primitive");
    const freeze_scene_btn = document.getElementById("freeze_scene");

    let scene_tracking_interval = null;

    await start_isaacsim_stream();

    // Load everything that comes from backend
    // load_objects();
    load_latest_timeline(); 

    // Init state listeners
    subscribe_state_with_prev(async (state, prev_state) => {
        
        if (SHOW_PLAN) {
            populate_timeline(state.primitive_plan, TIMELINE_ID, ALLOW_PLAN_INTERACTION);
        }
        
        // Open when toggled
        if (state.editing_index !== null && prev_state.editing == null) {
            open_primitive_editor(state.editing_index, "primitive_modal", "primitive_modal_content");
        }

        if (!prev_state || state.id != prev_state.id) {
            populate_task_history(TASK_HISTORY_ID, "task_history_label", );
        }

        // Update button labels
        freeze_scene_btn.textContent = state.scene_frozen ? "Track Scene" : "Freeze Scene";
        play_img.src = state.pause ? play_icon : pause_icon;

        // Handle freeze/unfreeze when scene_frozen changes
        if (!prev_state || state.scene_frozen !== prev_state.scene_frozen) {
            if (state.scene_frozen) scene_tracking_interval = await freeze_scene(scene_tracking_interval);
            else scene_tracking_interval = await unfreeze_scene();
        }

    });


    task_submit_btn.addEventListener("click", () => {
        handle_task_submit("task_input");
        set_state({ scene_frozen: true });
    });

    play_btn.addEventListener("click", () => {
        if (get_state().pause) {
            set_state({pause: false});
            handle_plan_play(false);
            set_state({ scene_frozen: true });
            
        } else {
            set_state({pause: true})
            post_plan_cancel();
        }
        
    });

    if (SHOW_PLAN && ALLOW_PLAN_INTERACTION) {
        add_btn.addEventListener("click", () => {
            open_add_primitive_editor("add_primitive_modal", "add_primitive_modal_content", "save_add");
        })
    } else if (!SHOW_PLAN || !ALLOW_PLAN_INTERACTION) {
        add_btn.classList.add("hidden");
    }


    execute_on_robot_btn.addEventListener("click", () => {
        handle_plan_play(true);
    });

    freeze_scene_btn.addEventListener("click", () => {
        set_state({ scene_frozen: !get_state().scene_frozen });
    });

    document.getElementById("cancel_add").addEventListener("click", () => {
        close_primitive_editor("add_primitive_modal");
    })

   
    // Primitive Edit Modal
    document.getElementById("cancel_edit").addEventListener("click", () => {
        close_primitive_editor(PRIMITIVE_MODAL_ID);
    });

    // Primitive Edit Modal
    document.getElementById(SAVE_BTN_ID).addEventListener("click", async () => {
        await save_primitive_edit(PRIMITIVE_MODAL_ID, SAVE_BTN_ID);
    });

    // Primitive Edit Modal
    document.getElementById("delete_primitive").addEventListener("click", async () => {
        delete_primitive(PRIMITIVE_MODAL_ID);
    });




});
