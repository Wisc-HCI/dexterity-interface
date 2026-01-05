import { get_state, set_state} from "/src/js/state.js";


/**
 * Opens the primitive editor modal and populates fields from a primitive.
 * @param {int| array[int]} editing_index Index of the prim in state plan to edit. If its core/sub primitive, the
 *      index is an array where each index corresponds to each level of the prim hierarchy.
 */
export function open_primitive_editor(editing_index) {
    let prim;
    const plan = get_state().plan;

    if (editing_index.constructor === Array) {
        // TODO: Handle multiple levels of prims
        prim = plan[editing_index[0]].core_primitives[editing_index[1]];
    } else {
        prim = plan[editing_index];
    }

 
    // Fill fields
    document.getElementById("primitive_type").innerHTML = prim.name ?? "";

    // ARM field
    if ("arm" in prim.parameters) {
        document.getElementById("arm_field").classList.remove("hidden");
        document.getElementById("edit_arm").value = prim.parameters.arm ?? "";
    } else {
        document.getElementById("arm_field").classList.add("hidden");
    }

    // POSE field
    if ("pose" in prim.parameters) {
        document.getElementById("pose_field").classList.remove("hidden");
        document.getElementById("edit_pose").value = prim.parameters.pose.join(", ");
    } else {
        document.getElementById("pose_field").classList.add("hidden");
    }

    document.getElementById("primitive_modal").classList.remove("hidden");

}


/**
 * Saves edits made to the currently selected primitive to state.
 */
export function save_primitive_edit() {
    // TODO: MAKE HIGH LEVEL UPDATES TRICKLE DOWN
    const { plan, editing_index } = get_state();

    let prim;
    if (editing_index.constructor === Array) {
        // TODO: Handle multiple levels of prims
        prim = { ...plan[editing_index[0]].core_primitives[editing_index[1]]};
    } else {
        prim =  { ...plan[editing_index] };
    }
    
    if ("arm" in prim.parameters) {
        const arm = document.getElementById("edit_arm").value;
        if (arm) prim.parameters.arm = arm;
    } 
    
    if ("pose" in prim.parameters) {
        const pose_str = document.getElementById("edit_pose").value;
        prim.parameters.pose = pose_str.split(",").map(Number);
    }

    const updated_plan = [...plan];
    updated_plan[editing_index] = prim;
    set_state({plan: updated_plan});
    

    close_primitive_editor("primitive_modal");
}


/**
 * Closes the primitive editor modal and clears editing state.
 * @param {string} modal_id The DOM element id of the modal to close.
 */
export function close_primitive_editor(modal_id) {
    document.getElementById(modal_id).classList.add("hidden");
    set_state({editing_index: null});
    
}
