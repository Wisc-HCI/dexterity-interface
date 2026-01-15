import { get_state, set_state} from "/src/js/state.js";
import {post_primitive} from "/src/js/helpers/api.js"

/**
 * Opens the primitive editor modal and populates fields from a primitive.
 * @param {int| array[int]} editing_index Index of the prim in state plan to edit. If its core/sub primitive, the
 *      index is an array where each index corresponds to each level of the prim hierarchy.
 * @param {str} primitive_model_id DOM ID of model to show/hide
 * @param {str} primitive_modal_content_id DOM ID of model div to add elements too
 */
export function open_primitive_editor(editing_index, primitive_modal_id, primitive_modal_content_id) {
    let prim;
    const plan = get_state().primitive_plan;

    
    if (editing_index.length >= 2) {
        // TODO: Handle multiple levels of prims
        prim = plan[editing_index[0]].core_primitives[editing_index[1]];
    } else {
        prim = plan[editing_index[0]];
    }

    const model = document.getElementById(primitive_modal_id);
    const model_content = document.getElementById(primitive_modal_content_id);
    model_content.innerHTML = ""; // Clear content

    // PRIMITIVE NAME
    const header = document.createElement("div");
    header.id = "primitive_type";
    header.className = "text-2xl font-bold mb-3";
    header.textContent = prim.name ?? "";
    model_content.appendChild(header);

    // PARAMETERS
    for (const [param_name, param_value] of Object.entries(prim.parameters)) {
        
        let formatted_value = param_value;
        if (formatted_value.constructor == Array) {
            // Make arrays readable by only showing first 3 elements
            formatted_value = param_value.join(", "); 
        }

        const label = document.createElement("label");
        label.id = `${param_name}_field`;
        label.className = "block mb-2";

        // Text
        const span = document.createElement("span");
        span.className = "text-sm font-medium";
        span.textContent = `${param_name}`;

        // Input
        let input;
        if (param_name == "arm") {
            input = document.createElement("select");
            const options = ["left", "right"];
            options.forEach(value => {
                const option = document.createElement("option");
                option.value = value;
                option.textContent = value;
                input.appendChild(option);
            });
        } else {
            input = document.createElement("input");
        }
        input.id = `${param_name}_field_input`;
        input.value = formatted_value;
        
        input.className = "w-full border p-2 rounded";

        label.appendChild(span);
        label.appendChild(input);
        
        model_content.appendChild(label);
    }

    model.classList.remove("hidden");

}


/**
 * Saves edits made to the currently selected primitive to state.
 */
export async function save_primitive_edit() {
    const { primitive_plan, editing_index } = get_state();

    let prim;
    if (editing_index.length >= 2) {
        // TODO: Handle multiple levels of prims
        prim = { ...primitive_plan[editing_index[0]].core_primitives[editing_index[1]]};
    } else {

        prim =  { ...primitive_plan[editing_index[0]] };
    }

    for (const [param_name, param_value] of Object.entries(prim.parameters)) {
        let input = document.getElementById(`${param_name}_field_input`).value;
        if (param_value.constructor === Array) {
            input = input.split(",").map(Number);
        }
        prim.parameters[param_name] = input;

    }

    // High level updates trickle down to low-level
    if (prim.core_primitives) {
        prim = await post_primitive(prim);
    }

    
    const updated_plan = [...primitive_plan];
    updated_plan[editing_index] = prim;
    set_state({primitive_plan: updated_plan});
    

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
