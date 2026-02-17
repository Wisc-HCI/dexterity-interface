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


/**
 * Deletes the primitive that is currently being edited
 * @param {string} modal_id The DOM element id of the modal to close
 *  after deleting
 */
export function delete_primitive(modal_id) {
    const { primitive_plan, editing_index } = get_state();

    if (editing_index == null) return;

    let plan = structuredClone(primitive_plan);
    if (editing_index.length >= 2) {
        // TODO: Handle multiple levels of prims
        plan[editing_index[0]].core_primitives.splice(editing_index[1], 1);
    } else {
        plan.splice(editing_index[0], 1);
    }

    close_primitive_editor(modal_id);
    set_state({primitive_plan: plan});
}


///////////////////////////////////////////
//            Add primitive
//////////////////////////////////////////

// TODO: REPLACE with reading from file
const PRIMITIVE_LIBRARY = {
  low_level: [
    { name: "home", parameters: {} },
    { name: "move_to_pose", parameters: { arm: "left", pose: [] } },
    { name: "envelop_grasp", parameters: { arm: "left" } },
    { name: "release", parameters: { arm: "left" } },
  ],
  mid_level: [
    { name: "pick",
      parameters: { arm: "left", grasp_pose: [], end_position: [] } },
    { name: "pour",
      parameters: { arm: "left", initial_pose: [],
        pour_orientation: [], pour_hold: 1.0, } },
  ],
};

/**
 * Renders parameter input fields for the given primitive definition.
 * @param {Object} primitive Primitive definition object from the 
 *      primitive library.
 * @param {HTMLElement} params_container DOM element into which parameter 
 *      input fields will be rendered.
 */
function render_params(primitive, params_container) {
    params_container.innerHTML = "";

    for (const [param, default_value] of Object.entries(primitive.parameters)) {

        // TODO: FUNCTIONALIZE THIS TO SHARE WITH PRIM EDITOR???
        const label = document.createElement("label");
        label.className = "block mb-2";

        const span = document.createElement("span");
        span.className = "text-sm font-medium";
        span.textContent = param;

        let input;
        if (param === "arm") {
            input = document.createElement("select");
            ["left", "right"].forEach(v => {
                const options = document.createElement("option");
                options.value = v;
                options.textContent = v;
                input.appendChild(options);
            });
        } else {
            input = document.createElement("input");
        }

        input.id = `add_${param}`;
        input.className = "w-full border p-2 rounded";
        input.value = Array.isArray(default_value) ? default_value.join(",") : default_value;

        label.appendChild(span);
        label.appendChild(input);
        params_container.appendChild(label);
    }
}


/**
 * Opens the add primitive modal and builds a UI that allows the user
 * to select a primitive and its parameters.
 *
 * @param {string} primitive_modal_id DOM id of the modal container 
 *      to show/hide.
 * @param {string} primitive_modal_content_id DOM id of the modal 
 *      content container where dynamic elements are injected.
 * @param {string} [save_add_id="save_add"] DOM id of the save 
 *      button used to confirm adding the primitive.
 * Source: Mostly ChatGPT
 */
export function open_add_primitive_editor( primitive_modal_id, primitive_modal_content_id,
    save_add_id="save_add") {

    const modal = document.getElementById(primitive_modal_id);
    const content = document.getElementById(primitive_modal_content_id);
    content.innerHTML = ""; // Clear

    // Primitive selector
    const select = document.createElement("select");
    select.className = "w-full border p-2 rounded mb-4";

    const flatList = [];
    for (const level of Object.values(PRIMITIVE_LIBRARY)) {
        for (const prim of level) {
        flatList.push(prim);
        const option = document.createElement("option");
        option.value = prim.name;
        option.textContent = prim.name;
        select.appendChild(option);
        }
    }

    content.appendChild(select);

    // Parameter container
    const params_container = document.createElement("div");
    content.appendChild(params_container);

    // Initial render
    render_params(flatList[0], params_container);

    select.addEventListener("change", () => {
        const prim = flatList.find(p => p.name === select.value);
        render_params(prim, params_container);
    });

    // Save button
    const save_button = document.getElementById(save_add_id);
    save_button.onclick = null;  // Clear previous listener
    save_button.onclick = async () => {
        const selected = flatList.find(p => p.name === select.value);

        const new_prim = {
            name: selected.name,
            parameters: {},
        };

        // TODO: SHARE THIS ALSO
        for (const [param, default_value] of Object.entries(selected.parameters)) {
            let value = document.getElementById(`add_${param}`).value;
            if (Array.isArray(default_value)) {
                value = value.split(",").map(Number);
            }
            new_prim.parameters[param] = value;
        }

        let prim_to_add = new_prim;

        // Expand mid-level primitives
        if (selected.parameters && Object.keys(selected.parameters).length > 0) {
            prim_to_add = await post_primitive(new_prim);
        }

        const { primitive_plan } = get_state();
        set_state({
            primitive_plan: [...primitive_plan, prim_to_add],
        });

        close_primitive_editor(primitive_modal_id);
    };

    modal.classList.remove("hidden");
}


