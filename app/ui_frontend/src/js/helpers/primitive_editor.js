import { get_state, set_state } from "/src/js/state.js";
import {
  post_primitive,
  post_ui_marker_spawn,
  post_ui_marker_move,
} from "/src/js/helpers/api.js";
import {
  format_number,
  decimals_from_step,
  quatToEulerDeg,
  eulerDegToQuat,
} from "/src/js/helpers/math_utils.js";

/**
 * Parameter units (from libs/planning/.../primitives.yaml).
 * Note: we keep backend storage as quaternion arrays, but display Euler in UI.
 */
const PARAM_UNITS = {
  arm: "str ('left'|'right')",
  pose: "", //"[x,y,z,qx,qy,qz,qw] (m, quat)",
  grasp_pose: "", //"[x,y,z,qx,qy,qz,qw] (m, quat)",
  initial_pose: "", //"[x,y,z,qx,qy,qz,qw] (m, quat)",
  end_position: "", //"[x,y,z] (m)",
  pour_orientation: "", //"[qx,qy,qz,qw] (quat)",
  pour_hold: "float (seconds)",
};

const POSE_7D_PARAMS = new Set(["pose", "grasp_pose", "initial_pose"]);
const ORIENTATION_ONLY_PARAMS = new Set(["pour_orientation"]);
const POSITION_ONLY_PARAMS = new Set(["end_position"]);
const POSE_MARKER_SUFFIXES = ["x", "y", "z", "roll", "pitch", "yaw"];
let is_saving_primitive_edit = false;
let current_primitive_modal_id = null;

/**
 * Normalize `editing_index` so downstream logic can always use array indexing.
 *
 * Args:
 *   editing_index (number|Array<number>|null): Current edit index from state.
 *
 * Returns:
 *   (Array<number>|null): Normalized edit index, or null when invalid.
 */
function normalize_editing_index(editing_index) {
  if (Array.isArray(editing_index) && editing_index.length > 0) {
    return editing_index;
  }
  if (Number.isInteger(editing_index)) {
    return [editing_index];
  }
  return null;
}

/**
 * Create a unit text span displayed next to parameter labels.
 *
 * Args:
 *   unit_text (string): Unit descriptor shown in the UI.
 *
 * Returns:
 *   (HTMLSpanElement): Unit span element.
 */
function make_unit_span(unit_text) {
  const unit = document.createElement("span");
  unit.className = "text-xs text-gray-500 ml-2";
  unit.textContent = unit_text ?? "";
  return unit;
}

/**
 * Create a number input with formatting derived from the step size.
 *
 * Args:
 *   id (string): Input DOM id.
 *   value (number): Initial numeric value.
 *   step (string): HTML number input step.
 *
 * Returns:
 *   (HTMLInputElement): Number input element.
 */
function make_number_input(id, value, step = "0.001") {
  const input = document.createElement("input");
  input.type = "number";
  input.id = id;
  input.step = step;
  input.className = "w-full border p-2 rounded";
  const decimals = decimals_from_step(step);
  input.value = format_number(value, decimals);

  return input;
}

/**
 * Create a labeled number input (label above the input).
 *
 * Args:
 *   label_text (string): e.g. "x", "roll"
 *   input_id (string): DOM id
 *   value (number): default value
 *   step (string): input step
 * Returns:
 *   (HTMLDivElement): container
 */
function make_labeled_number_input(label_text, input_id, value, step) {
  const wrapper = document.createElement("div");
  wrapper.className = "flex flex-col";

  const label = document.createElement("div");
  label.className = "text-xs text-gray-500 mb-1";
  label.textContent = label_text;

  wrapper.appendChild(label);
  wrapper.appendChild(make_number_input(input_id, value, step));
  return wrapper;
}

/**
 * Opens the primitive editor modal and populates fields from a primitive.
 * @param {int| array[int]} editing_index Index of the prim in state plan to edit. If its core/sub primitive, the
 *      index is an array where each index corresponds to each level of the prim hierarchy.
 * @param {str} primitive_model_id DOM ID of model to show/hide
 * @param {str} primitive_modal_content_id DOM ID of model div to add elements too
 */
export async function open_primitive_editor(
  editing_index,
  primitive_modal_id,
  primitive_modal_content_id
) {
  const normalized_editing_index = normalize_editing_index(editing_index);
  if (normalized_editing_index == null) {
    return;
  }

  let prim;
  const plan = get_state().primitive_plan;

  if (normalized_editing_index.length >= 2) {
    // TODO: Handle multiple levels of prims
    prim = plan[normalized_editing_index[0]].core_primitives[normalized_editing_index[1]];
  } else {
    prim = plan[normalized_editing_index[0]];
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
    const label = document.createElement("label");
    label.id = `${param_name}_field`;
    label.className = "block mb-3";

    const header_row = document.createElement("div");
    header_row.className = "flex items-center mb-1";

    const span = document.createElement("span");
    span.className = "text-sm font-medium";
    span.textContent = `${param_name}`;

    header_row.appendChild(span);
    header_row.appendChild(make_unit_span(PARAM_UNITS[param_name]));
    label.appendChild(header_row);

    // Pose-like params: show position + Euler (deg)
    if (
      POSE_7D_PARAMS.has(param_name) &&
      Array.isArray(param_value) &&
      param_value.length === 7
    ) {
      const pos = param_value.slice(0, 3);
      const quat = param_value.slice(3, 7);
      const euler = quatToEulerDeg(quat);

      const grid = document.createElement("div");
      grid.className = "grid grid-cols-3 gap-2";

      grid.appendChild(make_labeled_number_input("x (m)", `${param_name}_x`, pos[0], "0.001"));
      grid.appendChild(make_labeled_number_input("y (m)", `${param_name}_y`, pos[1], "0.001"));
      grid.appendChild(make_labeled_number_input("z (m)", `${param_name}_z`, pos[2], "0.001"));

      grid.appendChild(make_labeled_number_input("roll (deg)", `${param_name}_roll`, euler[0], "1"));
      grid.appendChild(make_labeled_number_input("pitch (deg)", `${param_name}_pitch`, euler[1], "1"));
      grid.appendChild(make_labeled_number_input("yaw (deg)", `${param_name}_yaw`, euler[2], "1"));


      label.appendChild(grid);
      model_content.appendChild(label);

      // Spawn marker at current pose (ignore if sim not running)
      try {
        await post_ui_marker_spawn({ pose: param_value });
      } catch (e) {
        console.warn("ui_marker_spawn failed:", e);
      }

            // Live update marker while editing
      let marker_move_timer = null;

      const base = param_name;

      const move_marker_from_inputs = async () => {
        const x = Number(document.getElementById(`${base}_x`).value);
        const y = Number(document.getElementById(`${base}_y`).value);
        const z = Number(document.getElementById(`${base}_z`).value);
        const r = Number(document.getElementById(`${base}_roll`).value);
        const p = Number(document.getElementById(`${base}_pitch`).value);
        const yw = Number(document.getElementById(`${base}_yaw`).value);

        const q = eulerDegToQuat([r, p, yw]);
        const pose = [x, y, z, q[0], q[1], q[2], q[3]];

        try {
          await post_ui_marker_move({ pose });
        } catch (e) {
          console.warn("ui_marker_move failed:", e);
        }
      };

      const schedule_marker_move = () => {
        if (marker_move_timer) clearTimeout(marker_move_timer);
        marker_move_timer = setTimeout(() => {
          move_marker_from_inputs();
        }, 50);
      };

      POSE_MARKER_SUFFIXES.forEach((suffix) => {
        const el = document.getElementById(`${base}_${suffix}`);
        if (!el) return;
        el.oninput = schedule_marker_move;
        el.onchange = move_marker_from_inputs;
      });

      continue;
    }

    // Orientation-only quaternion: show Euler (deg)
    if (
      ORIENTATION_ONLY_PARAMS.has(param_name) &&
      Array.isArray(param_value) &&
      param_value.length === 4
    ) {
      const euler = quatToEulerDeg(param_value);

      const grid = document.createElement("div");
      grid.className = "grid grid-cols-3 gap-2";

      grid.appendChild(
        make_labeled_number_input("roll (deg)", `${param_name}_roll`, euler[0], "1")
      );
      grid.appendChild(
        make_labeled_number_input("pitch (deg)", `${param_name}_pitch`, euler[1], "1")
      );
      grid.appendChild(
        make_labeled_number_input("yaw (deg)", `${param_name}_yaw`, euler[2], "1")
      );

      label.appendChild(grid);
      model_content.appendChild(label);
      continue;
    }

    // Position-only: show x/y/z with labels
    if (
      POSITION_ONLY_PARAMS.has(param_name) &&
      Array.isArray(param_value) &&
      param_value.length === 3
    ) {
      const grid = document.createElement("div");
      grid.className = "grid grid-cols-3 gap-2";
      grid.appendChild(
        make_labeled_number_input("x (m)", `${param_name}_x`, param_value[0], "0.001")
      );
      grid.appendChild(
        make_labeled_number_input("y (m)", `${param_name}_y`, param_value[1], "0.001")
      );
      grid.appendChild(
        make_labeled_number_input("z (m)", `${param_name}_z`, param_value[2], "0.001")
      );

      label.appendChild(grid);
      model_content.appendChild(label);
      continue;
    }

    // Default param rendering
    let input;
    if (param_name == "arm") {
      input = document.createElement("select");
      const options = ["left", "right"];
      options.forEach((value) => {
        const option = document.createElement("option");
        option.value = value;
        option.textContent = value;
        input.appendChild(option);
      });
      input.value = String(param_value);
    } else {
      input = document.createElement("input");
      input.value = Array.isArray(param_value)
        ? param_value.join(", ")
        : String(param_value);
    }

    input.id = `${param_name}_field_input`;
    input.className = "w-full border p-2 rounded";

    label.appendChild(input);
    model_content.appendChild(label);
  }

  current_primitive_modal_id = primitive_modal_id;
  if (model) model.classList.remove("hidden");
}

/**
 * Saves edits made to the currently selected primitive to state.
 */
export async function save_primitive_edit() {
  if (is_saving_primitive_edit) return;

  // Snapshot state ONCE, so it doesn't change mid-save
  const state = get_state();
  const normalized_editing_index = normalize_editing_index(state.editing_index);
  if (normalized_editing_index == null) return;

  const save_button = document.getElementById("save_edit");
  is_saving_primitive_edit = true;
  if (save_button) save_button.disabled = true;

  try {
    // Use the snapshotted primitive_plan (NOT get_state() again later)
    const primitive_plan = state.primitive_plan;

    let prim;
    if (normalized_editing_index.length >= 2) {
      prim = {
        ...primitive_plan[normalized_editing_index[0]].core_primitives[
          normalized_editing_index[1]
        ],
      };
    } else {
      prim = { ...primitive_plan[normalized_editing_index[0]] };
    }

    for (const [param_name, param_value] of Object.entries(prim.parameters)) {
      if (POSE_7D_PARAMS.has(param_name)) {
        const x = Number(document.getElementById(`${param_name}_x`)?.value);
        const y = Number(document.getElementById(`${param_name}_y`)?.value);
        const z = Number(document.getElementById(`${param_name}_z`)?.value);
        const r = Number(document.getElementById(`${param_name}_roll`)?.value);
        const p = Number(document.getElementById(`${param_name}_pitch`)?.value);
        const yw = Number(document.getElementById(`${param_name}_yaw`)?.value);

        const q = eulerDegToQuat([r, p, yw]);
        prim.parameters[param_name] = [x, y, z, q[0], q[1], q[2], q[3]];
        continue;
      }

      if (ORIENTATION_ONLY_PARAMS.has(param_name)) {
        const r = Number(document.getElementById(`${param_name}_roll`)?.value);
        const p = Number(document.getElementById(`${param_name}_pitch`)?.value);
        const yw = Number(document.getElementById(`${param_name}_yaw`)?.value);
        prim.parameters[param_name] = eulerDegToQuat([r, p, yw]);
        continue;
      }

      if (POSITION_ONLY_PARAMS.has(param_name)) {
        const x = Number(document.getElementById(`${param_name}_x`)?.value);
        const y = Number(document.getElementById(`${param_name}_y`)?.value);
        const z = Number(document.getElementById(`${param_name}_z`)?.value);
        prim.parameters[param_name] = [x, y, z];
        continue;
      }

      const el = document.getElementById(`${param_name}_field_input`);
      if (!el) continue;

      let input = el.value;
      if (Array.isArray(param_value)) input = input.split(",").map(Number);
      prim.parameters[param_name] = input;
    }

    if (prim.core_primitives) {
      prim = await post_primitive(prim);
    }

    const updated_plan = structuredClone(primitive_plan);
    if (normalized_editing_index.length >= 2) {
      updated_plan[normalized_editing_index[0]].core_primitives[
        normalized_editing_index[1]
      ] = prim;
    } else {
      updated_plan[normalized_editing_index[0]] = prim;
    }

    set_state({ primitive_plan: updated_plan });

    // Close last
    close_primitive_editor(current_primitive_modal_id ?? "primitive_modal");
  } catch (e) {
    console.error("save_primitive_edit failed:", e);
  } finally {
    if (save_button) save_button.disabled = false;
    is_saving_primitive_edit = false;
  }
}

/**
 * Closes the primitive editor modal and clears editing state.
 * @param {string} modal_id The DOM element id of the modal to close.
 */
export function close_primitive_editor(modal_id) {
  document.getElementById(modal_id).classList.add("hidden");
  set_state({ editing_index: null });
}

/**
 * Deletes the primitive that is currently being edited
 * @param {string} modal_id The DOM element id of the modal to close
 *  after deleting
 */
export function delete_primitive(modal_id) {
  const { primitive_plan, editing_index } = get_state();
  const normalized_editing_index = normalize_editing_index(editing_index);

  if (normalized_editing_index == null) return;

  let plan = structuredClone(primitive_plan);
  if (normalized_editing_index.length >= 2) {
    // TODO: Handle multiple levels of prims
    plan[normalized_editing_index[0]].core_primitives.splice(normalized_editing_index[1], 1);
  } else {
    plan.splice(normalized_editing_index[0], 1);
  }

  close_primitive_editor(modal_id);
  set_state({ primitive_plan: plan });
}

///////////////////////////////////////////
//            Add primitive
//////////////////////////////////////////

// TODO: REPLACE with reading from file
const PRIMITIVE_LIBRARY = {
  low_level: [
    { name: "home", parameters: {} },
    { name: "move_to_pose", parameters: { arm: "left", pose: [0, 0, 0, 0, 0, 0, 1] } },
    { name: "envelop_grasp", parameters: { arm: "left" } },
    { name: "release", parameters: { arm: "left" } },
  ],
  mid_level: [
    {
      name: "pick",
      parameters: {
        arm: "left",
        grasp_pose: [0, 0, 0, 0, 0, 0, 1],
        end_position: [0, 0, 0],
      },
    },
    {
      name: "pour",
      parameters: {
        arm: "left",
        initial_pose: [0, 0, 0, 0, 0, 0, 1],
        pour_orientation: [0, 0, 0, 1],
        pour_hold: 1.0,
      },
    },
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
    const label = document.createElement("label");
    label.className = "block mb-3";

    const header_row = document.createElement("div");
    header_row.className = "flex items-center mb-1";

    const span = document.createElement("span");
    span.className = "text-sm font-medium";
    span.textContent = param;

    header_row.appendChild(span);
    header_row.appendChild(make_unit_span(PARAM_UNITS[param]));
    label.appendChild(header_row);

    // Pose-like
    if (POSE_7D_PARAMS.has(param) && Array.isArray(default_value) && default_value.length === 7) {
      const pos = default_value.slice(0, 3);
      const quat = default_value.slice(3, 7);
      const euler = quatToEulerDeg(quat);

      const grid = document.createElement("div");
      grid.className = "grid grid-cols-3 gap-2";

      grid.appendChild(make_labeled_number_input("x (m)", `add_${param}_x`, pos[0], "0.001"));
      grid.appendChild(make_labeled_number_input("y (m)", `add_${param}_y`, pos[1], "0.001"));
      grid.appendChild(make_labeled_number_input("z (m)", `add_${param}_z`, pos[2], "0.001"));

      grid.appendChild(make_labeled_number_input("roll (deg)", `add_${param}_roll`, euler[0], "1"));
      grid.appendChild(make_labeled_number_input("pitch (deg)", `add_${param}_pitch`, euler[1], "1"));
      grid.appendChild(make_labeled_number_input("yaw (deg)", `add_${param}_yaw`, euler[2], "1"));

      label.appendChild(grid);
      params_container.appendChild(label);
      continue;
    }

    // Orientation-only
    if (ORIENTATION_ONLY_PARAMS.has(param) && Array.isArray(default_value) && default_value.length === 4) {
      const euler = quatToEulerDeg(default_value);

      const grid = document.createElement("div");
      grid.className = "grid grid-cols-3 gap-2";

      grid.appendChild(
        make_labeled_number_input("roll (deg)", `add_${param}_roll`, euler[0], "1")
      );
      grid.appendChild(
        make_labeled_number_input("pitch (deg)", `add_${param}_pitch`, euler[1], "1")
      );
      grid.appendChild(
        make_labeled_number_input("yaw (deg)", `add_${param}_yaw`, euler[2], "1")
      );

      label.appendChild(grid);
      params_container.appendChild(label);
      continue;
    }

    // Position-only
    if (
      POSITION_ONLY_PARAMS.has(param) &&
      Array.isArray(default_value) &&
      default_value.length === 3
    ) {
      const grid = document.createElement("div");
      grid.className = "grid grid-cols-3 gap-2";

      grid.appendChild(
        make_labeled_number_input("x (m)", `add_${param}_x`, default_value[0], "0.001")
      );
      grid.appendChild(
        make_labeled_number_input("y (m)", `add_${param}_y`, default_value[1], "0.001")
      );
      grid.appendChild(
        make_labeled_number_input("z (m)", `add_${param}_z`, default_value[2], "0.001")
      );

      label.appendChild(grid);
      params_container.appendChild(label);
      continue;
    }

    // Default
    let input;
    if (param === "arm") {
      input = document.createElement("select");
      ["left", "right"].forEach((v) => {
        const options = document.createElement("option");
        options.value = v;
        options.textContent = v;
        input.appendChild(options);
      });
      input.value = String(default_value);
    } else {
      input = document.createElement("input");
      input.value = Array.isArray(default_value)
        ? default_value.join(",")
        : default_value;
    }

    input.id = `add_${param}`;
    input.className = "w-full border p-2 rounded";

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
export function open_add_primitive_editor(
  primitive_modal_id,
  primitive_modal_content_id,
  save_add_id = "save_add"
) {
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
    const prim = flatList.find((p) => p.name === select.value);
    render_params(prim, params_container);
  });

  // Save button
  const save_button = document.getElementById(save_add_id);
  save_button.onclick = null; // Clear previous listener
  save_button.onclick = async () => {
    const selected = flatList.find((p) => p.name === select.value);

    const new_prim = {
      name: selected.name,
      parameters: {},
    };

    for (const [param, default_value] of Object.entries(selected.parameters)) {
      // Pose-like
      if (POSE_7D_PARAMS.has(param)) {
        const x = Number(document.getElementById(`add_${param}_x`).value);
        const y = Number(document.getElementById(`add_${param}_y`).value);
        const z = Number(document.getElementById(`add_${param}_z`).value);
        const r = Number(document.getElementById(`add_${param}_roll`).value);
        const p = Number(document.getElementById(`add_${param}_pitch`).value);
        const yw = Number(document.getElementById(`add_${param}_yaw`).value);
        const q = eulerDegToQuat([r, p, yw]);
        new_prim.parameters[param] = [x, y, z, q[0], q[1], q[2], q[3]];
        continue;
      }

      // Orientation-only
      if (ORIENTATION_ONLY_PARAMS.has(param)) {
        const r = Number(document.getElementById(`add_${param}_roll`).value);
        const p = Number(document.getElementById(`add_${param}_pitch`).value);
        const yw = Number(document.getElementById(`add_${param}_yaw`).value);
        new_prim.parameters[param] = eulerDegToQuat([r, p, yw]);
        continue;
      }

      // Position-only
      if (POSITION_ONLY_PARAMS.has(param)) {
        const x = Number(document.getElementById(`add_${param}_x`).value);
        const y = Number(document.getElementById(`add_${param}_y`).value);
        const z = Number(document.getElementById(`add_${param}_z`).value);
        new_prim.parameters[param] = [x, y, z];
        continue;
      }

      // Default
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
