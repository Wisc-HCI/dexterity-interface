import { get_state, set_state} from "/src/js/state.js";


export function open_primitive_editor(prim) {

  // Fill fields
  document.getElementById("primitive_type").innerHTML = prim.type ?? "";

  // ARM field
  if ("arm" in prim) {
    document.getElementById("arm_field").classList.remove("hidden");
    document.getElementById("edit_arm").value = prim.arm ?? "";
  } else {
    document.getElementById("arm_field").classList.add("hidden");
  }

  // POSE field
  if ("pose" in prim) {
    document.getElementById("pose_field").classList.remove("hidden");
    document.getElementById("edit_pose").value = prim.pose.join(", ");
  } else {
    document.getElementById("pose_field").classList.add("hidden");
  }

  document.getElementById("primitive_modal").classList.remove("hidden");

}

export function save_primitive_edit() {
  
  const { plan, editing_index } = get_state();
  const prim = { ...plan[editing_index] };

  if ("arm" in prim) {
    const arm = document.getElementById("edit_arm").value;
    if (arm) prim.arm = arm;
  } 
  
  if ("pose" in prim) {
    const pose_str = document.getElementById("edit_pose").value;
    prim.pose = pose_str.split(",").map(Number);
  }

  const updated_plan = [...plan];
  updated_plan[editing_index] = prim;
  set_state({plan: updated_plan});

  

  close_primitive_editor("primitive_modal");
}


/* TODO */
function close_primitive_editor(modal_id) {
  document.getElementById(modal_id).classList.add("hidden");
  set_state({editing_index: null,});
    
}
