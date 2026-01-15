import {set_state, get_state} from "/src/js/state.js";
import { post_task, get_all_plans } from "/src/js/helpers/api";
 


/**
 * Populate the task history panel with all existing plans.
 * Each task is clickable and restores its plan into state.
 * @param {string} task_history_id The DOM element id of the task history div.
 */
export async function populate_task_history(task_history_id) {
    const container = document.getElementById(task_history_id);
    container.innerHTML = ""; // Clear existing history

    try {
     const plans = await get_all_plans();

     if (!plans.length) {
          return;
     }

     const cur_id = get_state().id;

     // Show oldest first
     plans.forEach(plan => {
          const item = document.createElement("div");
          

          item.className =
               "p-2 mt-1 mb-3 rounded cursor-pointer bg-neutral-200 hover:bg-neutral-400 text-sm";
          
          if (plan.id == cur_id) {
               item.className += ' outline-6 outline-yellow-500';
          }

          item.innerHTML = `
               <div class="font-medium truncate">
                    ${plan.task_prompt}
               </div>
               <div class="text-xs text-neutral-600">
                    ${plan.primitive_plan.length} primitives
               </div>
          `;

          item.onclick = () => {
               set_state({
                    id: plan.id,
                    revision_of: plan.revision_of,
                    task_prompt: plan.task_prompt,
                    primitive_plan: plan.primitive_plan,
                    expanded: new Set(),
                    editing_index: null,
               });
          };

          container.appendChild(item);
     });

    } catch (err) {
        console.error("Failed to load task history:", err);
    }
}



/**
 * Displays the loading spinner and disables the submit button.
 * Prevents user interaction while a task is being processed.
 */
function show_loading() {
     const spinner = document.getElementById("loading_spinner");
     const btn = document.getElementById("submit_task");

     spinner.classList.remove("hidden");
     btn.disabled = true;
     btn.classList.add("opacity-50", "cursor-not-allowed");
}

/**
 * Hides the loading spinner and re-enables the submit button.
 * Restores user interaction after task processing completes.
 */
function hide_loading() {
     const spinner = document.getElementById("loading_spinner");
     const btn = document.getElementById("submit_task");

     spinner.classList.add("hidden");
     btn.disabled = false;
     btn.classList.remove("opacity-50", "cursor-not-allowed");
}

/**
 * Handles submission of a task prompt and requests a primitive plan.
 * @param {string} text_id The DOM element id of the task input textarea.
 * @throws {Error} If the primitive plan request fails.
 * Source: Mostly ChatGPT.
 */
export async function handle_task_submit(text_id) {
    const textarea = document.getElementById(text_id);
    const task = textarea.value.trim();

    // TODO: Revise this
    if (!task) {
        alert("Please enter a task.");
        return;
    }

     show_loading();
     try {
          
          const id = get_state().id;
          console.log("ID:", id)
          // Clear plan while loading
          set_state({ primitive_plan: [], expanded: new Set(), 
               editing_index: null,
          });
          const plan = await post_task(task, id);
          console.log("Received Plan:", plan['primitive_plan']);

          set_state({id: plan.id, revision_of: plan.revision_of,
               primitive_plan: plan.primitive_plan, task_prompt: plan.task_prompt});

     } catch (err) {
          console.error("Error calling primitive_plan API:", err);
          // TODO: Handle this prettily
          alert("Failed to generate primitive plan.");
     } finally {
          hide_loading();
     }
}