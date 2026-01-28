import {set_state, get_state} from "/src/js/state.js";
import { post_task, get_all_plans } from "/src/js/helpers/api";


/**
 * Computes the depth of a plan in the revision tree.
 *
 * @param {Object} plan The plan whose depth is being calculated in the form of:
 *     {
 *        id: string,
 *        revision_of: string | null,
 *        task_prompt: string,
 *        primitive_plan: [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
 *      }
 * @param {Object<string, Object>} plans_by_id Lookup table mapping plan IDs
 *        to plan objects.
 * @returns {number} The depth of the plan within the revision hierarchy.
 * Source: Mostly ChatGPT
*/
function get_depth(plan, plans_by_id) {
    let depth = 0;
    let cur = plan;
    while (cur.revision_of) {
        cur = plans_by_id[cur.revision_of];
        if (!cur) break;
        depth++;
    }
    return depth;
}


/**
 * Populate the task history panel with all existing plans. 
 * Each task is clickable and restores its plan into state.
 * @param {string} task_history_id The DOM element id of the task history div.
 */
export async function populate_task_history(task_history_id, task_history_label_id="task_history_label", task_input_div_id="task_input_div") {
    const container = document.getElementById(task_history_id);
    const scroll_position = container.scrollTop;
    container.innerHTML = ""; // Clear existing history

    try {
        const plans = await get_all_plans();
        
        // UI
        const history_label = document.getElementById(task_history_label_id);
        const input_div = document.getElementById(task_input_div_id);
        if (!plans.length) {
            history_label.classList.add("hidden");
            input_div.classList.remove("mt-auto");

            return;
        } 
        history_label.classList.remove("hidden");
        input_div.classList.add("mt-auto");

        const cur_id = get_state().id;

        const plans_by_id = Object.fromEntries(plans.map(p => [p.id, p]));

        plans.forEach(plan => {
            const depth = get_depth(plan, plans_by_id);

            const wrapper = document.createElement("div");
            if (depth !== 0)  wrapper.className = "tree-node";
            wrapper.style.marginLeft = `${depth * 12}px`;

            const item = document.createElement("div");
            item.className =
                "p-2 mb-2 rounded cursor-pointer bg-neutral-200 hover:bg-neutral-400 text-sm relative";

            if (plan.id === cur_id) {
                item.className += " border border-2 border-yellow-500";
            }

            item.innerHTML = `
                <div class="font-medium">${plan.task_prompt}</div>
                <div class="text-xs text-neutral-600">${plan.primitive_plan.length} primitives</div>
            `;

            wrapper.appendChild(item);
            container.appendChild(wrapper);

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
        });

        container.scrollTop = scroll_position;

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