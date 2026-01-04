import {set_state} from "/src/js/state.js";
import { post_task } from "/src/js/helpers/api";


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
          // Clear plan while loading
          set_state({plan: [], expanded: new Set(), editing_index: null});
          const primitives = await post_task(task);
          console.log("Received Plan:", primitives);
          set_state({plan: primitives});

     } catch (err) {
          console.error("Error calling primitive_plan API:", err);
          // TODO: Handle this prettily
          alert("Failed to generate primitive plan.");
     } finally {
          hide_loading();
     }
}