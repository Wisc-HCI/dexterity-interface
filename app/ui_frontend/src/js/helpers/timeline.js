import {get_state, set_state} from "/src/js/state.js";
import {post_plan, get_plan} from "/src/js/helpers/api.js"


/**
 * Executes the currently loaded plan.
 * @param {boolean} on_real True if executed on the real robot, else false.
 * @throws {Error} If plan execution fails.
 */
export async function handle_plan_play(on_real) {

    const { plan } = get_state();
    if (!plan || plan.length === 0) {
        alert("No plan to execute.");
        return;
    }

    try {
        post_plan(plan, on_real);
    } catch (err) {
        console.error("Failed to execute plan:", err);
        // TODO: handle this cleaner
        alert(`Plan execution failed: ${err}`);
    }
}

/**
 * Loads the most recent primitive plan and updates application state.
 * @throws {Error} If fetching the latest plan fails.
 */
export async function load_latest_timeline() {
  try {
    const latest_plan = await get_plan();
    set_state({plan: latest_plan});
  } catch (err) {
    console.error("Failed to load latest primitives:", err);
  }

}

/**
 * Renders a timeline of primitive cards into the specified container.
 * @param {Array<Object>} primitives The primitive plan to display in
 *  the form of: [{'type': 'grasp', 'arm': 'left', pose: [0,0,0,0,0,0,1]}, ...]
 * @param {string} timeline_id The DOM element id of the timeline container.
 * Source: GPT
 */
export function populate_timeline(primitives, timeline_id) {

    const timeline = document.getElementById(timeline_id);
    timeline.innerHTML = ""; // Clear existing

    primitives.forEach((prim, index) => {
        const card = document.createElement("div");
        card.className = "w-48 p-2 bg-neutral-300 border rounded-xl text-center flex-shrink-0";

        let content = `<h1 class="font-medium text-xl">${prim.type}</h1>`;

        // Conditionally add fields
        if (prim.arm) {
        content += `<p>Arm: <span>${prim.arm}</span></p>`;
        }

        if (prim.pose) {
        const xyz = prim.pose.slice(0, 3);
        content += `<p>Pose: <span>[${xyz.join(", ")}]</span></p>`;
        }

        card.innerHTML = content;
        
        card.addEventListener("click", () => {
        set_state({ editing_index: index });
        });


        timeline.appendChild(card);
    });
}