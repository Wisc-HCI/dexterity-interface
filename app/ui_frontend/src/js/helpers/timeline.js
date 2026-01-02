import {get_state, set_state} from "/src/js/state.js";
import {post_plan, get_plan} from "/src/js/helpers/api.js"
import expand_icon from "url:/src/assets/svgs/expand.svg";


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
 *  the form of: [{'name': 'grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
 * @param {string} timeline_id The DOM element id of the timeline container.
 * Source: GPT
 */
export function populate_timeline(primitives, timeline_id) {
  const timeline = document.getElementById(timeline_id);
  timeline.innerHTML = "";

  primitives.forEach((prim, index) => {
        const card = document.createElement("div");
        card.className =
        "w-48 p-2 bg-neutral-300 border rounded-xl text-center flex-shrink-0";

        // Title
        const title = document.createElement("h1");
        title.className = "font-medium text-xl";
        title.textContent = prim.name;
        card.appendChild(title);

        // Parameters
        const params = prim.parameters;

        if (params.arm) {
            const p = document.createElement("p");
            p.textContent = `Arm: ${params.arm}`;
            card.appendChild(p);
        }

        if (params.pose) {
            const xyz = params.pose.slice(0, 3);
            const p = document.createElement("p");
            p.textContent = `Pose: [${xyz.join(", ")}]`;
            card.appendChild(p);
        }

        // Expand button
        if (prim.core_primitives) {
            const expand_button = document.createElement("button");
            expand_button.className = "p-1 hover:bg-neutral-400 rounded";

            const img = document.createElement("img");
            img.src = expand_icon;
            img.alt = "Expand Prim Button";
            img.className = "h-6 mx-auto";

            expand_button.appendChild(img);

            expand_button.addEventListener("click", (e) => {
                e.stopPropagation();
                // TODO
            });

            card.appendChild(expand_button);
        }

        card.addEventListener("click", () => {
            set_state({ editing_index: index });
        });
        timeline.appendChild(card);
    });
}