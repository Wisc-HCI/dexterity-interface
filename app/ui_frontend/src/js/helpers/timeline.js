import {get_state, set_state} from "/src/js/state.js";
import {post_plan, get_plan} from "/src/js/helpers/api.js"
import expand_icon from "url:/src/assets/svgs/expand.svg";
import shrink_icon from "url:/src/assets/svgs/shrink.svg";



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
 * Creates a primitive card object
 * @param {Array<Object>} prim The primitive to build a card for in
 *  the form of: {'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]} }
 * @param {int| array[int]} index Index of the prim in state. If its core/sub primitive, the
 *      index is an array where each index corresponds to each level of the prim hierarchy.
 * @param {bool} is_sub_prim True if is a child of another prim.
 * @param {bool} is_expanded True if a parent prim and the children are expanded.
 * @returns {div} DOM card.
 */
function build_prim_card(prim, index, is_sub_prim, is_expanded) {

    const bg = is_sub_prim ? 'bg-blue-300 hover:bg-blue-400' :' bg-neutral-300 hover:bg-neutral-400';
    const card = document.createElement("div");
    card.className = `min-w-36 min-h-24 p-2 m-2 ${bg}   rounded-xl  flex-shrink-0`;

    // TOP LINE
    const header = document.createElement("div");
    header.className = "flex justify-between";
    card.appendChild(header);

    // Title
    const title = document.createElement("h1");
    title.className = "font-medium text-xl";
    title.textContent = prim.name;
    header.appendChild(title);

    

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
        expand_button.className = "p-1 hover:bg-neutral-500 rounded";

        const img = document.createElement("img");
        img.src = is_expanded ? shrink_icon : expand_icon;
    
        img.alt = "Expand Prim Button";
        img.className = "h-6";

        expand_button.appendChild(img);

        expand_button.addEventListener("click", (e) => {
            e.stopPropagation();

            const expanded = new Set(get_state().expanded); // Must make copy to change
            
            // Toggle expand or condense
            if (expanded.has(index)) expanded.delete(index);
            else expanded.add(index);
    
            set_state({ expanded: expanded });
            
        });

        header.appendChild(expand_button);
    }

    card.addEventListener("click", () => {
        set_state({ editing_index: index });
    });

    return card;

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
    timeline.innerHTML = ""; // Clear timeline


    primitives.forEach((prim, idx) => {
        const is_expanded = prim.core_primitives && get_state().expanded.has(idx);

        const card = build_prim_card(prim, idx, false, is_expanded);
        timeline.appendChild(card);

        // Show child prims
        if (is_expanded) {
            const sub_div = document.createElement("div");
            sub_div.className = "flex ";
            
            prim.core_primitives.forEach((core_prim, core_idx) => {
                const sub_card = build_prim_card(core_prim, [idx, core_idx], true, false);
                sub_div.appendChild(sub_card);    
            })

            card.appendChild(sub_div);
        }
    });
}