import {get_state, set_state} from "/src/js/state.js";
import {post_revised_plan, post_plan, get_plan} from "/src/js/helpers/api.js"
import {get_executing_primitive_idx} from "/src/js/helpers/api.js"
import expand_icon from "url:/src/assets/svgs/expand.svg";
import shrink_icon from "url:/src/assets/svgs/shrink.svg";

// TODO: Do this better????
let drag_state = {
    active: false,
    sourceIndex: null,
    targetIndex: null,
    ghost: null,
    holdTimer: null,
};

/**
 * Executes the currently loaded plan.
 * @param {boolean} on_real True if executed on the real robot, else false.
 * @throws {Error} If plan execution fails.
 */
export async function handle_plan_play(on_real) {
    let { primitive_plan, executing_index, id, task_prompt} = get_state();
    if (!primitive_plan || primitive_plan.length === 0) {
        alert("No plan to execute.");
        return;
    }

    try {
        // Save plan if edited
        const plan = await post_revised_plan(id, task_prompt, primitive_plan);

        set_state({id: plan.id, revision_of: plan.revision_of,
               primitive_plan: plan.primitive_plan, task_prompt: plan.task_prompt});

        // Start at paused index if exists
        if (executing_index && !on_real) {
            post_plan(plan.primitive_plan, on_real, executing_index);
        } else {
            post_plan(plan.primitive_plan, on_real);
        }
        
        check_execution_status();
    } catch (err) {
        console.error("Failed to execute plan:", err);
        // TODO: handle this cleaner
        alert(`Plan execution failed: ${err}`);
    }
}

/**
 * Periodically polls the backend for the currently executing primitive index
 * and keeps application state in sync.
 */
async function  check_execution_status() {
    let executing_idx = null;
    
    const interval_id = setInterval(async() => {

        // Don't set executing idx to null if paused.
        if (get_state().pause) {
            clearInterval(interval_id);
            return;
        }
        const idx = await get_executing_primitive_idx();
        const last_set_idx = get_state().executing_index;

        // Compare idx arrays 
        if (JSON.stringify(executing_idx) !== JSON.stringify(idx)) {
            executing_idx = idx;
            
            set_state({executing_index: executing_idx});
        }
        
        // Reach end of plan
        if (last_set_idx && !executing_idx) {
            set_state({pause: true})
            clearInterval(interval_id);
        }
    }, 300); // 0.3 seconds
}

/**
 * Loads the most recent primitive plan and updates application state.
 * @throws {Error} If fetching the latest plan fails.
 */
export async function load_latest_timeline() {
    try {
        const plan = await get_plan();

        set_state({id: plan.id, revision_of: plan.revision_of,
               primitive_plan: plan.primitive_plan, task_prompt: plan.task_prompt});
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
function build_prim_card(prim, index, is_sub_prim, is_expanded, is_executing) {
    // Parameters
    const params = prim.parameters;
    
    let bg = is_sub_prim ? 'bg-blue-300' : ' bg-neutral-300';

    // Add hover if clickable
    if (params && Object.keys(params).length > 0) {
        bg += is_sub_prim ? ' hover:bg-blue-400' : ' hover:bg-neutral-400';
    }
    if (is_executing) bg += ' outline-6 outline-yellow-500';

    const card = document.createElement("div");
    card.className = `min-w-36 min-h-30 p-2 m-2 ${bg}   rounded-xl  flex-shrink-0`;
    card.dataset.planIndex = JSON.stringify(index);  // Resolves to plan-index

    // TOP LINE
    const header = document.createElement("div");
    header.className = "flex justify-between";
    card.appendChild(header);

    // Title
    const title = document.createElement("h1");
    title.className = "font-medium text-xl";
    title.textContent = prim.name;
    header.appendChild(title);



    
    for (const [param_name, param_value] of Object.entries(params)) {
        
        let formatted_value = param_value;
        if (formatted_value.constructor == Array) {
            // Make arrays readable by only showing first 3 elements
            formatted_value = `[${param_value.slice(0, 3).join(", ")}]`; 
        }

        const p = document.createElement("p");
        p.textContent = `${param_name}: ${formatted_value}`;
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
            if (expanded.has(index[0])) expanded.delete(index[0]);
            else expanded.add(index[0]);
    
            set_state({ expanded: expanded });
            
        });

        header.appendChild(expand_button);
    }

    if (params && Object.keys(params).length > 0) {
        
        card.addEventListener("click", (e) => {
            e.stopPropagation();
            set_state({ editing_index: index });
        });
    }


    // Hold to drag (reorder)
    if (!is_sub_prim) {

        card.addEventListener("mousedown", (e) => {
            if (e.button !== 0) return;
            e.preventDefault(); 

            drag_state.holdTimer = setTimeout(() => {
                start_reorder_drag(e, index[0], card);
            }, 200); // hold delay
        });

        card.addEventListener("mouseup", () => {
            clearTimeout(drag_state.holdTimer);
        });

        card.addEventListener("mouseleave", () => {
            clearTimeout(drag_state.holdTimer);
        });
    }

    return card;

}

/**
 * Renders a timeline of primitive cards into the specified container.
 * @param {Array<Object>} primitives The primitive plan to display in
 *  the form of: [{'name': 'grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
 * @param {string} timeline_id The DOM element id of the timeline container.
 * Source: ChatGPT
 */
export function populate_timeline(primitives, timeline_id) {

    if (!primitives) return;

    const timeline = document.getElementById(timeline_id);
    timeline.innerHTML = ""; // Clear timeline


    primitives.forEach((prim, idx) => {
        const is_expanded = prim.core_primitives && get_state().expanded.has(idx);

        const executing_idx =  get_state().executing_index;
        const is_executing = executing_idx && executing_idx[0] == idx;

        const card = build_prim_card(prim, [idx], false, is_expanded, is_executing);
        timeline.appendChild(card);

        // Show child prims
        if (is_expanded) {
            const sub_div = document.createElement("div");
            sub_div.className = "flex ";
            sub_div.dataset.subPrimitives = "true";  // Equivalent to data-sub-primitives
            
            prim.core_primitives.forEach((core_prim, core_idx) => {
                const is_core_executing = is_executing && executing_idx[1] == core_idx;
                const sub_card = build_prim_card(core_prim, [idx, core_idx], true, false, is_core_executing);
                sub_div.appendChild(sub_card);    
            })

            card.appendChild(sub_div);
        }
    });
}



/**
 * 
 * @param {*} e 
 * @param {*} sourceIndex 
 * @param {*} card 
 * Source: Mostly ChatGPT
 */
function start_reorder_drag(e, sourceIndex, card) {

    drag_state.active = true;
    drag_state.sourceIndex = sourceIndex;

    // Ghost element
    const ghost = card.cloneNode(true);
    ghost.style.position = "absolute";
    ghost.style.pointerEvents = "none";
    ghost.style.opacity = "0.7";
    ghost.style.zIndex = "50";
    ghost.style.left = `${card.getBoundingClientRect().left}px`;
    ghost.style.top = `${card.getBoundingClientRect().top}px`;
    ghost.style.width = `${card.offsetWidth}px`;

    document.body.appendChild(ghost);
    drag_state.ghost = ghost;

    card.style.opacity = "0.3";

    document.addEventListener("mousemove", on_drag_move);
    document.addEventListener("mouseup", on_drag_end);
}

/**
 * 
 * @param {*} e 
 * @returns 
 * Source: Mostly ChatGPT
 * TODO
 */
function on_drag_move(e) {
    if (!drag_state.active) return;

    drag_state.ghost.style.left = `${e.clientX - drag_state.ghost.offsetWidth / 2}px`;

    const timeline = document.getElementById("timeline");
    const cards = [...timeline.children];

    drag_state.targetIndex = null;

    cards.forEach((c, idx) => {
        const rect = c.getBoundingClientRect();
        if (e.clientX > rect.left && e.clientX < rect.right) {
            drag_state.targetIndex = idx;
            c.style.transform = "translateY(-6px)";
        } else {
            c.style.transform = "";
        }
    });
}

/**
 * TODO
 * @returns TODO
 * Source: Mostly ChatGPT
 */
function on_drag_end() {
    document.removeEventListener("mousemove", on_drag_move);
    document.removeEventListener("mouseup", on_drag_end);

    const { sourceIndex, targetIndex } = drag_state;

    cleanup_drag_visuals();

    if (
        targetIndex === null ||
        sourceIndex === targetIndex
    ) return;

    const state = get_state();
    const newPlan = [...state.primitive_plan];
    const [moved] = newPlan.splice(sourceIndex, 1);
    newPlan.splice(targetIndex, 0, moved);

    // Reset execution index if affected
    let executing_index = state.executing_index;
    if (executing_index && executing_index[0] === sourceIndex) {
        executing_index = [targetIndex];
    }

    set_state({
        primitive_plan: newPlan,
        executing_index,
    });
}

/**
 * TODO
 */
function cleanup_drag_visuals() {
    drag_state.active = false;

    if (drag_state.ghost) {
        drag_state.ghost.remove();
    }

    document.querySelectorAll("#timeline > div").forEach(c => {
        c.style.opacity = "";
        c.style.transform = "";
    });

    drag_state = {
        active: false,
        sourceIndex: null,
        targetIndex: null,
        ghost: null,
        holdTimer: null,
    };
}

/**
 * Initializes draggable scrubber for a horizontal timeline.
 * @param {string} timeline_viewport_id  DOM ID of the scrollable timeline container.
 * @param {string} timeline_id           DOM ID of the timeline content container.
 * @param {string} scrubber_id           DOM ID of the scrubber overlay element.
 * Source: Mostly ChatGPT
 */
export function init_timeline_scrubber(timeline_viewport_id, timeline_id, scrubber_id) {
    const timeline = document.getElementById(timeline_id);
    const scrubber = document.getElementById(scrubber_id);

    let dragging = false;

    scrubber.addEventListener("mousedown", (e) => {
        dragging = true;
        e.preventDefault();
    });

    document.addEventListener("mousemove", (e) => {
        if (!dragging) return;

        set_state({pause: true});

        const rect = timeline.getBoundingClientRect();
        let x = e.clientX - rect.left;

        // Clamp
        x = Math.max(0, Math.min(x, rect.width));

        // Move scrubber
        scrubber.style.left = `${x}px`;

        // Scroll timeline accordingly
        const scrollRatio = x / rect.width;
        timeline.scrollLeft = scrollRatio * (timeline.scrollWidth - timeline.clientWidth);
    });


    document.addEventListener("mouseup", () => {
        if (!dragging) return;
        const index = snap_scrubber_to_card(timeline_viewport_id, timeline_id, scrubber_id)
        
        set_state({executing_index: index});

        dragging = false;

    });
}


/**
 * Snaps the scrubber to the closest timeline card.
 * @param {string} timeline_viewport_id  DOM ID of the scrollable timeline container.
 * @param {string} timeline_id           DOM ID of the timeline content container.
 * @param {string} scrubber_id           DOM ID of the scrubber overlay element.
 * @returns {number[]}   Hierarchical plan index of the snapped card,
 *                              or null if no valid card is found.
 * Source: Mostly ChatGPT
 */
function snap_scrubber_to_card(viewport_id, timeline_id, scrubber_id) {

    const viewport = document.getElementById(viewport_id); // scroll container
    const timeline = document.getElementById(timeline_id); // content container
    const scrubber = document.getElementById(scrubber_id); // overlay

    const cards = [...timeline.querySelectorAll('[data-plan-index]')];
    if (cards.length === 0) return;

    const scrubber_x = viewport.scrollLeft + scrubber.offsetLeft; // TODO: CHECK THIS

    let closest = null;
    let minDist = Infinity;

    // Find closest card
    for (const card of cards) {
        const center = card.offsetLeft + card.offsetWidth / 2;
        const dist = Math.abs(center - scrubber_x);
        if (dist < minDist) {
            minDist = dist;
            closest = card;
        }
    }

    if (!closest) return;

    // Snap to left of closest card
    scrubber.style.left = `${closest.offsetLeft}px`;

    const index = JSON.parse(closest.dataset.planIndex);
    return index;
}



/**
 * Moves the timeline scrubber to the card corresponding to the plan index if expanded.
 * @param {number[]} index              Hierarchical plan index (e.g. [0] or [1, 2]).
 * @param {string} viewport_id          ID of the scrollable timeline viewport.
 * @param {string} timeline_id          ID of the timeline content container.
 * @param {string} scrubber_id          ID of the scrubber overlay element.
 */
export function move_scrubber_to_index(index, viewport_id, timeline_id, scrubber_id) {
    if (!Array.isArray(index) || index.length === 0) return;

    const viewport = document.getElementById(viewport_id);
    const timeline = document.getElementById(timeline_id);
    const scrubber = document.getElementById(scrubber_id);

    const parent_idx = index[0];
    const parent_card = timeline.children[parent_idx];
    if (!parent_card) return;

    let target_card = parent_card;

    // TODO: HANDLE more levels
    if (index.length >= 2) {
        const sub_idx = index[1];

        // Only use children if parent is expanded
        const expanded = get_state().expanded;
        if (expanded.has(parent_idx)) {
            const sub_container = parent_card.querySelector('[data-sub-primitives]');
            if (!sub_container || !sub_container.children[sub_idx]) return;
            target_card = sub_container.children[sub_idx];
        }
    }

    // Position scrubber relative to viewport
    scrubber.style.left =`${target_card.offsetLeft}px`;
}
