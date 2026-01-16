import {ROOT_URL} from "/src/js/constants.js"


/**
 * Sends a request to spawn the objects in simulation.
 * @throws {Error} If the API request fails.
 */
export async function post_objects() {
    const URL = `${ROOT_URL}/api/spawn_objects`;

    const response = await fetch(URL, {
            method: "POST",
            headers: { "Content-Type": "application/json"},
    });
    if (!response.ok) throw new Error("post_objects API request failed.");
}


/**
 * Submits a high-level task and returns the generated primitive plan.
 * @param {String} task_prompt The task description to be planned.
 * @param {string} revision_of [Optional] identifier of a prior plan.
 * @returns {Promise<Object>} The stored plan object in the form:
 *      {
 *        id: string,
 *        revision_of: string | null,
 *        task_prompt: string,
 *        primitive_plan: [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
 *      }
 *      
 * @throws {Error} If the API request fails.
 */
export async function post_task(task_prompt, revision_of) {
    const URL = `${ROOT_URL}/api/primitive_plan`;

    const response = await fetch(URL, {
        method: "POST",
        headers: { "Content-Type": "application/json"},
        body: JSON.stringify({
            'task_prompt': task_prompt,
            'revision_of': revision_of 
            }),
    });

    if (!response.ok) throw new Error("post_task API request failed");

    const plan = await response.json();
    return plan;
}


/**
 * Submits a primitive plan to be saved
 *  @param {string} revision_of Identifier of a prior plan.
 * @param {String} task_prompt The task description to be planned.
 * @param {<Array<Object>>} primitive_plan Primitive plan in form of:
 *      [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
 * @returns {Promise<Object>} The stored plan object in the form:
 *      {
 *        id: string,
 *        revision_of: string | null,
 *        task_prompt: string,
 *        primitive_plan: [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
 *      }
 *      
 * @throws {Error} If the API request fails.
 */
export async function post_revised_plan(revision_of, task_prompt, primitive_plan) {
    const URL = `${ROOT_URL}/api/primitive_plan_revision`;

    const response = await fetch(URL, {
        method: "POST",
        headers: { "Content-Type": "application/json"},
        body: JSON.stringify({
            'task_prompt': task_prompt,
            'revision_of': revision_of,
            'primitive_plan': primitive_plan 
            }),
    });

    if (!response.ok) throw new Error("post_task API request failed");

    const plan = await response.json();
    return plan;
}


/**
 * Executes the primitive plan either in sim or on the real robot.
 * @param {Array<Object>} plan The plan to execute in the form of:
 *      [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
 * @param {boolean} on_real Whether to execute on the real system.
 * @param {Array<number>|null} start_index Optional hierarchical start index to begin primitive execution in the middle (e.g. [0,1,2]).
 * @returns {Promise<String>} Execution status in the form of: {'success': True, 'executed_on': 'real'}
 * @throws {Error} If execution fails.
 */
export async function post_plan(plan, on_real, start_index) {
    let url = `${ROOT_URL}/api/execute_plan?on_real=${String(on_real)}`;
        
    if (Array.isArray(start_index)) {
        const params = start_index.map(i => `start_index=${encodeURIComponent(i)}`).join("&");
        url += `&${params}`;
    }
    
    const response = await fetch(url, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(plan),
    });

    if (!response.ok) {
        const text = await response.text();
        throw new Error(`Execution failed: ${text}`);
    }

    const result = await response.json();
    return result;
}

/**
 * Fetches the most recently generated primitive plan.
 * @returns {Promise<Object>} The latest plan in the form of:
 *      {
 *        id: string,
 *        revision_of: string | null,
 *        task_prompt: string,
 *        primitive_plan: [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
 *      }
 * @throws {Error} If the fetch or JSON parsing fails.
 */
export async function get_plan() {
    const URL = `${ROOT_URL}/api/primitive_plan/latest`
    const response = await fetch(URL, {
        method: "GET",
    });

    if (!response.ok) {
        const text = await response.text();
        throw new Error(`get_plan failed: ${text}`);
    }

    const latest_plan = await response.json();
    return latest_plan;
}


/**
 * Fetches all generated prim plans.
 * @returns {Promise<Array<Object>>} All the plans in a list of object in the form of:
 *      {
 *        id: string,
 *        revision_of: string | null,
 *        task_prompt: string,
 *        primitive_plan: [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
 *      }
 * @throws {Error} If the fetch or JSON parsing fails.
 */
export async function get_all_plans() {
    const URL = `${ROOT_URL}/api/primitive_plan/all`
    const response = await fetch(URL, {
        method: "GET",
    });

    if (!response.ok) {
        const text = await response.text();
        throw new Error(`get_plan failed: ${text}`);
    }

    const plans = await response.json();
    return plans;
}



/**
 * Given new parameters, regenerates the low-level prims for a given high-level prim.
 * @param  {Object} primitive Original high level prim in the form of
 *     {'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }
 * @returns {Object} The updated prim in the form of:
 *     {'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }
 * @throws {Error} If the fetch or JSON parsing fails.
 */
export async function post_primitive(primitive) {
    const URL = `${ROOT_URL}/api/primitive`
    const response = await fetch(URL, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(primitive),
    });

    if (!response.ok) {
        const text = await response.text();
        throw new Error(`post_primitive failed: ${text}`);
    }

    const updated_prim = await response.json();
    return updated_prim;
}


/**
 * Gets the idx of the current executing primitive
 * @returns {Array} The index of the currently executing primitive in the form of [first-level-idx,sec-level-idx,...]
        based on the primitive hierarchy from the most recently posted plan to execute.
 * @throws {Error} If the fetch or JSON parsing fails.
 */
export async function get_executing_primitive_idx() {
    const URL = `${ROOT_URL}/api/executing_primitive_idx`
    const response = await fetch(URL, {
        method: "GET",
    });

    if (!response.ok) {
        const text = await response.text();
        throw new Error(`get_executing_primitive_idx failed: ${text}`);
    }

    const idx = await response.json();
    return idx;
}


/**
 * Cancels the currently executing primitive plan
 * @throws {Error} If the fetch or JSON parsing fails.
 */
export async function post_plan_cancel() {
    const URL = `${ROOT_URL}/api/primitive_plan/cancel`
    const response = await fetch(URL, {
        method: "POST",
    });

    if (!response.ok) {
        const text = await response.text();
        throw new Error(`post_plan_cancel failed: ${text}`);
    }
    
    await response.json();
}


