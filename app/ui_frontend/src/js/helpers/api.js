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
 * @param {String} taskThe task description to be planned.
 * @returns {Promise<Array<Object>>} The generated primitive plan in the form of:
 *      [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
 * @throws {Error} If the API request fails.
 */
export async function post_task(task) {
    const URL = `${ROOT_URL}/api/primitive_plan`;

    const response = await fetch(URL, {
        method: "POST",
        headers: { "Content-Type": "application/json"},
        body: JSON.stringify({ task }),
    });

    if (!response.ok) throw new Error("post_task API request failed");

    const primitives = await response.json();
    return primitives;
}


/**
 * Executes the primitive plan either in sim or on the real robot.
 * @param {Array<Object>} plan The plan to execute in the form of:
 *      [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
 * @param {boolean} on_real Whether to execute on the real system.
 * @returns {Promise<String>} Execution status in the form of: {'success': True, 'executed_on': 'real'}
 * @throws {Error} If execution fails.
 */
export async function post_plan(plan, on_real) {
    const URL = `${ROOT_URL}/api/execute_plan?on_real=${String(on_real)}`;

    const response = await fetch(URL, {
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
 * @returns {Promise<Array<Object>>} The latest plan in the form of:
 *     [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
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
 * Gets the  TODO
 * @param  {Object} primitive Original high level prim in the form of
 *     {'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }
 * @returns {Object} The updated prim in the form of:
 *     {'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }
 * @throws {Error} If the fetch or JSON parsing fails.
 */
export async function get_primitive(primitive) {
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