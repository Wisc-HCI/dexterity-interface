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
 *      [{'type': 'grasp', 'arm': 'left', pose: [0,0,0,0,0,0,1]}, ...]
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
 *      [{'type': 'grasp', 'arm': 'left', pose: [0,0,0,0,0,0,1]}, ...]
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
      throw new Error("Execution failed");
    }

    const result = await response.json();
    return result;
}

/**
 * Fetches the most recently generated primitive plan.
 * @returns {Promise<Array<Object>>} The latest plan in the form of:
 *      [{'type': 'grasp', 'arm': 'left', pose: [0,0,0,0,0,0,1]}, ...]
 * @throws {Error} If the fetch or JSON parsing fails.
 */
export async function get_plan() {
    const URL = `${ROOT_URL}/api/primitive_plan/latest`
    const response = await fetch(URL, {
        method: "GET",
    });

    const latest_plan = await response.json();
    return latest_plan;
}