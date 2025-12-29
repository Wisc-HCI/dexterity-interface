import {ROOT_URL} from "/src/js/constants.js"


export async function post_objects() {
    const URL = `${ROOT_URL}/api/spawn_objects`;

    const response = await fetch(URL, {
            method: "POST",
            headers: { "Content-Type": "application/json"},
    });
    if (!response.ok) throw new Error("post_objects API request failed.");
}

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


export async function get_plan() {
    const URL = `${ROOT_URL}/api/primitive_plan/latest`
    const response = await fetch(URL, {
        method: "GET",
    });
    
    const latest_plan = await response.json();
    return latest_plan;
}