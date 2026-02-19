

let state = {
  id: null,
  revision_of: null,
  primitive_plan: [],
  task_prompt: null,
  
  expanded: new Set(),   // TODO: Handle multiple levels
  editing_index: null,   // The index is [first-level-idx,sec-level-idx,...]
  executing_index: null, // The index is [first-level-idx,sec-level-idx,...]
  pause: true,  // Start paused

  // Don't update scene with vision
  scene_frozen: false,
};

const listeners = new Set();

/**
 * Read-only access to state
 */
export function get_state() {
  return state;
}

/**
 * Update state and notify observers
 */
export function set_state(patch) {
  state = { ...state, ...patch };
  listeners.forEach((fn) => fn(state));
}

/**
 * Subscribe to state changes
 * @returns {function} an unsubscribe function
 */
export function subscribe_state (fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}



/**
 * Subscribe with access to both current and previous state.
 * @param {(state, prev_state) => void} callback
 */
export function subscribe_state_with_prev(callback) {
    let prev_state = null;

    subscribe_state((state) => {
        callback(state, prev_state);
        prev_state = structuredClone(state);
    });
}