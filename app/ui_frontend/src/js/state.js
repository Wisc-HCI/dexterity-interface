

let state = {
  plan: [],
  editing_index: null,
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
