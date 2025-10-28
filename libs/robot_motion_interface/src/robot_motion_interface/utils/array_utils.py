import numpy as np


def get_partial_update_reference_map(reference_order:list) -> dict:
    """
    Creates a mapping from element names to their index positions in the reference order

    Args:
        reference_order (list): (a) List of element names representing the order of elements (often str).

    Returns:
        (dict): Dictionary mapping element names to their integer index.
            Example: {"joint_1": 0, "joint_2": 1, "joint_3": 2}
    Example:
        {"joint_1": 0, "joint_2": 1, "joint_3": 2} = get_reference_map(["joint_1", "joint_2", "joint_3"])
    """

    reference_map = {name: i for i, name in enumerate(reference_order)}
    return reference_map


def partial_update(original_array:np.ndarray, reference_map:dict, update_array:np.ndarray, update_order:list) -> np.ndarray:
    """
    Updates elements of the original array using update array, based on ordering from update order

    Args:
        original_array (np.ndarray): (a,) Base array to update (not modified in-place)
        reference_map (dict):  Dictionary mapping element names to their indices in original_array. 
            Created with get_partial_update_reference_map()
        update_array (np.ndarray): (a,) Array of new values to insert.
        update_order (list): (n) List of element names corresponding to positions to update (often str).

    Returns:
        (np.ndarray): (a,) New array with specified elements updated.
            
    Example:
        [0.1, 0.5, 0.4] = partial_array_update(original_array=[0.1, 0.2, 0.3], reference_order=["joint_1", "joint_2", "joint_3"],
                                               update_array=[0.4, 0.5] update_order=["joint_3", "joint_2"])
    """
    
    # This is not numpy-ized since the arrays we use are short (and this is a lot more readable)

    updated_array = original_array.copy()  # Don't modify original
    for name, val in zip(update_order, update_array):
        if name not in reference_map:
            raise ValueError(f"{name} is not in reference_map. Options are: {', '.join(reference_map.keys())}")
        updated_array[reference_map[name]] = val
    
    return updated_array
