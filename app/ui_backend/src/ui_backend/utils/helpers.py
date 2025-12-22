

def get_current_scene():
    # TODO
    # return [
    #     {"name": "cube", "description": "blue block", "position": [0.3, 0.2, 0.95, 0.0, 0.0, 0.0, 1.0]},
    #     {"name": "bowl", "description": "bowl", "position": [ 0.3, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0]},
    # ]

    # TODO: PASS grasp pose better
    return [
        {"name": "cup", "description": "Small cup. Height: 0.08 (m). Use this grasp pose: [0.2, 0.11, 1, 0, -0.818, 0.574, 0]", "position": [0.2, 0.1, 0.95, 0.0, 0.0, 0.0, 1.0]},
        {"name": "bowl", "description": "Bowl. Height: 0.0476, Width:0.18 (m). Pour location: [0.2, -0.2, 1.15, 0, -0.788, -0.614, 0]", "position": [ 0.2, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0]},
    ]