
## Setup
1. Install rust using [these instructions](https://rust-lang.org/learn/get-started/). Cargo will also be installed by default.
2. Install Ranged IK:
    a. [click here](https://github.com/Wisc-HCI/relaxed_ik_core) and clone our fork of relaxed_ik_core to `~/robot-libs`.
    b. Inside  `~/robot-libs/relaxed_ik_core` run the following to build the package:
    ```bash
    cargo build
    ```
    c. Test that the install was successful by running `cargo run --bin relaxed_ik_bin`. It should output joint solutions.

## Running
Now, Make sure you are in `/dexterity-interface/libs/robot_motion/src/robot_motion/ik` before running the following:

```bash
python3 -m robot_motion.examples.bimanual_ik
```
