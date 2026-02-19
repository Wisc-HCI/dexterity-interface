# USD Sources
* bimanual_arms: Generated from the urdfs in this repo (see robot_motion_interface/README.md)
* bowl: Onshape assembly [from here](https://cad.onshape.com/documents/6cc169caeb31cd08d910d204/w/0b283636293523a797a9c429/e/39f8f41ec4da2fc820945c39?renderMode=0&uiState=693adc0720c752a04de73d27) 
* Cup: Onshape assembly [from here](https://cad.onshape.com/documents/4c47262372500a49d0c2832c/w/f787994e7a0c037329aecba9/e/392e89385a5bb7985177a1dd?renderMode=0&uiState=693ad7756e9af32341a9dba0) 
* Funnel: Onshape assembly [from here](https://cad.onshape.com/documents/d19516ade0bd4f9b75b36c79/w/148b417dd4a60ee78b1c0e7a/e/fc6293d880b389c65e32171c?renderMode=0&uiState=6971447d64b693f60a8b9257)
* Spoon: Onshape assembly [from here](https://cad.onshape.com/documents/8e34ac7b478eeb3f67e713fa/w/0798a40d542eb6368b6e0907/e/18405d683bf7eed03cd324e3?renderMode=0&uiState=69964222006fd3d4817426ef)
* Fork: Onshape assembly [from here](https://cad.onshape.com/documents/c9c1df3349466dccee8e63e1/w/5c48859f4483fed9322af5f3/e/36389c4973592ca076b52f22?renderMode=0&uiState=6996462c6ff4824731a08cad)
* Wooden Cooking Spoon: Onshape assembly [from here](https://cad.onshape.com/documents/6d4d50a9c1839f4bda8e2e10/w/342c8ce12e507ff6e049919c/e/2c76611c5c6a93415363def0?renderMode=0&uiState=6996464a006fd3d48174502b)
* Sauce Bowl: Onshape assembly [from here](https://cad.onshape.com/documents/d00afad54766b0dd01ef6337/w/1f10703e33d39009bef76315/e/ab69c69d2ede53c113d8a889?renderMode=0&uiState=69964667fa14040e0fc03329)
* Mug Cup: Onshape assembly [from here](https://cad.onshape.com/documents/7feb72aedd1151e2f3f7f80e/w/b77d4979c20199f413dac624/e/56cbe1967b3e8a8ee093f3f9?renderMode=0&uiState=6996467f2f6f60774d6c466c)
* Plate: Onshape assembly [from here](https://cad.onshape.com/documents/f60508ae50b65acfbf3e7552/w/bace9845f212cdc1e9b7e8dc/e/d4a4353d45865efa2925ddf2?renderMode=0&uiState=699646959ef12859708e29c1)
* Plastic Box: Onshape assembly [from here](https://cad.onshape.com/documents/528682956771b7016628844e/w/10cb4de2421cac24d9e29f2a/e/6cf9fabcb28352e4a57b56f8?renderMode=0&uiState=699646a4fa14040e0fc0346b)

# Onshape Conversions
In order to convert the Onshape assemblies, the following steps were taken.
1. In your Onshape account, make sure you can open the onshape file.
2. Open Isaacsim and Select `File` > `Import from Onshape`. Authenticate Onshape when prompted.
3. In the Isaacsim Onshape Importer, double click the root of the object (i.e. bowl or cup).
4. In the table that shows up, choose PLA for the Material for each part. Then exit the popup.
5. In the Isaacsim stage side-panel, right-click the root of the object and select `Add` > `Physics` > `Rigid Body with Colliders Preset`.
6. Under the object, for EVERY part, click each one (should have child Looks folder and mesh) and scroll down in the Property tab and under `Physics` > `Collider` > `Approximation`, make sure `Convex Hull` is selected.
7. Click the second child from the root and under the Property tab under `Physics` > `Articulation Root`, make sure `Articulation-Enabled` is un-selected.
8. Then again in the Stage side panel, right click the root object and select Save Selected. Make sure to select .usd in the popup and save to your desired location.