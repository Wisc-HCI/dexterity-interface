# USD Sources
* bimanual_arms: Generated from the urdfs in this repo (see robot_motion_interface/README.md)
* bowl: Onshape assembly [from here](https://cad.onshape.com/documents/6cc169caeb31cd08d910d204/w/0b283636293523a797a9c429/e/39f8f41ec4da2fc820945c39?renderMode=0&uiState=693adc0720c752a04de73d27) 
* Cup: Onshape assembly [from here](https://cad.onshape.com/documents/4c47262372500a49d0c2832c/w/f787994e7a0c037329aecba9/e/392e89385a5bb7985177a1dd?renderMode=0&uiState=693ad7756e9af32341a9dba0) 
* Funnel: Onshape assembly [from here](https://cad.onshape.com/documents/d19516ade0bd4f9b75b36c79/w/148b417dd4a60ee78b1c0e7a/e/fc6293d880b389c65e32171c?renderMode=0&uiState=6971447d64b693f60a8b9257)

# Onshape Conversions
In order to convert the Onshape assemblies, the following steps were taken.
1. In your Onshape account, make sure you can open the onshape file.
2. Open Isaacsim and Select `File` > `Import from Onshape`. Authenticate Onshape when prompted.
3. In the Isaacsim Onshape Importer, double click the root of the object (i.e. bowl or cup).
4. In the table that shows up, choose PLA for the Material for each part. Then exit the popup.
5. Now add colliders:
    1. In the Isaacsim stage side-panel, right-click the root of the object and select `Add` > `Physics` > `Rigid Body with Colliders Preset`.
    2. Under the object, for EVERY part, click each one (should have child Looks folder and mesh) and in the Property tab and under `Physics` > `Collider` > `Approximation`, make sure `Convex Hull` is selected.
    3. Click the second child from the root and under the Property tab under `Physics` > `Articulation Root`, make sure `Articulation-Enabled` is un-selected.
6. Now add friction:
    1. In the Isaacsim stage side-panel, right-click anywhere and select `Create` > `Physics` > `Physics Material`. When the popup shows up, check `Rigid Body Material`.
    2. Click the newly generated material (should be named `PhysicsMaterial`). In the `Property Panel` under `Physics` > `Rigid Body Material`, set `Friction Combine Mode` to `Max`. Then enter the Dynamic Friction and Static Friction according to your desired material. Here are some guidelines:
    * Plastic: Static Friction=0.5, Dynamic Friction=0.4
    * Rubber: Static Friction=1.0, Dynamic Friction=0.8
    3. Again in the Stage side panel, under the object, for EVERY part, click each one (should have child Looks folder and mesh) and in the Property tab and under `Material on selected model`, in the field that says `None`, select the material (should be named `/Root/PhysicsMaterial`).
8. Then again in the Stage side panel, click the root object, then right click the root object and select Save Selected. Make sure to select .usd in the popup and save to your desired location.