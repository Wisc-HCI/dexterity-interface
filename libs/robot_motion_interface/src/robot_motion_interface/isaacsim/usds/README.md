# USD Sources
* bimanual_arms: Generated from the urdfs in this repo (see robot_motion_interface/README.md)
* bowl: Onshape assembly [from here](https://cad.onshape.com/documents/6cc169caeb31cd08d910d204/w/0b283636293523a797a9c429/e/39f8f41ec4da2fc820945c39?renderMode=0&uiState=693adc0720c752a04de73d27) 
* Cup: Onshape assembly [from here](https://cad.onshape.com/documents/4c47262372500a49d0c2832c/w/f787994e7a0c037329aecba9/e/392e89385a5bb7985177a1dd?renderMode=0&uiState=693ad7756e9af32341a9dba0) 


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