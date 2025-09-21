https://docs.omniverse.nvidia.com/services/latest/services/web-streaming-library/overview.html

@omniverse:registry=https://edge.urm.nvidia.com/artifactory/api/npm/omniverse-npm/
npm install @nvidia/omniverse-webrtc-streaming-library

npm run build
npx serve dist

LIVESTREAM=2  ENABLE_CAMERAS=1 python3 -m robot_motion_interface.isaacsim.isaacsim_interface

# TODO???
https://medium.com/@BeingOttoman/scalable-streaming-nvidia-omniverse-applications-over-the-internet-using-webrtc-8946a574fef2

https://github.com/NVIDIA-Omniverse/configurator-viewer-sample


isaacsim isaacsim.exp.full.streaming --no-window --/app/livestream/port=49100  --/app/livestream/publicEndpointAddress=127.0.0.1


LIVESTREAM=2  python3 -m robot_motion_interface.isaacsim.isaacsim_interface

LIVESTREAM=2  python3 -m robot_motion_interface.isaacsim.isaacsim_interface --kit_args="--/app/window/hideUi=true --/app/window/drawMouse=false"


npm run dev

npm run build
npx serve dist