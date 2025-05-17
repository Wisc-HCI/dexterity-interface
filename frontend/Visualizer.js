export class Visualizer {
    constructor(rosConnection, divID) {
      this.ros = rosConnection;
      this.divID = divID;
    }
  
   init() {
      // Create the main viewer.
      let viewer = new ROS3D.Viewer({
        divID : this.divID,
        width : 800,
        height : 600,
        antialias : true,
        fixedFrame: 'scene'
      });

      viewer.addObject(new ROS3D.Grid());
      viewer.camera.position.x = 2;
      viewer.camera.position.y = 2;
      viewer.camera.position.z = 2;

      // Setup a client to listen to TFs.
      let tfClient = new ROSLIB.TFClient({
        fixedFrame: 'panda_link0',
        ros : this.ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0
      });

      // Setup the URDF client.
      let urdfClient = new ROS3D.UrdfClient({
        ros : this.ros,
        tfClient : tfClient,
        path : 'https://raw.githubusercontent.com/Wisc-HCI/panda-primitives/master/', // TODO: Host these somewhere else???
        rootObject : viewer.selectableObjects,
        loader : ROS3D.COLLADA_LOADER_2
      });

      // Setup a client to listen to TFs.
      var tfMarkerClient = new ROSLIB.TFClient({
        ros : this.ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0,
        // fixedFrame : '/world'
        fixedFrame: '/scene'
      });


      var imClient = new ROS3D.InteractiveMarkerClient({
        ros : this.ros,
        tfClient : tfMarkerClient,
        topic : '/scene/object_controls',
        camera : viewer.camera,
        rootObject : viewer.selectableObjects
      });


    }

  }
