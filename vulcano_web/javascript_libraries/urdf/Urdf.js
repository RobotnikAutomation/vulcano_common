/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A URDF can be used to load a ROSLIB.UrdfModel and its associated models into a 3D object.
 *
 * @constructor
 * @param options - object with following keys:
 *   * urdfModel - the ROSLIB.UrdfModel to load
 *   * tfClient - the TF client handle to use
 *   * path (optional) - the base path to the associated Collada models that will be loaded
 *   * tfPrefix (optional) - the TF prefix to used for multi-robots
 */
ROS3D.Urdf = function(options) {
  options = options || {};
  var urdfModel = options.urdfModel;
  var path = options.path || '/';
  var tfClient = options.tfClient;
  var tfPrefix = options.tfPrefix || '';

  THREE.Object3D.call(this);
  this.useQuaternion = true;

  // load all models
  var links = urdfModel.links;
  for ( var l in links) {
    var link = links[l];
    if (link.visual && link.visual.geometry) {
      if (link.visual.geometry.type === ROSLIB.URDF_MESH) {
        var frameID = tfPrefix + '/' + link.name;
        var uri = link.visual.geometry.filename;
        var fileType = uri.substr(-4).toLowerCase();

        // ignore mesh files which are not in Collada format
        if (fileType === '.dae') {
          // create the model
          var mesh = new ROS3D.MeshResource({
            path : path,
            resource : uri.substring(10)
          });
          
          // check for a scale
          if(link.visual.geometry.scale) {
            mesh.scale = new THREE.Vector3(
              link.visual.geometry.scale.x,
              link.visual.geometry.scale.y,
              link.visual.geometry.scale.z
            );
          }

          // create a scene node with the model
          var sceneNode = new ROS3D.SceneNode({
            frameID : frameID,
            pose : link.visual.origin,
            tfClient : tfClient,
            object : mesh
          });
          this.add(sceneNode);
        }
      } else {
        var colorMaterial, shapeMesh;
        // Save frameID
        var newFrameID = '/' + link.name;
        // Save color material
        if (link.visual.material && link.visual.material.color) {
          var color = link.visual.material && link.visual.material.color;
          colorMaterial = ROS3D.makeColorMaterial(color.r, color.g, color.b, color.a);
        } else {
          colorMaterial = ROS3D.makeColorMaterial(0, 0, 0, 1);
        }
        // Create a shape
        switch (link.visual.geometry.type) {
            case ROSLIB.URDF_BOX:
                var dimension = link.visual.geometry.dimension;
                var cube = new THREE.CubeGeometry(dimension.x, dimension.y, dimension.z);
                shapeMesh = new THREE.Mesh(cube, colorMaterial);
                break;
            case ROSLIB.URDF_CYLINDER:
                var radius = link.visual.geometry.radius;
                var length = link.visual.geometry.length;
                var cylinder = new THREE.CylinderGeometry(radius, radius, length, 16, 1, false);
                shapeMesh = new THREE.Mesh(cylinder, colorMaterial);
                shapeMesh.useQuaternion = true;
                shapeMesh.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI * 0.5);
                break;
            case ROSLIB.URDF_SPHERE:
                var sphere = new THREE.SphereGeometry(link.visual.geometry.radius, 16);
                shapeMesh = new THREE.Mesh(sphere, colorMaterial);
                break;
        }
        // Create a scene node with the shape
        var scene = new ROS3D.SceneNode({
            frameID: newFrameID,
            pose: link.visual.origin,
            tfClient: tfClient,
            object: shapeMesh
        });
        this.add(scene);
      }
    }
  }
};
ROS3D.Urdf.prototype.__proto__ = THREE.Object3D.prototype;
