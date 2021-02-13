var ROSLIB = require('roslib');
var ROS3D = require('ros3d');

var ros = new ROSLIB.Ros({
    url: 'ws://' + process.env.REACT_APP_ROS_BRIDGE_URL + ':' + process.env.REACT_APP_ROS_BRIDGE_PORT
});

ros.on('connection', function () {
    console.log('Connected to websocket server.');
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function () {
    console.log('Connection to websocket server closed.');
});

var JointStatesListener = (topic) => {
    return new ROSLIB.Topic({
        ros: ros,
        name: topic,
        messageType: 'sensor_msgs/JointState'
    });
};

var tfClientToFrame = (fixedFrame, rate) => {
    return new ROSLIB.TFClient({
        ros: ros,
        fixedFrame: fixedFrame,
        angularThres: 0.01,
        transThres: 0.01,
        rate: rate
    });
};

var viewer3d = (divId, width, height, cameraPosition) => {
    return new ROS3D.Viewer({
        divID: divId,
        width: width,
        height: height,
        antialias: true,
        cameraPose: cameraPosition
    });
};

var markerClient = (tfClient, markerTopic, viewer3d) => {
    return new ROS3D.MarkerClient({
        ros: ros,
        tfClient: tfClient,
        topic: markerTopic,
        rootObject: viewer3d.scene
    });
};

var markerArrayClient = (tfClient, markerTopic, viewer3d) => {
    return new ROS3D.MarkerArrayClient({
        ros: ros,
        tfClient: tfClient,
        topic: markerTopic,
        rootObject: viewer3d.scene
    });
};

var urdfClient = (tfClient, viewer3d, path) => {
    return new ROS3D.UrdfClient({
        ros: ros,
        tfClient: tfClient,
        path: path,
        rootObject: viewer3d.scene
    });
};


module.exports = {
    JointStatesListener,
    tfClientToFrame,
    viewer3d,
    markerClient,
    markerArrayClient,
    urdfClient
};
