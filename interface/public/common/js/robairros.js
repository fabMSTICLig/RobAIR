var ros = new ROSLIB.Ros({
    url : 'wss:'+config.serverurl+':'+config.rosport
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  // Publishing a Topic
  // ------------------

  var cmdToggle = new ROSLIB.Topic({
    ros : ros,
    name : '/toggle_led',
    messageType : 'std_msgs/Empty'
  });

  var toggle = new ROSLIB.Message({});
  cmdToggle.publish(toggle);
