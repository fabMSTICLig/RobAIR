var ros = new ROSLIB.Ros({
    url: 'wss:' + config.serverurl + ':' + config.rosport
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


var robairros = {
    speed: 100
}


//Command motor
var topic_cmd = new ROSLIB.Topic({
    ros: ros,
    name: '/cmdmotors',
    messageType: 'robairmain/MotorsCmd'
});

robairros.foward = function() {

    var msg = new ROSLIB.Message({
        speedL: robairros.speed,
        speedR: robairros.speed
    });
    topic_cmd.publish(msg);
}

robairros.backward = function() {

    var msg = new ROSLIB.Message({
        speedL: -robairros.speed,
        speedR: -robairros.speed
    });
    topic_cmd.publish(msg);
}


robairros.left = function() {

    var msg = new ROSLIB.Message({
        speedL: -robairros.speed,
        speedR: robairros.speed
    });
    topic_cmd.publish(msg);
}

robairros.right = function() {

    var msg = new ROSLIB.Message({
        speedL: robairros.speed,
        speedR: -robairros.speed
    });
    topic_cmd.publish(msg);
}
robairros.stop = function() {

    var msg = new ROSLIB.Message({
        speedL: 0,
        speedR: 0
    });
    topic_cmd.publish(msg);
}

robairros.setSpeed = function(speed) {
        if (speed < 0) speed = 0;
        else if (speed > 100) speed = 100;
        robairros.speed = parseInt(speed);
}

////PING
var topic_ping = new ROSLIB.Topic({
    ros: ros,
    name: '/ping',
    messageType: 'std_msgs/UInt64'
});
var topic_pong = new ROSLIB.Topic({
    ros: ros,
    name: '/pong',
    messageType: 'std_msgs/UInt64'
});
setInterval(function() {
    var msg = new ROSLIB.Message({
        data: $.now()
    });
  
  robairros.ping=$.now();  
  topic_ping.publish(msg);
}, 1000);
topic_pong.subscribe(function(message) {
  console.log($.now()-robairros.ping);
  robairros.pingChange($.now()-robairros.ping);
});








////Subscribers


//Topic for battery_level
var topic_battery_level = new ROSLIB.Topic({
    ros: ros,
    name: '/battery_level',
    messageType: 'std_msgs/Int32'
});

topic_battery_level.subscribe(function(message) {
    robairros.batteryChange(parseInt(message.data));
});
//Topic for battery_level
var topic_social_touch = new ROSLIB.Topic({
    ros: ros,
    name: '/social_touch',
    messageType: 'std_msgs/Bool'
});

topic_social_touch.subscribe(function(message) {
    robairros.socialChange(message.data);

});
