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
        robairros.speed = speed;
    }
    ////Subscribers


//Topic for battery_level
var topic_battery_level = new ROSLIB.Topic({
    ros: ros,
    name: '/battery_level',
    messageType: 'std_msgs/Byte'
});

topic_battery_level.subscribe(function(message) {
    robairros.batteryChange(parseInt(message.data));
});
