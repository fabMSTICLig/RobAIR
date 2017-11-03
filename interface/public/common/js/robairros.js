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

window.onunload = function (e) {
  var e = e || window.event;

  robairros.stop();

  // For IE and Firefox
  if (e) {
    e.returnValue = 'Any string';
  }

  // For Safari
  return 'Any string';
};

// Publishing a Topic
// ------------------


var robairros = {
    speed: 0.5,
    wheel_radius: 0.2,
    turn_factor: 0.5
}


//Command motor
var topic_cmd = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
});

robairros.send_speed_command = function() {
    topic_cmd.publish(robairros._speed_msg);
};

robairros.move = function(linear, angular) {
    robairros._speed_msg = new ROSLIB.Message({
        linear: {
            x: linear,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: angular
        }
    });
};

robairros.foward = function() {
    robairros.move(robairros.speed, 0);
}

robairros.backward = function() {
    robairros.move(-robairros.speed, 0);
}


robairros.left = function() {
    robairros.move(0, robairros.speed
            * robairros.turn_factor / robairros.wheel_radius);
}

robairros.right = function() {
    robairros.move(0, -robairros.speed
            * robairros.turn_factor / robairros.wheel_radius);
}

robairros.stop = function() {
    robairros.move(0, 0);
}

robairros.stop();


robairros.setSpeed = function(speed) {
        if (typeof speed == 'string')
                speed = parseFloat(speed);
        if (speed < 0)
                speed = 0;
        else if (speed > 1)
                speed = 1;
        robairros.speed = speed;
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

robairros.sendPing = function()
{
      robairros.ping=$.now();
      var msg = new ROSLIB.Message({
          data: robairros.ping
      });
      topic_ping.publish(msg);

}

robairros.pongChange =function (){}
topic_pong.subscribe(function(message) {
  robairros.pongChange()
});








////Subscribers


//Topic for battery_level
var topic_battery_level = new ROSLIB.Topic({
    ros: ros,
    name: '/battery_level',
    messageType: 'std_msgs/Int32'
});

robairros.batteryChange =function (val){}
topic_battery_level.subscribe(function(message) {
    robairros.batteryChange(parseInt(message.data));
});

///////////EYES/////////////
var topic_cmdeyes = new ROSLIB.Topic({
    ros: ros,
    name: '/cmdeyes',
    messageType: 'std_msgs/UInt8'
});
var topic_eyes = new ROSLIB.Topic({
    ros: ros,
    name: '/eyes',
    messageType: 'std_msgs/UInt8'
});

robairros.eyesChanges =function (val){}
topic_eyes.subscribe(function(message) {
    robairros.eyesChange(parseInt(message.data));
});

robairros.setEyes = function(id){

    var msg = new ROSLIB.Message({
        data: id
    });
    topic_cmdeyes.publish(msg);
}

///////////head/////////////
var topic_cmdhead = new ROSLIB.Topic({
    ros: ros,
    name: '/cmdhead',
    messageType: 'std_msgs/Int8'
});
var topic_head = new ROSLIB.Topic({
    ros: ros,
    name: '/head',
    messageType: 'std_msgs/Int8'
});

robairros.headChange =function (val){}
topic_head.subscribe(function(message) {
    robairros.headChange(parseInt(message.data));
    //console.log("head "+parseInt(message.data));
});

robairros.setHead = function(degree){

    var msg = new ROSLIB.Message({
        data: degree
    });
    topic_cmdhead.publish(msg);
}



///////////reboot/////////////
var topic_reboot = new ROSLIB.Topic({
    ros: ros,
    name: '/reboot',
    messageType: 'std_msgs/Int8'
})
robairros.rebooting =function (){}

topic_reboot.subscribe(function(message) {

    console.log("reboot");
    setTimeout(function(){location.reload();},5000);
    robairros.rebooting();
});

robairros.reboot = function()
{
  topic_reboot.publish(new ROSLIB.Message({data:0}));

}


///////////bumper/////////////
var topic_bumper_front = new ROSLIB.Topic({
    ros: ros,
    name: '/bumper_front',
    messageType: 'std_msgs/Bool'
})
var topic_bumper_rear = new ROSLIB.Topic({
    ros: ros,
    name: '/bumper_rear',
    messageType: 'std_msgs/Bool'
})
robairros.bumper_front_change =function (on){}
robairros.bumper_rear_change =function (on){}

topic_bumper_front.subscribe(function(message) {
    robairros.bumper_front_change(message.data);
});
topic_bumper_rear.subscribe(function(message) {
    robairros.bumper_rear_change(message.data);
});
///////////hand/////////////
var topic_touch_left = new ROSLIB.Topic({
    ros: ros,
    name: '/touch_left',
    messageType: 'std_msgs/Bool'
})
var topic_touch_right = new ROSLIB.Topic({
    ros: ros,
    name: '/touch_right',
    messageType: 'std_msgs/Bool'
})
robairros.touch_left_change =function (on){}
robairros.touch_right_change =function (on){}

topic_touch_left.subscribe(function(message) {
    robairros.touch_left_change(message.data);
});
topic_touch_right.subscribe(function(message) {
    robairros.touch_right_change(message.data);
});


/////////// Persistent config /////////////

var set_param_srv = new ROSLIB.Service({
    ros: ros,
    name: "/config_manager/set_param",
    serviceType: "mongodb_store/SetParam"
});

var fisheye_enable_param = new ROSLIB.Param({
    ros: ros,
    name: "/enable_fisheye_correction"
});

var fisheye_correction_param = new ROSLIB.Param({
    ros: ros,
    name: "/fisheye_correction"
});


robairros.get_fisheye_params = function(cb) {
    var res = {
        enable: null,
        value: null
    };

    var retrieve_cb = function(param, val) {
        res[param] = val;
        if (res.enable != null && res.value != null)
            cb(res);
    };

    fisheye_enable_param.get(retrieve_cb.bind(null, "enable"));
    fisheye_correction_param.get(retrieve_cb.bind(null, "value"));
};


//TODO: make a helper to create these

robairros.enable_fisheye_correction = function(val) {
    var req = new ROSLIB.ServiceRequest({
        param: JSON.stringify({
            path: "/enable_fisheye_correction",
            value: !!val
        })
    });

    if (arguments[2])
        set_param_srv.callService(req, arguments[1], arguments[2]);
    else if (arguments[1])
        set_param_srv.callService(req, arguments[1]);
    else
        set_param_srv.callService(req, function(res) { });
};

robairros.set_fisheye_correction = function(val) {
    var req = new ROSLIB.ServiceRequest({
        param: JSON.stringify({
            path: "/fisheye_correction",
            value: val
        })
    });

    if (arguments[2])
        set_param_srv.callService(req, arguments[1], arguments[2]);
    else if (arguments[1])
        set_param_srv.callService(req, arguments[1]);
    else
        set_param_srv.callService(req, function(res) { });
};
