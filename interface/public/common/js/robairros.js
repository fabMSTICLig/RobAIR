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
    speed: 0.5
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
    robairros.move(0, robairros.speed);
}

robairros.right = function() {
    robairros.move(0, -robairros.speed);
}

robairros.stop = function() {
    robairros.move(0, 0);
}

robairros.stop();


robairros.analogGamepad = function (dx, dy) {

        var kx = robairros.speed / 100;
        var ky = robairros.speed / 100;

        speed1 = 0;
        speed2 = 0;


                dx=-dx;
                var theta = Math.atan(dy / dx); // En radian
                if (dx <= 0 && dy >= 0) {
                    theta = theta + Math.PI;
                } else if (dx <= 0 && dy <= 0) {
                    theta = theta + Math.PI;
                } else if (dx >= 0 && dy <= 0) {
                    theta = theta + 2 * Math.PI;
                }


                var v, speed1, speed2;

                if (theta >= 0 && theta <= Math.PI / 2) { // 1er cadran
                    if (theta <= Math.PI / 4) { // 1ère moitié du 1er cadran
                        v = dx * kx;
                        speed1 = v * Math.sin(theta + Math.PI / 4);
                        speed2 = v * Math.sin(theta * 2 - Math.PI / 4);
                    } else {
                        v = dy * ky;
                        speed1 = v;
                        speed2 = v * Math.sin(theta);
                    }

                } else if (theta > Math.PI / 2 && theta <= Math.PI) { // 2ème cadran
                    if (theta <= 3 * Math.PI / 4) { // 1ère moitié du 2ème cadran
                        v = dy * ky;
                        speed1 = v * Math.sin(theta);
                        speed2 = v;
                    } else {
                        v = -dx * kx;
                        speed1 = v * Math.sin(theta * 2 - 3 * Math.PI / 4);
                        speed2 = v * Math.sin(theta - Math.PI / 4);
                    }

                } else if (theta > Math.PI && theta <= 3 * Math.PI / 2) { // 3ème cadran
                    if (theta <= 5 * Math.PI / 4) { // 1ère moitié du 3ème cadran
                        v = dx * kx;
                        speed2 = v * Math.sin(theta * 3 - 5 * Math.PI / 4);
                        speed1 = v * Math.sin(Math.PI / 4);
                    } else {
                        v = dy * ky;
                        speed1 = -v * Math.sin(theta);
                        speed2 = v;
                    }

                } else { // 4ème cadran
                    if (theta <= 7 * Math.PI / 4) { // 1ère moitié du 4ème cadran
                        v = dy * ky;
                        speed1 = v;
                        speed2 = -v * Math.sin(theta);
                    } else {
                        v = -dx * kx;
                        speed1 = -v * Math.sin(theta * 3 - 7 * Math.PI / 4);
                        speed2 = v * Math.sin(Math.PI / 4);
                    }

                }


        //speed1 = -speed1;
        //speed2 = -speed2;


        if (speed1 >= robairros.speed) {
            speed1 = robairros.speed;
        }

        if (speed1 <= -robairros.speed) {
            speed1 = -robairros.speed;
        }

        if (speed2 >= robairros.speed) {
            speed2 = robairros.speed;
        }

        if (speed2 <= -robairros.speed) {
            speed2 = -robairros.speed;
        }

        //console.log(""+speed1+" "+speed2);

        var msg = new ROSLIB.Message({
            speedL: speedL,
            speedR: speedR
        });
        topic_cmd.publish(msg);
};

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


window.addEventListener('load', function() {
	setInterval(robairros.send_speed_command, 100);
}, false);
