function setArrowActive(arrow) {
    $("#" + arrow).removeClass("btn-default");
    $("#" + arrow).addClass("active");
    $("#" + arrow).addClass("btn-primary");
}

function setArrowDanger(arrow, on) {
    if(on)
    {
      $("#" + arrow).removeClass("btn-default");
      $("#" + arrow).removeClass("active");
      $("#" + arrow).removeClass("btn-primary");
      $("#" + arrow).addClass("btn-danger");
    }
    else {
        $("#" + arrow).removeClass("btn-danger");
        $("#" + arrow).addClass("btn-default");
    }
}

var left = function() {
    stop();
    setArrowActive("left");
    robairros.left();
    console.log("Debug client side left");
}

var right = function() {
    stop();
    setArrowActive("right");
    robairros.right();
    console.log("Debug client side right");
}

var foward = function() {
    stop();
    setArrowActive("foward");
    robairros.foward();
    console.log("Debug client side forward");
}

var backward = function() {
    stop();
    setArrowActive("backward");
    robairros.backward();
    console.log("Debug client side backward");
}

var stop = function() {
    $(".active").removeClass("btn-primary");
    $(".active").addClass("btn-default");
    $(".active").removeClass("active");
    robairros.stop();
    console.log("Debug client side stop"); 
}



var setSpeed = function(speed) {
    robairros.setSpeed(speed);
    $('#speed').val(robairros.speed);
    console.log("Debug client side set speed",speed);
}
$('#speed').val(robairros.speed);

//Gestion evenement utilisateur

$('#speed').on("change", function() {
    setSpeed($(this).val());
});

var lastkeydown = 0;

$(document).keydown(function(e) {

    if (lastkeydown != e.which) {
        //console.log(e.which);
        switch (e.which) {
            case 37: // left
                left();
                lastkeydown = e.which;
                break;
            case 38: // up
                foward();
                lastkeydown = e.which;
                break;
            case 39: // right
                right();
                lastkeydown = e.which;
                break;
            case 40: // down
                backward();
                lastkeydown = e.which;
                break;
            case 107: // down
                setSpeed(robairros.speed + 0.05);
                break;
            case 109: // down
                setSpeed(robairros.speed - 0.05);
                break;
            case 79: // down
                robairros.setHead(5);
                break;
            case 80: // down
                turnheadleft();
                break;
            case 73: // down
                turnheadright();
                break;
            default:
                return; // exit this handler for other keys
        }
    }
    if (e.which == 37 || e.which == 38 || e.which == 39 || e.which == 40 || e.which == 107 || e.which == 109 || e.which == 73 || e.which == 79 || e.which == 80) {
        e.preventDefault(); // prevent the default action (scroll / move caret)
    }
});

$(document).keyup(function(e) {
    switch (e.which) {
        case 37:
        case 38: // up
        case 39: // right
        case 40: // down
            lastkeydown = 0;
            stop();
            e.preventDefault(); // prevent the default action (scroll / move caret)
            break;
        case 79: // down
        case 80: // down
        case 73: // down
            lastkeydown = 0;
            turnheadstop();
            e.preventDefault(); // prevent the default action (scroll / move caret)
            break;
        default:
            return; // exit this handler for other keys
    }
});


$('#left').on('mousedown touchstart', left).on('mouseup touchend', stop);
$('#right').on('mousedown touchstart', right).on('mouseup touchend', stop);
$('#foward').on('mousedown touchstart', foward).on('mouseup touchend', stop);
$('#backward').on('mousedown touchstart', backward).on('mouseup touchend', stop);

$('#refresh').click(function()
{
  robairros.reboot();
  $('#rebootModal').modal();
  var count =5;
  setInterval(function(){
    count--;
    $('#rebootCount').html(""+count);
  },1000)
});





////////////////////////////// EVENT ROS /////////////////////////////////////

robairros.batteryChange = function(battery) {
    if (battery > 26) battery = 26;
    else if (battery < 21) battery = 21;
    $("#battery").removeClass();
    $("#battery").addClass("fa fa-battery-" + (battery - 21));
}

var sendPing = function()
{
  robairros.sendPing();
  robairros.pingTimeout = setTimeout(function() {
    robairros.pingChange(3000);
    sendPing();
  }, 3000);
}

sendPing();

robairros.pongChange = function() {
    var ping= $.now()-robairros.ping;
    clearTimeout(robairros.pingTimeout);
    if(ping < 1000)
    {
      setTimeout(function(){
        sendPing();
      },1000-ping);
    }else {
      sendPing();
    }
    if (ping > 500)
        $("#ping").css('color', 'red');
    else if (ping > 200)
        $("#ping").css('color', 'orange');
    else {
        $("#ping").css('color', 'black');
    }
    $("#pingtxt").text(ping);
}
    /////////////////////////////YEUX//////////////////////////////

robairros.eyesChange = function(id) {
    Eyes.drawEyes(id);
}

var eyesCanvas = $("#eyesCanvas");
eyesCanvas.on("mousedown touchstart", function(e) {
    //var posX = (e.pageX - $(this).offset().left) / $(this).width(),
    //    posY = (e.pageY - $(this).offset().top) / $(this).height();
    var posX = -1;
    var posY = -1;
    var leftOffset = $(this).offset().left;
    var topOffset  = $(this).offset().top;

    if(e.type == 'mousedown'){
       posX = (e.pageX - leftOffset) / eyesCanvas.width();
       posY = (e.pageY - topOffset)  / eyesCanvas.height();
    }
    else if(e.type == 'touchstart'){
       posX = (e.originalEvent.touches[0].pageX - leftOffset) / eyesCanvas.width();
       posY = (e.originalEvent.touches[0].pageY - topOffset) / eyesCanvas.height();
    }
    if (posX < 0.25) {
        //Eyes.drawEyes(Eyes.EYESLEFT);
        robairros.setEyes(Eyes.EYESLEFT);
    } else if (posX > 0.75) {
        //Eyes.drawEyes(Eyes.EYESRIGHT);
        robairros.setEyes(Eyes.EYESRIGHT);
    } else if (posY < 0.25) {
        //Eyes.drawEyes(Eyes.EYESTOP);
        robairros.setEyes(Eyes.EYESTOP);
    } else if (posY > 0.75) {
        //Eyes.drawEyes(Eyes.EYESBOTTOM);
        robairros.setEyes(Eyes.EYESBOTTOM);
    } else {
        //Eyes.drawEyes(Eyes.EYESSTRAIGHT);
        robairros.setEyes(Eyes.EYESSTRAIGHT);
    }
    e.preventDefault();
});


//////////////////////HEAD////////////////


var headcur = 0;
robairros.headChange = function(deg) {
    headcur=deg;
    setHeadTheta(deg);
}

function setHeadTheta(val) {
    var srotate = "rotate(" + val + "deg)";
    $("#head").css({
        "-webkit-transform": srotate,
        "transform": srotate,
        "-webkit-transform-origin": "50% 100%",
        "transform-origin": "50% 100%"
    });

}
var turnheadright = function() {

  robairros.setEyes(Eyes.EYESLEFT);
  robairros.setHead(90);
  console.log("debug head right");
}
var turnheadleft = function() {

  robairros.setEyes(Eyes.EYESRIGHT);
  robairros.setHead(-90);
  console.log("debug head left");
}
var turnheadstop = function() {
  robairros.setHead(headcur);
  robairros.setEyes(Eyes.EYESSTRAIGHT);
  console.log("debug stop");
}

$('#headorigin').on('mousedown touchstart', turnheadstop);
$('#headleft').on('mousedown touchstart', turnheadleft).on('mouseup touchend', turnheadstop);
$('#headright').on('mousedown touchstart', turnheadright).on('mouseup touchend', turnheadstop);


var analogGamepad = function (dx, dy) {
    var norm = Math.sqrt(dx*dx + dy*dy);
    if (norm < 0.01)
        return [0, 0];
    if (norm > 1) {
        dx = dx / norm;
        dy = dy / norm;
        norm = 1;
    }

    var theta = Math.atan(dy / dx);
    if (dx < 0) {
        theta = theta + Math.PI;
    } else if (dy < 0) {
        theta = theta + 2 * Math.PI;
    }

    var linear = Math.sin(theta) * norm * robairros.speed;
    var angular = -Math.cos(theta) * norm * robairros.speed
        * robairros.turn_factor / robairros.wheel_radius;

    return [linear, angular];
}


Math.degrees = function(radians) {
    return radians * 180 / Math.PI;
};

var headCanvas = $("#overlay");
console.log(headCanvas);
headMouseDown = false;

headCanvas.on("mouseup touchend mouseleave", function(e) {
    headMouseDown = false;
    robairros.stop();
});
headCanvas.on("mousedown touchstart", function(e) {

        headMouseDown = true;

 	var posX = -1;
        var posY = -1;
        var leftOffset = $(this).offset().left;
        var topOffset  = $(this).offset().top;

        if(e.type == 'mousedown'){
           posX = (e.pageX - leftOffset) / headCanvas.width();
           posY = (e.pageY - topOffset)  / headCanvas.height();
        }
        else if(e.type == 'touchstart'){
           posX = (e.originalEvent.touches[0].pageX - leftOffset) / headCanvas.width();
           posY = (e.originalEvent.touches[0].pageY - topOffset) / headCanvas.height();
        }

        posX = posX * 2 - 1;
        posY = (posY * 2 - 1)*-1;
        var speeds = analogGamepad(posX, posY);
        robairros.move(speeds[0], speeds[1]);
	e.preventDefault();
});
headCanvas.on("mousemove touchmove", function(e) {
    if (headMouseDown) {
	var posX = -1;
	var posY = -1;
	var leftOffset = $(this).offset().left;
        var topOffset  = $(this).offset().top;
	
	if(e.type == 'mousemove'){
	   posX = (e.pageX - leftOffset) / headCanvas.width();
           posY = (e.pageY - topOffset)  / headCanvas.height();
	}
	else if(e.type == 'touchmove'){
	   posX = (e.originalEvent.touches[0].pageX - leftOffset) / headCanvas.width();
           posY = (e.originalEvent.touches[0].pageY - topOffset) / headCanvas.height();
	}

        posX = posX * 2 - 1;
        posY = (posY * 2 - 1)*-1;
        var speeds = analogGamepad(posX, posY);
        robairros.move(speeds[0], speeds[1]);
    }
    e.preventDefault();
    //e.stopPropagation();
});
//////////////////////////////Bumper ///////////////////////

robairros.bumper_front_change =function (on){
  if(on)
  {
    $('#bumperFront').show();
  }
  else
  {
    $('#bumperFront').hide();
  }
  setArrowDanger('foward', on);
}
robairros.bumper_rear_change =function (on){
  if(on) $('#bumperRear').show();
  else $('#bumperRear').hide();
  setArrowDanger('backward', on);
}

$('#bumperRear').hide();
$('#bumperFront').hide();

//////////////////////////////Bumper ///////////////////////

robairros.touch_left_change =function (on){
  if(on) $('#touchLeft').addClass('touched');
  else $('#touchLeft').removeClass('touched');
}
robairros.touch_right_change =function (on){
  if(on) $('#touchRight').addClass('touched');
  else $('#touchRight').removeClass('touched');
}


setInterval(robairros.send_speed_command, 100);
