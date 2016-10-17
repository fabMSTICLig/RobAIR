function setArrowActive(arrow) {
    $("#" + arrow).removeClass("btn-default");
    $("#" + arrow).addClass("active");
    $("#" + arrow).addClass("btn-primary");
}

var left = function() {
    stop();
    setArrowActive("left");
    robairros.left();
}

var right = function() {
    stop();
    setArrowActive("right");
    robairros.right();
}

var foward = function() {
    stop();
    setArrowActive("foward");
    robairros.foward();
}

var backward = function() {
    stop();
    setArrowActive("backward");
    robairros.backward();
}

var stop = function() {
    $(".active").removeClass("btn-primary");
    $(".active").addClass("btn-default");
    $(".active").removeClass("active");
    robairros.stop();
}

var  setSpeed = function (speed)
{
  robairros.setSpeed(speed);
  $('#speed').val(robairros.speed);
}
$('#speed').val(robairros.speed);

//Gestion evenement utilisateur

$('#speed').on("change", function() {
    setSpeed($(this).val());
});


$(document).keydown(function(e) {
//console.log(e.which);
    switch (e.which) {
        case 37: // left
            left();
            break;
        case 38: // up
            foward();
            break;
        case 39: // right
            right();
            break;
        case 40: // down
            backward();
            break;
        case 107: // down
            setSpeed(robairros.speed+5);
            break;
        case 109: // down
            setSpeed(robairros.speed-5);
            break;
        default:
            return; // exit this handler for other keys
    }
    if (e.which == 37 || e.which == 38 || e.which == 39 || e.which == 40 || e.which == 107 || e.which == 109) {
        e.preventDefault(); // prevent the default action (scroll / move caret)
    }
});

$(document).keyup(function(e) {
    switch (e.which) {
        case 37:
        case 38: // up
        case 39: // right
        case 40: // down
            stop();
            e.preventDefault(); // prevent the default action (scroll / move caret)
            break;
        default:
            return; // exit this handler for other keys
    }
});


$('#left').mousedown(left).mouseup(stop);
$('#right').mousedown(right).mouseup(stop);
$('#foward').mousedown(foward).mouseup(stop);
$('#backward').mousedown(backward).mouseup(stop);





////////////////////////////// EVENT ROS /////////////////////////////////////

robairros.batteryChange = function (battery)
{
  if(battery>26) battery=26;
  else  if(battery<22) battery=22;
  $("battery").removeClass();
  $("battery").addClass("fa-battery-"+(battery-22));
}
