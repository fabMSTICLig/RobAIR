

var left = function() {
    stop();
    robairros.left();
}

var right = function() {
    stop();
    robairros.right();
}

var foward = function() {
    stop();
    robairros.foward();
}

var backward = function() {
    stop();
    robairros.backward();
}

var stop = function() {
    robairros.stop();
}


var setSpeed = function(speed) {
    robairros.setSpeed(speed);
}

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
                setSpeed(robairros.speed + 5);
                break;
            case 109: // down
                setSpeed(robairros.speed - 5);
                break;
            default:
                return; // exit this handler for other keys
        }
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
            lastkeydown = 0;
            stop();
            e.preventDefault(); // prevent the default action (scroll / move caret)
            break;
        default:
            return; // exit this handler for other keys
    }
});


robairros.rebooting = function ()
{
  $('#rebootModal').modal();
  var count =5;
  setInterval(function(){
    count--;
    $('#rebootCount').html(""+count);
  },1000)
}
