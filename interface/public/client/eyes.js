
var Eyes = {};

var eyesstraight=[
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,2,0,7,0,0,7,0,2,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
];


var eyesright=[
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,0,2,7,0,0,7,0,0,2,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
];

var eyesleft=[
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,2,0,0,7,0,0,7,2,0,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
];


var eyestop=[
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,2,0,7,0,0,7,0,2,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
];

var eyesbottom=[
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,2,0,7,0,0,7,0,2,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
];

var exclamations=[
    0,0,1,0,1,0,1,0,1,0,1,0,0,0,
    0,0,1,0,1,0,1,0,1,0,1,0,0,0,
    0,0,1,0,1,0,1,0,1,0,1,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,1,0,1,0,1,0,1,0,1,0,0,0
];

var interogations=[
    0,4,4,4,0,4,4,4,0,4,4,4,0,0,
    0,4,0,4,0,4,0,4,0,4,0,4,0,0,
    0,0,4,4,0,0,4,4,0,0,4,4,0,0,
    0,0,4,0,0,0,4,0,0,0,4,0,0,0,
    0,0,2,0,0,0,2,0,0,0,2,0,0,0
];

var stop=[
    1,1,1,0,1,1,1,0,1,1,1,0,1,1,
    1,0,0,0,0,1,0,0,1,0,1,0,1,1,
    1,1,1,0,0,1,0,0,1,0,1,0,1,0,
    0,0,1,0,0,1,0,0,1,0,1,0,1,0,
    1,1,1,0,0,1,0,0,1,1,1,0,1,0
];

var hello=[
    7,0,7,0,2,2,0,1,0,1,0,5,5,5,
    7,0,7,0,2,0,0,1,0,1,0,5,0,5,
    7,7,7,0,2,2,0,1,0,1,0,5,0,5,
    7,0,7,0,2,0,0,1,0,1,0,5,0,5,
    7,0,7,0,2,2,0,1,1,1,1,5,5,5

];


var vide=[
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0
];

var error=[
    1,1,3,3,3,1,1,1,3,3,3,1,1,1,
    1,0,3,0,3,1,0,1,3,0,3,1,0,1,
    1,1,3,3,3,1,1,1,3,0,3,1,1,1,
    1,0,3,3,0,1,1,0,3,0,3,1,1,0,
    1,1,3,0,3,1,0,1,3,3,3,1,0,1
];

Eyes.EYESVIDE=0;
Eyes.EYESSTRAIGHT=1;
Eyes.EYESRIGHT=2;
Eyes.EYESLEFT=3;
Eyes.EYESTOP=4;
Eyes.EYESBOTTOM=5;
Eyes.EYESEXCLAMATIONS=6;
Eyes.EYESINTEROGATIONS=7;
Eyes.EYESSTOP=8;
Eyes.EYESHELLO=9;
Eyes.EYESERROR=10;

var tabeyes = [
    vide,
    eyesstraight,
    eyesright,
    eyesleft,
    eyestop,
    eyesbottom,
    exclamations,
    interogations,
    stop,
    hello,
    error
]


function valtocolor(val)
{
    r=((val&1) == 1)? "FF":"96";
    g=((val&2) == 2)? "FF":"96";
    b=((val&4) == 4)? "FF":"96";
    return "#"+r+g+b;
}

function drawcircle(ctx,x,y,color)
{
    ctx.beginPath();
    ctx.arc(parseInt(x),parseInt(y),5,0,2*Math.PI);
    ctx.fillStyle = color;
    ctx.fill();
}


var eyesCanvas = document.getElementById("eyesCanvas");
eyesCanvas.style.backgroundColor = '#969696';
var eyesctx = eyesCanvas.getContext("2d");

Eyes.drawEyes = function(id)
{
    var ctx = eyesctx;
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    // Will always clear the right space
    ctx.clearRect(0, 0, eyesCanvas.width, eyesCanvas.height);
    ctx.restore();
    var t=tabeyes[id];
    /*for(i=0;i<5;i++)
    {
        for(j=0;j<14;j++)
        {
            //drawcircle(ctx,10+20*(j),10+15*i,valtocolor(t[i*14+(j)]));
        }
    }*/
    for(i=0;i<5;i++)
    {
        for(j=0;j<14;j++)
        {
            drawcircle(ctx,45+15*(13-j),5+11*i,valtocolor(t[i*14+(13-j)]));
        }
    }
}


Eyes.drawEyes(1);
