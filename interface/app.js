var fs = require('fs'),
    https = require('https'),
    http = require('http'),
    express = require('express'),
    app = express();
var session = require('express-session')

//Config file
var data = fs.readFileSync("config.json");
var config = JSON.parse(data);


//Local interface
var ifs = require('os').networkInterfaces();
localAddressTab = Object.keys(ifs).map(x => ifs[x].filter(x => x.family === 'IPv4' && !x.internal)[0]).filter(x => x);
var localAddress='localhost';
if (localAddressTab.length > 0)
{
	localAddress=localAddressTab[0].address;
}



///////Mini Serveur pour téléchargé l'autorité de certification////////////
apphttp = express();
apphttp.use('/common', express.static('public/common'));
apphttp.get('/', function(req, res) {
    res.header('Content-type', 'text/html');
    return res.end('<h1>Installer le certificat en tant qu\'autorité racine de confiance. Puis redémarez votre navigateur.</h1> <a href="common/rootCA.crt">CA</a> <br/> <a href="https://'+localAddress+':6080">Contrôle</a>');
});
http.createServer(apphttp).listen(config.httpPort);

//Session initialisation
app.set('trust proxy', 1) // trust first proxy
app.use(session({
    secret: 'fabmstic',
    resave: false,
    httpOnly: true,
    saveUninitialized: true,
    cookie: {
        secure: true
    }
}))

app.use('/', express.static('public/'));
app.get('/', function(req, res) {
    res.header('Content-type', 'text/html');
    console.log(localAddress+" "+req.ip);
    if ("::ffff:" + localAddress == req.ip)
            return res.sendFile(__dirname+'/views/robair/index.html');
    else {
        return res.sendFile(__dirname+'/views/client/index.html');
    }
});
app.get('/local', function(req, res) {
    res.header('Content-type', 'text/html');
        return res.sendFile(__dirname+'/views/client/index.html');
});

https.createServer({
    key: fs.readFileSync(config.ssl.key),
    cert: fs.readFileSync(config.ssl.crt)
}, app).listen(config.httpsPort);
