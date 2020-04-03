var http = require('http');
var url = require('url');
var fs = require('fs');
const SerialPort = require('serialport');
const Readline = SerialPort.parsers.Readline;
var dgram = require('dgram');

var HOST = '192.168.1.108';
var WEBPORT = 3000;
var UDPPORT = 8080;
var ESPHOST = 0;
var ESPPORT = 0;

// ======================== WEBSERVER

console.log('Initializing Webserver on port: ' + WEBPORT);

http.createServer(function (req, res) {
  var q = url.parse(req.url, true);
  var filename = "." + q.pathname;

  if (q.pathname == "/start" || q.pathname == "/stop") {
    sendToEsp(q.pathname)
    res.write('light recieved.'); //write a response to the client
    res.end(); //end the response
  }
  else{ 
    fs.readFile(filename, function(err, data) {
      if (err) {
        fs.readFile("404.html", function(err, data) {
          res.writeHead(200, {'Content-Type': 'text/html'});
          res.write(data);
          return res.end();
        });
      }
      else {
      res.writeHead(200, {'Content-Type': 'text/html'});
      res.write(data);
      return res.end();
  }
  });
}
}).listen(WEBPORT);

// end Webserver

// ======================== ESP ENDPOINT

// Create socket
var server = dgram.createSocket('udp4');

// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('Initializing UDP Server on ' + address.address + ":" + address.port + "\n\n");
});


// if the server has not seen the esp32 yet.
if (ESPPORT == 0 && ESPHOST == 0){
  // On connection, print out received message
  server.on('message', function (message, remote) {
      if (message == "INIT") {
        ESPHOST = remote.address
        ESPPORT = remote.port
        console.log('Device connected and initialized @: ' + remote.address + ':' + remote.port)
      }
      else {
        console.log("Message from ESP: "+ message)
      }
  });
}



function sendToEsp(pathname) {

  if (ESPPORT != 0 && ESPHOST != 0){

    //send message to esp
    server.send(pathname,ESPPORT,ESPHOST,function(error){
        if(error){
          console.log('ERROR: Sending ' + pathname + ' Signal to ESP.');
        }
        else{
          console.log('- Sending ' + pathname + ' to ESP: ' + ESPPORT + ':' + ESPHOST);
        }
      });
  }

  else {
    console.log('ERROR: esp not connected, cannot send ' + pathname + ' data');
  }
}

// Bind server to port and IP
server.bind(UDPPORT, HOST);

// end esp endpoint

