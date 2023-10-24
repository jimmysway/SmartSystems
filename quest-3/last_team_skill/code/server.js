// Required module
var dgram = require('dgram');

// Port and IP
var PORT = 8080;
var HOST = '192.168.1.18';

let BLINK_TIME = 1000;

// Create socket
var server = dgram.createSocket('udp4');

// Create server that listens on a port
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

server.on('message', (msg, rinfo) => {
  console.log(`server got: ${msg} from ${rinfo.address}:${rinfo.port}`);
});

server.on('message', (msg, rinfo) => {
  console.log(`Server received: ${msg} from ${rinfo.address}:${rinfo.port}`);

  if (msg.toString() === "GET_BLINK_TIME") {
      server.send(String(BLINK_TIME), rinfo.port, rinfo.address, (error) => {
          if (error) {
              console.error(`UDP message send error: ${error.stack}`);
              server.close();
          } else {
              console.log(`Blink time sent to ${rinfo.address}:${rinfo.port}`);
          }
      });

      BLINK_TIME = (BLINK_TIME === 1000) ? 100 : 1000;
  }
});

// Bind server to port and IP
server.bind(PORT, HOST);

