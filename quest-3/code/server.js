// Modules
var dgram = require('dgram'); // Socket programming
const { DateTime } = require('luxon'); // Get the current time of day
var fs = require('fs'); // To save to a file
let now = DateTime.now();

// Port and IP
var PORT = 3333; // Initialize a port
var HOST = '10.239.114.40'; // Ip address of pi

// Create a CSV file
fs.writeFile('data.csv', '', function (err) {
    if (err) throw err;
});

// Create socket
var server = dgram.createSocket('udp4');

// Create server that listens on a port
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// ------ Initialize some variables to track leaderboard stats ------
let carminID = []; // Track which carmins are connected
let count = 0;
let stepsArr = [];
let tempArr = [];

// On connection, print out received message
server.on('message', function (message, remote) {
    console.log(remote.address + ':' + remote.port +' - ' + message);
    let carminData = remote.address + ':' + remote.port + "-" + message; // Later parse by "-" to get the sensor contents

    // Save carmin watch data to CSV in format IPaddress:Port-Sensor,Sensor
    fs.appendFile('data.csv', content, function (err) {
        if (err) throw err;
    });

    // ------ Add some leaderboard/parsing logic ------
    let leader = "";

    // Send leaderboard information
    server.send(leader,remote.port,remote.address,function(error){ // Send leader to carmin watches
      if(error){
        console.log('MEH!');
      }
      else{
        console.log('Sent: ', leader);
      }
    });
});

// Bind server to port and IP
server.bind(PORT, HOST);

