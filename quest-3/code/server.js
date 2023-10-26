// Modules
var dgram = require('dgram'); // Socket programming
const { DateTime } = require('luxon'); // Get the current time of day
var fs = require('fs'); // To save to a file
let now = DateTime.now();

// Port and IP
var PORT = 3333; // Initialize a port
var HOST = '192.168.1.36'; // Ip address of pi

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

// initialize leaderboard
let leaderboard = [];


// On connection, print out received message
// On connection, print out received message
server.on('message', function (message, remote) {
  let carminData = remote.address + ':' + remote.port + "-" + message;
  console.log(carminData);

  // Save carmin watch data to CSV in format IPaddress:Port-Sensor,Sensor
  fs.appendFile('data.csv', carminData, function (err) {
      if (err) throw err;
  });

  // ------ Update leaderboard/parsing logic ------
  let data = message.split(",");
  let stepsReceived = parseInt(data[0]);

  // If this watch has the highest number of steps, update the leader
  if (stepsReceived > leaderSteps) {
      leaderSteps = stepsReceived;
      leader = `${remote.port} - Steps: ${leaderSteps}, Temp: ${parseFloat(data[1])}`;
  }

  // Send leaderboard information (only the leader)
  server.send(leader, remote.port, remote.address, function (error) {
      if (error) {
          console.log('MEH!');
      } else {
          console.log('Sent Leader:\n', leader);
      }
  });
});

  // Send leaderboard information
  server.send(leaderboardMessage, remote.port, remote.address, function (error) {
      if (error) {
          console.log('MEH!');
      } else {
          console.log('Sent Leaderboard:\n', leaderboardMessage);
      }
  });

  // Limit the leaderboard to the top 10 entries to prevent overflow
  if (leaderboard.length > 10) {
      leaderboard.pop(); // Remove the last entry
  }
});

// Bind server to port and IP
server.bind(PORT, HOST);

