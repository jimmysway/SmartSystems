// Modules
var dgram = require('dgram'); // Socket programming
const { DateTime } = require('luxon'); // Get the current time of day
var fs = require('fs'); // To save to a file
let now = DateTime.now();

// Port and IP
var PORT = 3333; // Initialize a port
var HOST = '192.168.1.36'; // Ip address of pi
// var HOST = '10.239.114.40'; // Ip address of pi

// Clear all CSV files
const directoryPath = './';
fs.readdir(directoryPath, (err, files) => {
    if (err) {
        throw err;
    }

    // Filter the files with the .csv extension
    const csvFiles = files.filter(file => file.endsWith('.csv'));

    // Delete each of the filtered .csv files
    csvFiles.forEach(file => {
        fs.unlink(`${directoryPath}${file}`, err => {
            if (err) {
                throw err;
            }
        });
    });
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
server.on('message', function (message, remote) {
  let carminData = remote.address + ':' + remote.port + "-" + message; // Later parse message by "," to get the sensor contents
  console.log(carminData);

  // ------ Add some leaderboard/parsing logic ------
  // Parser
  let data = message.toString();
  data = data.split(",");
  stepsArr.push(parseInt(data[0])); // Push the new steps recieved into array of steps
  tempArr.push(parseFloat(data[1])); // Push new temps into array

  // Get time
  now = DateTime.now();
  let currentTime = now.toFormat('HH:mm:ss');
  let currentTimeParsed = currentTime.toString().split(':');
  let totalTime = currentTimeParsed[0] + currentTimeParsed[1];

  if (!fs.existsSync('port' + remote.port + '.csv')) {
    // If the file doesn't exist, create it and write the header row
    fs.writeFileSync('port' + remote.port + '.csv', 'Time,Step,Temp\n', function() {
        console.log('Created a file!');
    });
  }

  // Save carmin watch data to CSV in format IPaddress:Port-Sensor,Sensor
  fs.appendFile('port' + remote.port + '.csv', currentTime + ',' + message, function (err) {
    if (err) throw err;
  });

  // Send leaderboard information
  server.send(totalTime, remote.port, remote.address, function (error) {
      if (error) {
          console.log('MEH!');
      } else {
          console.log('Sent: ', totalTime);
      }
  });
});

// Bind server to port and IP
server.bind(PORT, HOST);