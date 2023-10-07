const {SerialPort} = require('serialport');
const { DateTime } = require('luxon');

const port = new SerialPort({ path: '/dev/cu.usbserial-0264FEBE', baudRate: 115200 });

let now = DateTime.now();
let currentTime = now.toFormat('HH:mm:ss');

const parsedTime = currentTime.split(":");
const dataToSend = parsedTime[0] + parsedTime[1];

port.write(dataToSend, (err) => {
    if (err) {
        console.error('Error writing to serial port:', err);
    } else {
        console.log(`sent: ${dataToSend}`);

        // Add a delay before reading from the serial port
        setTimeout(() => {
            readFromPort();
        }, 1000);  // For example, delay of 1 second
    }
});

function readFromPort() {
    port.once('data', (data) => {
        console.log(`Received: ${data.toString()}`);
    });
}
