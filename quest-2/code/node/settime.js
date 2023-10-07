const {SerialPort} = require('serialport');
const { DateTime } = require('luxon');

const port = new SerialPort({ path: '/dev/cu.usbserial-0264FEBE', baudRate: 115200 });

// add
const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const fs = require('fs');
const Papa = require('papaparse');
const app = express();
const server = http.createServer(app);
const io = socketIo(server);

const {ReadlineParser} = require('@serialport/parser-readline');

const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));
const textFileStream = fs.createWriteStream('data.csv', { flags: 'a' });
//

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

port.on("open", () => {
    textFileStream.write('Time,Step,Temp\n');
    });
    parser.on('data', data =>{
        let content;
        now = DateTime.now();
        currentTime = now.toFormat('HH:mm:ss');
        content = currentTime + "," + data + '\n';
        console.log(data);
        textFileStream.write(content);
    });

    function readFromPort() {
    port.once('data', (data) => {
        console.log(`Received: ${data.toString()}`);
    });
}
