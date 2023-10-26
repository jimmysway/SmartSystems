const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const fs = require('fs');
const Papa = require('papaparse');

const app = express();
const server = http.createServer(app);
const io = socketIo(server);

app.use(express.static(__dirname));  // Serve static files

function getDataFromCSV(filePath) {
    const csvContent = fs.readFileSync(filePath, 'utf-8');
    const parsed = Papa.parse(csvContent, { header: true });
    const data = parsed.data;

    const newEntry = data[data.length - 2];
    const totalSteps = data.reduce((accum, curr) => {
        if (curr.Step) {  // Ensure that the current row has a Step value
            return accum + parseFloat(curr.Step);
        }
        return accum;
    }, 0);

    return { newEntry, totalSteps };
}

const csvFilePath1 = './carmin0.csv';
const csvFilePath2 = './carmin1.csv';

function timeStringToDate(timeStr) {
    const [hours, minutes, seconds] = timeStr.split(":").map(Number);
    const date = new Date();
    date.setHours(hours, minutes, seconds, 0);
    return date;
}

function watchAndEmitData(csvFilePath, eventName) {
    fs.watch(csvFilePath, (eventType, filename) => {
        if (eventType === 'change') {
            const {newEntry, totalSteps} = getDataFromCSV(csvFilePath);
            const parsedTime = timeStringToDate(newEntry.Time);
            
            io.emit(eventName, {
                x: parsedTime,
                y: parseFloat(newEntry.Step),
                temp: parseFloat(newEntry.Temp),
                totalSteps : totalSteps
            });
        }
    });
}

// Watch both CSV files and emit data
watchAndEmitData(csvFilePath1, 'data0');
watchAndEmitData(csvFilePath2, 'data1');

server.listen(3000, () => {
    console.log('listening on *:3000');
});
