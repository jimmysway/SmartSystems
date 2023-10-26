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

const csvFilePath1 = '../serverJS/port59033.csv';
const csvFilePath2 = '../serverJS/port64333.csv';

function timeStringToDate(timeStr) {
    const [hours, minutes, seconds] = timeStr.split(":").map(Number);
    const date = new Date();
    date.setHours(hours, minutes, seconds, 0);
    return date;
}

let largestStepsFile = null;  // This will keep track of the file with the largest number of steps
let totalStepsMap = {
    'carmin0.csv': 0,
    'carmin1.csv': 0
};

function updateLargestStepsFile(filePath, totalSteps) {
    totalStepsMap[filePath] = totalSteps;

    if (largestStepsFile === null || totalStepsMap[filePath] > totalStepsMap[largestStepsFile]) {
        largestStepsFile = filePath;
    }
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
            
            updateLargestStepsFile(csvFilePath, totalSteps);
            console.log(`File with the largest number of steps is: ${largestStepsFile}`);
        }
    });
}

// Watch both CSV files and emit data
watchAndEmitData(csvFilePath1, 'data0');
watchAndEmitData(csvFilePath2, 'data1');

server.listen(3000, () => {
    console.log('listening on *:3000');
});
