const dgram = require('dgram');
const express = require('express');
const app = express();
const client = dgram.createSocket('udp4');

const ESP32_SERVER_IP = 'group7.ddns.net'; // Using DDNS
const ESP32_UDP_PORT = 8080;

app.use(express.json()); // Middleware to parse JSON
app.use(express.static('public'));


client.on('error', (err) => {
    console.error('Socket error:', err);
    client.close();
});

app.post('/control', (req, res) => {
    const message = String(req.body.command);
    console.log(`Preparing to send message: ${message}`);

    client.send(message, 0, message.length, ESP32_UDP_PORT, ESP32_SERVER_IP, (err) => {
        if (err) {
            console.error("Error sending message:", err);
            res.status(500).send('Error sending message to ESP32');
        } else {
            console.log("Message sent:", message);
            res.send('Message sent to ESP32');
        }
    });
});

client.on('listening', () => {
  const address = client.address();
  console.log(`UDP Client listening on ${address.address}:${address.port}`);
});

app.listen(3000, () => {
  console.log('HTTP server is listening on port 3000');
});

client.bind();  // Bind the client to start listening for input
