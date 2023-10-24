const dgram = require('dgram');
const client = dgram.createSocket('udp4');

const ESP32_SERVER_IP = '192.168.1.33';
const ESP32_UDP_PORT = 8080;

client.on('error', (err) => {
    console.error('Socket error:', err);
    client.close();
});

let ledState = false;

function sendMessage() {
    const message = ledState ? "TURN_ON" : "TURN_OFF";
    console.log("Preparing to send message:", message);

    client.send(message, 0, message.length, ESP32_UDP_PORT, ESP32_SERVER_IP, (err) => {
        if (err) {
            console.error("Error sending message:", err);
        } else {
            console.log("Message sent:", message);
            ledState = !ledState;
            setTimeout(sendMessage, 2000);  // Schedule the next message send after a 2-second delay
        }
    });
}

client.bind(sendMessage);  // Start the first message send once the client is bound
