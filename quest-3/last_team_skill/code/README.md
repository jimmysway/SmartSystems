# ESP32 UDP Communication Project

This project establishes a UDP communication link between an ESP32 device and a server running on a laptop. The ESP32 sends messages to the server, which acknowledges them and responds.

- ESP-IDF environment set up.
- Node.js installed for the server-side application.

## ESP32 Client Side Setup (C)

### Files

- main.c: Contains the main code for ESP as a server
- client.c: Contains code for ESP as a client
- server.js: Contains code for js as server
- client.js: Contains code for js as client


Defines WiFi credentials (WIFI_SSID, WIFI_PASSWORD) and the server's IP and port (UDP_SERVER_IP, UDP_PORT). Whenever there was a connection between server and client there would be a confirmation message. Which ever system acts as the server or client will check for a certain message that was passed i.e. TURN_ON, TURN_OFF. Whenever this message is heard it will trigger the an event i.e. the LED turning on or turning off. For the blink task it was simply alternating between two different blink times every time it sent data to show the functionality, in this case it just listened for a string of numbers.

