# Venus Rover

Authors: Jake Lee, Jason Li , JiaLin Sui, Maxim Slobodchikov

Date: 2023-11-10

### Summary
Our rover buggy is a robust platform for autonomous driving that makes the round trip between a spaceship and the hot springs of Venus. We implemented (1) "cruise control" (or maintaining a constant velocity under perturbations), (2) "turn-around" (reversing the direction of the vehicle), and (3) "collision avoidance" by detecting obstructions and driving around them.

The approach involved (a) attaching sensors and the ESP to your vehicle, (b) enabling control using feedback from the wheel speed sensor to maintain a speed setpoint (driving the vehicle motor), (c) using range sensors to detect and avoid objects, and (d) maintaining forward progress towards each waypoint (A and B)

The key features involved:
- Successfully makes A--B--A trip in reasonable time window
- No collisions with obstructions
- Start and stop instructions should be issued wirelessly through wireless control
- Can use automatic or manual control at turnaround (you can implement a turning algorithm or do this with remote control of steering)
- Displays elapsed time on alpha display
- Constant speed except when doing turns or collision avoidance

### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Uses PID for speed control holding a fixed speed setpoint after startup and before slowdown |  |  1     | 
| Stops within 20 cm of end without collision |  |  1     | 
| Start and stop instructions issued wirelessly from phone, laptop or ESP |  |  1     | 
| Measures wheel speed |  |  1     | 
| Uses alpha display to show elapsed time |  |  1     | 
| Successfully traverses A-B in one go, no hits or nudges |  |  1     | 
| Successfully reverses direction (auto or remote control), no hits or nudges |  |  1     | 
| Successfully traverses B-A in one go, no hits or nudges |  |  1     | 
| No collisions with obstructions |  |  1     | 

#### Wireless Interface
##### Startup Instructions
To start server, we can cd into the Website folder and run "node server.js" in console. The site interface can then seen on http://localhost:3000/. One must remember to npm install express and node-fetch and ESP32 device needs to have a UDP server listening on port 8080 to receive messages "start", "stop", "turn".

##### Destination IP Address (ESP32_SERVER_IP): 
The messages are sent to the IP address specified by ESP32_SERVER_IP, which in your case is set to 'group7.ddns.net'. This is the dynamic DNS (DDNS) address that should resolve to the IP address of your network where the ESP32 device is located.

##### Destination UDP Port (ESP32_UDP_PORT): 
The messages are sent to the port number specified by ESP32_UDP_PORT, which is 8080 in your code. This means that your ESP32 device needs to have a UDP server listening on port 8080 to receive these messages.

##### Handling in the ESP32: 
The ESP32 should be programmed to run a UDP server that listens on the specified port (8080). When it receives a message on this port, it should interpret and act upon the message accordingly.

##### Process Flow:
When a user interacts with the buttons on the web interface served from the public directory, an HTTP POST request is sent to the /control endpoint on your Node.js server.
The server then extracts the command from the request body (req.body.command).
This command is converted to a string (String(req.body.command)) and sent as a UDP message to the ESP32 device at the specified DDNS address and port.

##### Network Configuration:
For the ESP32 to receive these messages, it must be connected to a network that is accessible via the group7.ddns.net address.
If the ESP32 is behind a router (which is usually the case), you'll need to set up port forwarding on your router to forward UDP traffic on port 8080 to the internal IP address of the ESP32 device.

##### Security Considerations:
Since your server sends commands to an ESP32 device over the internet, it's crucial to consider security implications, especially if the commands control a physical device.
Make sure to secure the communication and possibly authenticate the requests to prevent unauthorized access or control.

### Sketches/Diagrams


### Supporting Artifacts

### Modules, Tools, Source Used Including Attribution
- [Recipe for Wiring ESP32 to Buggy](/docs/briefs/recipes/recipe-buggy-interfacing.md)
- [Recipe for Calibrating ESC and Steering Servo -- Buggy](/docs/briefs/recipes/recipe-esc-buggy.md)
- [Recipe for using Multiple LIDAR-Lite V4s on same bus](/docs/briefs/recipes/recipe-lidarlite-v4.md)
- [PWM Design Pattern](/docs/briefs/design-patterns/dp-pwm.md)
- [PID For Wall Tracking and Speed Control Design Pattern](/docs/briefs/design-patterns/dp-pid.md)
