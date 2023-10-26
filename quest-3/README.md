# Straba Social Media Hub

Authors: Jake Lee, Jason Li , JiaLin Sui, Maxim Slobodchikov

Date: 2023-10-26

### Summary
The Straba Media Hub is portal for hosting data originating from multiple Carmins. The project iterated from our previous Carmin watch by collecting data from many individual smart watches, sourcing and passing data across a wireless network/routers in a bidirectional way, making data available from any browser, and aggregating data and displaying leaders for specific metrics.

The key features involved:
- Carmins connecting via WiFi
- Collecting data from multiple Carmins to central server and presenting data as web portal
- Central server reports leader status back to each Carmin alpha display
- Central server runs on node.js, enables portal, and is accessible on open Internet
- Webcam sourcing video from pi cam, embedded in a single client browser window

### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Carmins connected via WiFi |  |  1     | 
| Data from each Carmin sent to central server and aggregated |  |  1     | 
| Portal reports live leader status and charts on web site |  |  1     | 
| Central server reports live leader status back to Carmin alpha displays |  |  1     | 
| Portal accessible from open internet |  |  1     | 
| Web cam operational in same browser window at client |  |  1     | 
| Node.js runs on pi |  |  1     | 

### Sketches/Diagrams


### Supporting Artifacts


### Modules, Tools, Source Used Including Attribution
- [WiFi Station Example](https://github.com/espressif/esp-idf/tree/master/examples/wifi/getting_started/station)
- [UDP client](https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_client)
- [Serial Port Module](https://www.npmjs.com/package/serialport)
- [Socket.io Brief](/docs/briefs/design-patterns/dp-socketIO.md)
- [Serial to Node Example](https://github.com/BU-EC444/04-Code-Examples/tree/main/serial-esp-to-node-serialport)
