const express = require('express');
const mqtt = require('mqtt');
const socketIo = require('socket.io');
const http = require('http');

const app = express();
const server = http.createServer(app);
const io = socketIo(server);
const mqttClient = mqtt.connect('mqtt://localhost'); // Replace with your MQTT broker

app.use(express.static('public'));

// MQTT Topics
const stopTopic = 'webUI/stop'; // Topic for start/stop commands
const targetTopic = 'webUI/target'; // Topic for sending target position
const moveTopic = 'webUI/move'; // Topic for sending target position
const mapTopic = 'robot/map'; // Topic for receiving map
const modeTopic = 'robot/mode'; // Topic for receiving robot mode
const stateTopic = 'robot/state'; // Topic for receiving robot state
const bumperTopic = 'robot/bumper'; // Topic for receiving bumper state
const cliffTopic = 'robot/cliff'; // Topic for receiving cliff state
const coordinatesTopic = 'robot/coordinates'; // Topic for receiving robot coordinates

mqttClient.on('connect', () => {
    console.log('Connected to MQTT broker');
    mqttClient.subscribe(coordinatesTopic, (err) => {
        if (!err) {
        console.log('Subscribed to "robot/coordinates"');
        }
    });
    mqttClient.subscribe(mapTopic, (err) => {
        if (!err) {
        console.log('Subscribed to "robot/map"');
        }
    });
    mqttClient.subscribe(modeTopic, (err) => {
        if (!err) {
        console.log('Subscribed to "robot/mode"');
        }
    });
    mqttClient.subscribe(stateTopic, (err) => {
        if (!err) {
        console.log('Subscribed to "robot/state"');
        }
    });
    mqttClient.subscribe(bumperTopic, (err) => {
        if (!err) {
        console.log('Subscribed to "robot/bumper"');
        }
    });
    mqttClient.subscribe(cliffTopic, (err) => {
        if (!err) {
        console.log('Subscribed to "robot/cliff"');
        }
    });
});

mqttClient.on('message', (topic, message) => {
    if (topic === coordinatesTopic) {
        console.log('Received "robot/coordinates"', message.toString());
        const positionData = JSON.parse(message.toString()); // Assuming the message is a JSON string
        io.emit('robot coordinates', positionData); // Emit to all connected clients
    } else if (topic === mapTopic) {
        const data = JSON.parse(message.toString());
        console.log('Received "robot/map"');
        io.emit('map update', data); // Emit to all connected clients
    } else if (topic === modeTopic) {
        console.log('Received "robot/mode"', message.toString());
        const data = JSON.parse(message.toString());
        io.emit('robot mode', data); // Emit to all connected clients
    } else if (topic === stateTopic) {
        console.log('Received "robot/state"', message.toString());
        const data = JSON.parse(message.toString());
        io.emit('moving state', data); // Emit to all connected clients
    } else if (topic === bumperTopic) {
        console.log('Received "bumperTopic"');
        // Parse bumper message
        const data = JSON.parse(message.toString());
        io.emit('bumper state', data);
    } else if (topic === cliffTopic) {
        console.log('Received "cliffTopic"');
        // Parse cliff message
        const data = JSON.parse(message.toString());
        io.emit('cliff state', data);
    }
});

// Socket.IO events
io.on('connection', (socket) => {
    console.log('A client connected');

    // Handle 'start command' from clients
    socket.on('start command', (data) => {
        const message = JSON.stringify(data);
        mqttClient.publish(targetTopic, message);
        console.log(`Publishing start command to ${targetTopic}:`, message);
    });

    // Handle 'move command' from clients
    socket.on('move command', (data) => {
        const message = JSON.stringify(data);
        mqttClient.publish(moveTopic, message);
        console.log(`Publishing move command to ${moveTopic}:`, message);
    });

    // Handle 'stop command' from clients
    socket.on('stop command', (data) => {
        const message = JSON.stringify(data);
        mqttClient.publish(stopTopic, message);
        console.log(`Publishing stop command to ${stopTopic}`);
    });
});

io.on('error', function (err) {
    console.error('Error:', err);
});

server.listen(3000, () => {
  console.log('Server is running on http://localhost:3000');
});
