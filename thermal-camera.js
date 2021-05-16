const { spawn } = require('child_process');
const WebSocketServer = require('ws').Server;

const wsServer = new WebSocketServer({port: 2053});
const thermalCamera = spawn('python3', ['./thermal-camera.py']);
thermalCamera.stdout.on('data', (data) => {
    wsServer.clients.forEach((socket) => {
        socket.send(JSON.stringify(JSON.parse(data.toString()).data));
    });
});
