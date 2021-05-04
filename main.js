const { spawn } = require('child_process');
const WebSocket = require('ws');

class CameraRelay {
    constructor(remoteAddr, localPort) {
        this.wsServer = new WebSocket.Server({port: localPort});
        this.wsServer.on('connection', (socket) => {
            socket.send(this.playerHeader);
            socket.send(this.h264Header, {binary: true});
        });

        this.wsToRemote = new WebSocket('ws://' + remoteAddr);
        this.messageNum = 0;
        this.wsToRemote.on('message', this.incoming);
    }

    send(data) {
        this.wsServer.clients.forEach((socket) => {
            socket.send(data, {binary: true});
        });
    }

    incoming(data) {
        switch(this.messageNum) {
            case 0:
                this.playerHeader = data;
                this.messageNum ++;
                break;
            case 1:
                this.h264Header = data;
                this.messageNum ++;
                break;
            default:
                this.send(data);
        }
    }
}

const localCameraServerProcess = spawn('node', ['./camera-server/live-camera.js']);
localCameraServerProcess.stdout.on('data', (data) => {
    if (data === "Server Ready.") {
        new CameraRelay('localhost:8080', 2050);
    }
});

new CameraRelay('192.168.1.3:8080', 2052);
