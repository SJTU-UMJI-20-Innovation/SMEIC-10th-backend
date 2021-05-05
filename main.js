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
        this.wsToRemote.on('message', this.incoming.bind(this));
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

class CommandServer {
    constructor(destAddr, localPort) {
        this.wsServer = new WebSocket.Server({port: localPort});
        this.wsServer.on('connection', ((socket) => {
            socket.on('message', this.incoming.bind(this));
        }).bind(this));

        this.wsToDest = new WebSocket('ws://' + destAddr);
    }

    incoming(msg) {
        this.wsToDest.send(msg);
    }
}

const localCameraServerProcess = spawn('node', ['./camera-server/live-camera.js']);
localCameraServerProcess.stdout.on('data', (data) => {
    if (data.toString() === "Server Ready.\n") {
        new CameraRelay('localhost:8080', 2050);
    }
});

new CameraRelay('192.168.1.3:8080', 2052);

new CommandServer('192.168.1.3:2051', 2051);
