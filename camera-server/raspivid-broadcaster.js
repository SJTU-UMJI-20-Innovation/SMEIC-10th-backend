const WebSocketServer = require('ws').Server;

const raspivid = require('raspivid');
const stream = require('stream');
const Splitter = require('stream-split');
const NALSeparator = new Buffer([0, 0, 0, 1]);

var wsServer = null;

var headerFrames = [];
var latestIdrFrame = null;
var videoStream = null;

function createBroadcaster(options) {
    wsServer = new WebSocketServer({port: options.port});
    wsServer.on('connection', (socket) => {
        socket.send(JSON.stringify({
            action: "init",
            width: options.width,
            height: options.height
        }));
        socket.send(Buffer.concat([...headerFrames, latestIdrFrame]), {binary: true});
    });

    const {port, ...raspividOptions} = {...options, profile: 'baseline', timeout: 0};
    videoStream = raspivid(raspividOptions)
        .pipe(new Splitter(NALSeparator))
        .pipe(new stream.Transform({
            transform: function (chunk, _encoding, callback){
                const chunkType = chunk[0] & 0b11111;
                const chunkWithSeparator = Buffer.concat([NALSeparator, chunk]);

                if (chunkType === 7 || chunkType === 8) {
                    headerFrames.push(chunkWithSeparator);
                } else {
                    if (chunkType === 5) {
                        latestIdrFrame = chunkWithSeparator;
                    }
                    this.push(chunkWithSeparator);
                }

                callback();
            }
        }));

    videoStream.on('data', (data) => {
        wsServer.clients.forEach((socket) => {
            socket.send(data, {binary: true});
        });
    });
}

class Broadcaster {
    constructor(options) {
        createBroadcaster(options);
        this.wsServer = wsServer;
        this.headerFrames = headerFrames;
        this.latestIdrFrame = latestIdrFrame;
        this.videoStream = videoStream;
    }
}

module.exports = Broadcaster;
