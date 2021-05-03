const Broadcaster = require('./raspivid-broadcaster');
new Broadcaster({width: 960, height: 720, framerate: 30, port: 8080});
