const Broadcaster = require('./raspivid-broadcaster');
new Broadcaster({width: 1920, height: 1080, framerate: 24, port: 8080});
