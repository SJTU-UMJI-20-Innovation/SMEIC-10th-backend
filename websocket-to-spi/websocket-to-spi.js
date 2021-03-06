const SPI = require('pi-spi');
const WebSocketServer = require('ws').Server;
const PORT = 2051;

const MEGA = 0;
const UNO = 1;
const spiMEGA = SPI.initialize('/dev/spidev0.0');
spiMEGA.clockSpeed(1e6);
const spiUNO = SPI.initialize('/dev/spidev0.1');
spiUNO.clockSpeed(1e6);

const wsServer = new WebSocketServer({port: PORT});
wsServer.on('connection', (socket) => {
    socket.on('message', processMessage);
});

const POLYNOME = 0x7;
const SPI_TIMEOUT = 15;
const ERR_GPIO_MEGA = 17;
const ERR_GPIO_UNO = 27;

const Gpio = require('onoff').Gpio;
const errorPin = [];
errorPin[MEGA] = new Gpio(ERR_GPIO_MEGA, 'in');
errorPin[UNO] = new Gpio(ERR_GPIO_UNO, 'in');
process.on('SIGINT', () => {
    errorPin[MEGA].unexport();
    errorPin[UNO].unexport();
    process.exit();
});

var commandQueue = [];
function send() {
    const command = commandQueue[0];
    sendSPI(command.cmd, command.dest);
    setTimeout(() => {errorPin[command.dest].read(check);}, SPI_TIMEOUT);
}

function check(err, value){
    if (err) {
        throw err;
    }

    if (value) {
        console.log("CRC check failed or byte lost. Resending.");
        send();
    } else {
        console.log("Successfully sent.\n");
        commandQueue.shift();
        if (commandQueue.length) {
            send();
        }
    }
}

function getCRC(cmd){
    var crc = 0;
    for (var i = 0; i < cmd.length; ++i){
        crc ^= cmd[i];
        for (var j = 8; j; --j){
            if (crc & (1 << 7)){
                crc <<= 1;
                crc ^= POLYNOME;
            } else {
                crc <<= 1;
            }
        }
        crc = crc & 0xFF;
    }
    return crc;
}

function processMessage(msg) {
    var message = msg.split(" ");
    var dest;
    var cmd;
    switch (message[0]) {
        case "stop":
            dest = MEGA;
            cmd = Buffer.from([0b00001000, 0, 0, 0, 0]);
            break;
        case "cgSg": case "changeSignal":
            dest = MEGA;
            cmd = Buffer.from([0b00011001, message[1], 0, 0 ,0]);
            break;
        // Following commands use this protocol:
        // 0-4| command ID
        // 5-7| parameter bytes required (obsolete)
        // 8| flag (?delay/?left)
        // 9-10| num
        // 11-23| float stored as (int)(float * 16 + 4096)
        // 24-31| 8bit signal1
        // 32-39| 8bit signal2
        // 00000111 23344444 44444444 55555555 6666666
        // byte0 = commandID << 5 + bytesRequired
        // encodedFloat = (int)(float * 16 + 4096) & 0x1FFF
        // byte1 = flag << 15 + num << 13 + encodedFloat >> 8
        // byte2 = encodedFloat & 0xFF
        // byte3 = signal1
        // byte4 = signal2
        case "rtRtBs": case "rotateRotationBase":
            dest = MEGA;
            cmd = Buffer.from([0b00100100, ((((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)) >> 8), ((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF), message[3], message[4]]);
            break;
        case "dlRtBs": case "delayRotationBase":
            dest = MEGA;
            cmd = Buffer.from([0b00100100, (0x8000 + ((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)) >> 8, (0x8000 + ((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)), message[3], message[4]]);
            break;
        case "lfLfBs": case "liftLiftBase":
            dest = MEGA;
            cmd = Buffer.from([0b00101100, (((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)) >> 8, (((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)), message[3], message[4]]);
            break;
        case "dlLfBs": case "delayLiftBase":
            dest = MEGA;
            cmd = Buffer.from([0b00101100, (0x8000 + ((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)) >> 8, (0x8000 + ((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)), message[3], message[4]]);
            break;
        case "mvMvBs": case "moveMovingBase":
            dest = MEGA;
            cmd = Buffer.from([0b00110100, (((message[1] * 16 + 4096) & 0x1FFF)) >> 8, (((message[1] * 16 + 4096) & 0x1FFF)), message[2], message[3]]);
            break;
        case "dlMvBs": case "delayMovingBase":
            dest = MEGA;
            cmd = Buffer.from([0b00110100, (0x8000 + ((message[1] * 16 + 4096) & 0x1FFF)) >> 8, (0x8000 + ((message[1] * 16 + 4096) & 0x1FFF)), message[2], message[3]]);
            break;
        case "mvAm": case "mvMvAm": case "moveArm":
            dest = MEGA;
            cmd = Buffer.from([0b00111100, (((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)) >> 8, (((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)), message[3], message[4]]);
            break;
        case "dlAm": case "dlMvAm": case "delayMoveArm":
            dest = MEGA;
            cmd = Buffer.from([0b00111100, (0x8000 + ((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)) >> 8, (0x8000 + ((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)), message[3], message[4]]);
            break;
        case "rtSvOne": case "rotateServoOne":
            dest = MEGA;
            cmd = Buffer.from([0b01000100, (((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)) >> 8, (((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)), message[3], message[4]]);
            break;
        case "rtSvTwo": case "rotateServoTwo":
            dest = MEGA;
            cmd = Buffer.from([0b01000100, (0x8000 + ((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)) >> 8, (0x8000 + ((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)), message[3], message[4]]);
            break;
        case "cgMs": case "changeMass":
            dest = MEGA;
            cmd = Buffer.from([0b01001100, (((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)) >> 8, (((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)), message[3], message[4]]);
            break;
        case "gtLq": case "getLiquid":
            dest = UNO;
            cmd = Buffer.from([0b01010100, (((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)) >> 8, (((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)), message[3], message[4]]);
            break;
        case "mvCm": case "moveCamera":
            dest = MEGA;
            cmd = Buffer.from([0b01011100, (((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)) >> 8, (((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)), message[3], message[4]]);
            break;
        case "gtSl": case "getSolid":
            dest = UNO;
            cmd = Buffer.from([0b01100100, (((message[1] * 16 + 4096) & 0x1FFF)) >> 8, (((message[1] * 16 + 4096) & 0x1FFF)), message[2], message[3]]);
            break;
        case "dtSvThree": case "deltaServoThree":
            dest = MEGA;
            cmd = Buffer.from([0b01101100, (0x8000 + ((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)) >> 8, (0x8000 + ((message[1] & 3) << 13) + ((message[2] * 16 + 4096) & 0x1FFF)), message[3], message[4]]);
            break;
        case "changeBurette": case"cgBr":
            dest = UNO;
            cmd = Buffer.from([0b01110100, (((message[1] * 16 + 4096) & 0x1FFF)) >> 8, (((message[1] * 16 + 4096) & 0x1FFF)), message[2], message[3]]);
            break;
        default:
            console.error("Unknown Command: ", msg);
    }
    if (cmd) {
        cmd = Buffer.concat([cmd, Buffer.from([getCRC(cmd)])]);
        commandQueue.push({cmd: cmd, dest: dest});
        if (commandQueue.length === 1) {
            send();
        }
    }
}

function sendSPI(buffer, dest) {
    if (dest === MEGA) {
        spiMEGA.write(buffer, () => {console.log("Sent:", buffer, "to MEGA.")});
    }
    if (dest === UNO) {
        spiUNO.write(buffer, () => {console.log("Sent:", buffer, "to UNO.")});
    }
}
