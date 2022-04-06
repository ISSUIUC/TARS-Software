"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
Object.defineProperty(exports, "__esModule", { value: true });
const electron_1 = require("electron");
const SerialPort = require("serialport");
const path = require("path");
electron_1.app.on('ready', () => {
    console.log('App is ready');
    const win = new electron_1.BrowserWindow({
        width: 600,
        height: 400,
        webPreferences: {
            nodeIntegration: true,
            nodeIntegrationInWorker: true,
            contextIsolation: false,
        }
    });
    const indexHTML = path.join(__dirname + '/index.html');
    win.loadFile(indexHTML).then(() => {
        serial_communicate(win);
    }).catch(e => console.error(e));
});
function serial_communicate(window) {
    setInterval(() => __awaiter(this, void 0, void 0, function* () {
        const ports = yield SerialPort.list();
        window.webContents.send("serial", JSON.stringify(ports));
    }), 1000);
    // const port = new SerialPort("COM3");
    // const msg = port.read(128);
    // window.webContents.send("serial", msg);
}
//# sourceMappingURL=index.js.map