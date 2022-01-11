import { app, BrowserWindow } from 'electron';
import * as SerialPort from "serialport"
import * as path from 'path';

app.on('ready', () => {
    console.log('App is ready');
    const win = new BrowserWindow({
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


function serial_communicate(window: BrowserWindow){
    setInterval(async ()=>{
        const ports = await SerialPort.list();
        window.webContents.send("serial", JSON.stringify(ports));
    }, 1000);
    // const port = new SerialPort("COM3");
    // const msg = port.read(128);
    // window.webContents.send("serial", msg);
}