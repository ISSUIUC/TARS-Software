import { ipcRenderer } from "electron";

const led_status = document.getElementById("led_status");
const led_button = document.getElementById("Blink");
led_status.innerHTML = "0";
document.body.appendChild(led_status);

let led = false;

// led_button.addEventListener("click", ()=>{
ipcRenderer.on("serial",(event, message)=>{
    led_status.innerHTML = message;
    // led = !led;
    // const text = led ? "ON" : "OFF";
    // led_status.innerHTML = text;
    // // port.write(text);
});