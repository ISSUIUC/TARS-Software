import * as SerialPort from "serialport";

const led_status = document.getElementById("led_status");
const led_button = document.getElementById("Blink");
led_status.innerHTML = "0";
document.body.appendChild(led_status);

let port = new SerialPort("idk");

let led = false;

led_button.addEventListener("click", ()=>{
    led = !led;
    const text = led ? "ON" : "OFF";
    led_status.innerHTML = text;
    port.write(text);
});