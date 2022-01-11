import { ReadStream } from "original-fs";

const altitude_box = document.getElementById("Altitude");
const liftoff_button = document.getElementById("Liftoff");
altitude_box.innerHTML = "0";
document.body.appendChild(altitude_box);


let time = 0

liftoff_button.addEventListener("click", ()=>{
    setInterval(()=>{
        time += 0.016;
        const altitude = (100 * (time - time * time / 10)) | 0;
        altitude_box.innerHTML = `Altitude is really ${altitude}`;
    }, 16);
});