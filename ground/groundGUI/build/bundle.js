/******/ (() => { // webpackBootstrap
/******/ 	"use strict";
var __webpack_exports__ = {};
// This entry need to be wrapped in an IIFE because it uses a non-standard name for the exports (exports).
(() => {
var exports = __webpack_exports__;
/*!*********************!*\
  !*** ./src/main.ts ***!
  \*********************/

Object.defineProperty(exports, "__esModule", ({ value: true }));
var altitude_box = document.getElementById("Altitude");
var liftoff_button = document.getElementById("Liftoff");
altitude_box.innerHTML = "0";
document.body.appendChild(altitude_box);
var time = 0;
liftoff_button.addEventListener("click", function () {
    setInterval(function () {
        time += 0.016;
        var altitude = (100 * (time - time * time / 10)) | 0;
        altitude_box.innerHTML = "Altitude is really ".concat(altitude);
    }, 16);
});

})();

/******/ })()
;
//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiYnVuZGxlLmpzIiwibWFwcGluZ3MiOiI7Ozs7Ozs7Ozs7O0FBRUEsSUFBTSxZQUFZLEdBQUcsUUFBUSxDQUFDLGNBQWMsQ0FBQyxVQUFVLENBQUMsQ0FBQztBQUN6RCxJQUFNLGNBQWMsR0FBRyxRQUFRLENBQUMsY0FBYyxDQUFDLFNBQVMsQ0FBQyxDQUFDO0FBQzFELFlBQVksQ0FBQyxTQUFTLEdBQUcsR0FBRyxDQUFDO0FBQzdCLFFBQVEsQ0FBQyxJQUFJLENBQUMsV0FBVyxDQUFDLFlBQVksQ0FBQyxDQUFDO0FBR3hDLElBQUksSUFBSSxHQUFHLENBQUM7QUFFWixjQUFjLENBQUMsZ0JBQWdCLENBQUMsT0FBTyxFQUFFO0lBQ3JDLFdBQVcsQ0FBQztRQUNSLElBQUksSUFBSSxLQUFLLENBQUM7UUFDZCxJQUFNLFFBQVEsR0FBRyxDQUFDLEdBQUcsR0FBRyxDQUFDLElBQUksR0FBRyxJQUFJLEdBQUcsSUFBSSxHQUFHLEVBQUUsQ0FBQyxDQUFDLEdBQUcsQ0FBQyxDQUFDO1FBQ3ZELFlBQVksQ0FBQyxTQUFTLEdBQUcsNkJBQXNCLFFBQVEsQ0FBRSxDQUFDO0lBQzlELENBQUMsRUFBRSxFQUFFLENBQUMsQ0FBQztBQUNYLENBQUMsQ0FBQyxDQUFDIiwic291cmNlcyI6WyJ3ZWJwYWNrOi8vSVNTLWdyb3VuZC1zdGF0aW9uLy4vc3JjL21haW4udHMiXSwic291cmNlc0NvbnRlbnQiOlsiaW1wb3J0IHsgUmVhZFN0cmVhbSB9IGZyb20gXCJvcmlnaW5hbC1mc1wiO1xuXG5jb25zdCBhbHRpdHVkZV9ib3ggPSBkb2N1bWVudC5nZXRFbGVtZW50QnlJZChcIkFsdGl0dWRlXCIpO1xuY29uc3QgbGlmdG9mZl9idXR0b24gPSBkb2N1bWVudC5nZXRFbGVtZW50QnlJZChcIkxpZnRvZmZcIik7XG5hbHRpdHVkZV9ib3guaW5uZXJIVE1MID0gXCIwXCI7XG5kb2N1bWVudC5ib2R5LmFwcGVuZENoaWxkKGFsdGl0dWRlX2JveCk7XG5cblxubGV0IHRpbWUgPSAwXG5cbmxpZnRvZmZfYnV0dG9uLmFkZEV2ZW50TGlzdGVuZXIoXCJjbGlja1wiLCAoKT0+e1xuICAgIHNldEludGVydmFsKCgpPT57XG4gICAgICAgIHRpbWUgKz0gMC4wMTY7XG4gICAgICAgIGNvbnN0IGFsdGl0dWRlID0gKDEwMCAqICh0aW1lIC0gdGltZSAqIHRpbWUgLyAxMCkpIHwgMDtcbiAgICAgICAgYWx0aXR1ZGVfYm94LmlubmVySFRNTCA9IGBBbHRpdHVkZSBpcyByZWFsbHkgJHthbHRpdHVkZX1gO1xuICAgIH0sIDE2KTtcbn0pOyJdLCJuYW1lcyI6W10sInNvdXJjZVJvb3QiOiIifQ==