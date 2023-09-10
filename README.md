# TARS Flight Software Repository
Illinois Space Society's flight software codebase for the TARS system.

<div align="center">
<a href="https://www.youtube.com/watch?v=OQC60KljR3A"><img src="https://img.youtube.com/vi/OQC60KljR3A/0.jpg"></img></a><br>
<i>Click to see a cool video!</i>
</div>

### Directory Structure:
- `TARS/`: Mission critical flight software running on TARS. This is the code that actually flies on the rocket
	- `src/`: All the flight code that we write ourselves is in this directory. 
		- `common/`: Utility code we write that is used across microcontrollers.
		- `mcu_main/`: Code for the primary microcontroller on TARS (Teensy 4.1)
		- `mcu_telemetry/`: Code for the microcontroller in charge of telemetry and GPS (ESP32-S3)
		- `mcu_power`: Code for the microcontroller on the power board (ATMega328P)
	- `lib/`: Third-party libraries that are not available on the PlatformIO Registry. Other libraries are included via the `lib_deps` build flag in `platformio.ini`
- `ground/`: Code running on ground station hardware (Adafruit LoRa Feather)

### Branch Naming Convention
Please use the following naming conventions when creating branches while developing:

Your `<branch-name>` should consist of the Trello ticket ID and a short description of the work being done. For example:

`AV-420-write-cp-location-interpolation-function`

Then use the following scheme to then organize your branches:

- `<branch-name>` for small and simple contributions pertaining to a ticket
- `user/<github-username>/<branch-name>` for individual tasks or contributions, or as a sandbox for yourself
- `feature/<branch-name>` for **new** functionality that didn't exist before
- `bug/<branch-name>` for bug fixes
- `general/<branch-name>` for overall repository organization or development pipeline tweaks
- `misc/<branch-name>` or `junk/<branch-name>` for just messing around :)

Some fictional examples:\
`user/AyberkY/improve-lsm9ds1-spi-latency`\
`feature/create-mcu-state-estimation-thread`\
`bug/gps-thread-deadlock-fix`\
`general/create-new-directory-for-gnc`\
`misc/testing-sd-card-bandwidth`

Please include the Trello ticket ID when relevant! i.e. for a ticket [AV-69] your branch might look like:

`user/AyberkY/AV-69-graceful-failure-for-fifo-buffer`
or
`feature/AV-69-create-data-logger-class`


### Archiving Code Base Revisions
We should try to keep an archive of the version of code that ran on each rocket launch, so that we can associate the data we collected with the code that was running on TARS.

After every launch or major milestone, create branch with the following convention:\
`archive/<month.day.year>-<milestone-or-launch-description>`

For example: `archive/09.01.21-start-of-2022-school-year`

### Code Style Guide
The repository now has a GitHub Actions instance that will automatically check for code style violations!

The Actions instance **will not** inhibit a pull-request from merging. It is merely there to _encourage_ style consistency throughout our code base.

There is also an auto formatting script that will _format your code for you_! (its beautiful, you should use it) This means that you don't have to worry about coding to meet the style yourself, as you can simply run the formatting script before you commit/push your changes.

You can run the script on Linux, Mac, or WSL like so:
```
./run_clang_format.py -i -r .
```

Things to keep in mind about code formatting:
- The code style being used is defined in `.clang-format`. It currently follows Google's C++ style guide exactly. However, a modification was made so that the maximum characters per line was increased from 80 to 120. 
- Third party libraries (e.g.`ChRt` which is ChibiOS) are exempted from code-style checks and the auto formatting script. This is to avoid any possibility of breaking proven/working libraries. If adding a new library to flight code, make sure to update `.clang-format-ignore` with the relevant file path if it isn't in `lib`.
  - Exempt directories should be listed in `.clang-format-ignore` so they don't get auto-formatted by the script.
  - Exempt directories should also be listed in `.github/workflows/clang_format_check.yml` on the lines with `exclude:`, so they're not checked for style violations by GitHub. 
- Changing/tweaking the style guide is always option! If you have ideas, reach out!

 - You can copy the file tools/pre-commit to .git/hooks with the command `cp tools/pre-commit .git/hooks/pre-commit` to automatically run clang tidy before committing.
