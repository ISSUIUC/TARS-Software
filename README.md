# TARS Flight Software Repository - Intrepid 3 Archive
Illinois Space Society's flight software codebase for the TARS system.

### Directory Structure:
- `flight/`: Mission critical flight software running on TARS. This is the code that actually flies on the rocket
- `ground/`: Code running on ground station hardware
- `misc/`: Miscellaneous one-off projects for particular tests or R&D
- `utils/`: Tools and utilities that make life easier

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
- The code style being used is defined in `.clang-format`. It currently follows Google's C++ style guide exactly.
- Third party libraries (e.g.`ChRt` which is ChibiOS) are exempted from code-style checks and the auto formatting script. This is to avoid any possibility of breaking proven/working libraries.
  - Exempt directories should be listed in `.clang-format-ignore` so they don't get auto-formatted by the script.
  - Exempt directories should also be listed in `.github/workflows/clang_format_check.yml` on the lines with `exclude:`, so they're not checked for style violations by GitHub. 
- Changing/tweaking the style guide is always option! If you have ideas, reach out!
