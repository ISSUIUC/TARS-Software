# IREC-2020_21
Illinois Space Society's rocket code for IREC 2020-21

### Directory Structure:
- `flight/`: Mission critical flight software running on TARS
- `ground/`: Telemetry and monitoring software running on ground station 
- `misc/`: Miscellaneous one-off projects for particular tests or R&D

### Code Style Guide
The repository now has a GitHub Actions instance that will automatically check for code style violations!

The Actions instance **will not** inhibit a pull-request from merging. It is merely there to _encourage_ style consistency throughout our code base.

There is also an auto formatting script that will _format your code for you_! (its beatiful, you should use it) This means that you don't have to worry about coding to meet the style yourself, as you can simply run the formatting script before you commit/push your changes.

You can run the script on Linux, Mac, or WSL like so:
```
./run_clang_format.py -i -r .
```

Things to keep in mind about code formatting:
- The code style being used is defined in `.clang-format`. It currently follows Google's C++ style guide exactly.
- Third party libraries (e.g.`ChRt` which is ChibiOS) are exempted from code-style checks and the auto formatting script. This is to avoid any possibility of breaking proven/working libraries.
  - Exempt directories should be listed in `.clang-format-ignore` so they don't get auto-formatted by the script.
  - Exempt directories should also be listed in `.github/workflows/clang_format_check.yml` on the lines with `exlcude:`, so they're not checked for style violations by GitHub. 
- Changing/tweaking the style guide is always option! If you have ideas, reach out!
