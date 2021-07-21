## Flight Software Directory

### Directory Structure:
- `mpu/`: Flight software running on the OSD3358 application processor
- `mcu/`: Flight software running on both MIMXRT1062 microcontrollers
  - `src/mcu_ac/`: Active control MCU main program 
  - `src/mcu_hybrid/`: Hybrid engine control MCU main program
  - `lib/`: Libraries shared between **both** MCUs
- `common/`: Common files shared between the MPU and MCUs
