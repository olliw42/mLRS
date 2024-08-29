# Targets

This page documents in detail the steps with which the targets in this repository were created.

It can, and should, serve as manual for creating new targets. For some steps it is very important to do them in the correct sequence. So it is best to just follow all steps exactly as given below.

Quotation marks "" are used to enclose text, but should never be entered. '->' is used to indicate to goto a certain field, option, menu.

Some steps depend on the STM32 part.

Notation:
- 'part' refers to the STM32 chip family and version. Example: STM32G431KB.
- 'target' refers to the code folder specific for the board/device. Example: tx-diy-E22-g431kb. The target thus also implies a specific part.

It is assumed that the build environment for mLRS was set up as described in [Software: Installation Bits and Bops](https://github.com/olliw42/mLRS#software-installation-bits-and-bops).

## I. Create Target: Cube MX

The steps in this chapter are target and part dependent. Pay attention to the details.

### 1. File -> New -> STM32 Project

This opens the "Target Selection" dialog (note: this can take a while, be patient).

### 2. MCU/MPU Selector -> Commercial Part Number: enter the part (e.g., STM32G431KB)

The part to enter here depends on the specific target which shall be created.

On the right side a selection of compatible parts will be shown, which differ by their package, e.g., may have an extension like U6 (e.g. STM32G431KBU6). It doesn't matter which of them you chose, but these are the preferred choices ('x' stands for a number like 3 or 6, choose whatever you like):
- STM32F103C8Tx, STM32F103CBTx, STM32F103RBHx
- STM32G431KBUx, STM32G441KBUx, STM32G431CBUx, STM32G491RETx
- STM32WLE5CCUx, STM32WLE5JCIx
- STM32F072CBTx
- STM32F303CCTx

Then hit "Next".

This opens the "Project & Options" dialog.

### 3. Project -> Project Name: enter the name of the target (e.g. "tx-diy-E22-g431kb")

The target name must start with "tx-" or "rx-" and must end with the part shorthand name in lower case, e.g., "-g431kb".

-> Options -> Targeted Language: choose "C++"

Then hit "Finish".

This opens the STM32CubeMX "Pinout & Configuration" page (inside STM32CubeIDE).

### 4. Pinout & Configuration

- STM32F103 and STM32G4 parts
  - -> System Core -> SYS -> Debug: select "Serial Wire"
  - -> System Core -> RCC -> High Speed Clock (HSE): select "Crystal/Ceramic Resonator"

- STM32WLE5CC in "old" E77 module (E77 module with xtal, not TCXO)
  - -> System Core -> SYS -> Timebase Source: select "SysTick"
  - -> System Core -> RCC -> High Speed Clock (HSE): select "Crystal/Ceramic Resonator"
  - -> Trace and Debug -> DEBUG -> JTAG and Trace: select "Serial Wire"

- STM32WLE5JC in WioE5 module or STM32WLE5CC in "new" E77 module (E77 module with TCXO)
  - -> System Core -> SYS -> Timebase Source: select "SysTick"
  - -> System Core -> RCC -> High Speed Clock (HSE): select "TCXO"
  - -> Trace and Debug -> DEBUG -> JTAG and Trace: select "Serial Wire"

- STM32F072 parts
  - -> System Core -> SYS -> check "Debug Serial Wire"
  - -> System Core -> RCC -> High Speed Clock (HSE): select "Crystal/Ceramic Resonator"

- STM32F303 parts
  - -> System Core -> SYS -> check "Debug Serial Wire"
  - -> System Core -> RCC -> High Speed Clock (HSE): select "Crystal/Ceramic Resonator"

### 5. Clock Configuration

- STM32F1 parts
  - -> Input frequency: enter 8
  - -> select check buttons such that HSE & PLLCLK is selected for SYSCLK
  - -> HCLK (Mhz): enter 72 (corresponds to PLLMul = x9)

- STM32G4 parts
  - -> Input frequency: enter 8
  - -> select check buttons such that HSE & PLLCLK is selected for SYSCLK
  - -> HCLK (Mhz): enter 170 (corresponds to PLLM = /2, PLLN = x85, PLLR = /2)

- STM32WLE5 parts (E77 and WioE5)
  - -> Input frequency: enter 32
  - -> select check buttons such that HSE & PLLCLK is selected for SYSCLK
  - -> HCLK1 (Mhz): enter 48 (corresponds to PLLM = /2, PLLN = x6, PLLR = /2)

- STM32F072 parts
  - -> Input frequency: enter 8
  - -> select check buttons such that HSE & PLLCLK is selected for SYSCLK
  - -> HCLK (Mhz): enter 48 (corresponds to PREDiv = /1, VCOInput = 8, PLLMul = x6)

- STM32F303 parts
  - -> Input frequency: enter 12
  - -> select check buttons such that HSE & PLLCLK is selected for SYSCLK
  - -> HCLK (Mhz): enter 72 (corresponds to PLLMul = x6)

You may have to do this a couple of times, or need to manually help with entering specific values in the PLL block.

### 6. Project Manager

-> Project -> Linker Settings -> Minimum Heap Size: change to "0x0"

-> Code Generator -> Generated files: un-check "Delete previously generated files when not re-generated"

-> Advanced Settings -> Driver Selector -> GPIO: select "LL"

Then hit "Save" or "Save All".

Acknowledge all dialogs which open with yes.

The file "main.c" should be opened now in the editor.


## II. Create Target: Mangle Files

The steps in this chapter are basically equal for all targets and parts, with few exceptions.

### 7. In the IDE Project explorer: unfold "Core", unfold "Src", right-mouse click main.c -> Rename

This opens the "Rename Resource" dialog.

-> New name: change to "main.cpp"

Then hit "OK".

### 8. Double-click on "main.cpp" to open file in editor

-> find section "/\* USER CODE BEGIN PFP \*/" and insert "int main_main();"

-> find section "/\* USER CODE BEGIN 2 \*/" and insert "return main_main();"

- STM32G4 parts only
  - -> find section "/\* USER CODE BEGIN 1 \*/" and insert "__HAL_FLASH_DATA_CACHE_DISABLE();"

### 9. Go to "mlrs/Tools" folder

-> run the Python script "run_copy_st_drivers.py"


## III. Create Target: IDE Config

The steps in this chapter are equal for all targets and parts.

### 10. hammer icon -> down arrow: select Release

This starts the compiler. Let it run, don't worry about the errors which result.

### 11. Right-mouse click on newly created target (e.g. "tx-diy-e22-g431kb") -> New -> Folder

This opens the "New Folder" dialog.

-> Advanced -> check "Link to alternate folder (Linked Folder)"
-> hit "Browse"

This opens the select folder dialog.

-> select the folder "Common" and accept
-> hit "Finish"

This should add a folder "Common" to the project.

Repeat these steps for "CommonRx" or "CommonTx", depending on whether it is a rx or tx target.

Repeat the steps also for "modules".

### 12. Right-mouse click on "Common" -> Properties

This opens the "Properties for Common" dialog.

-> C/C++ Build -> Settings: uncheck "Exclude resource from build"
-> hit "Apply and Close"

Repeat these steps for "CommonRx" or "CommonTx", depending on whether it is a rx or tx target.

Repeat the steps also for "modules".

### 13. Unfold "modules" folder, to see "fastmavlink", "mavlink", "stm32ll-lib", "sx12xx-lib"

Right-mouse click on "fastmavlink" -> Properties.

This opens the "Properties for fastmavlink" dialog.

-> C/C++ Build -> Settings: check "Exclude resource from build"

Then hit "Apply and Close".

Repeat the steps for "mavlink".

## IV. Create Target: Target HAL

These steps are equal for all targets and parts, except of course that values need to be entered as appropriate for the target and part.

### 14. In the IDE, unfold "Common", unfold "hal"

-> double-click "device_conf.h" to open this file in the editor
-> enter new target in appropriate place, with define e.g. TX_DIY_E22_G431KB

-> double-click "hal.h" to open this file in the editor
-> enter new target in appropriate place, with define e.g. TX_DIY_E22_G431KB, tx-hal-diy-e22-g431kb.h

### 15. Create decvice hal file (e.g. tx-hal-diy-e22-g431kb.h) with appropriate entries

The hal file for the target contains (nearly) all information on the specific board/device. Creating it with the proper content thus represents a most important step.

### 16. Right-mouse click created target (e.g. "tx-hal-diy-e22-g431kb") -> Properties

This opens the "Properties for tx-hal-diy-e22-g431kb" dialog.

-> C/C++ Build -> Settings -> Tool Settings -> MCU G++ Compiler -> Preprozessor -> Define Symbols -> "+" icon: add e.g. "TX_DIY_E22_G431KB"

Then hit "Apply and Close".

## V. Create Target: Finalize

The project should compile now without errors, and only some intentional warnings. A last step is needed.

### 17. Close STM32CubeIDE, go to "mlrs/Tools" folder

-> run the Python script "run_replace_names_by_variables.py"


## A1. Targets with USB

Targets which use the native USB peripheral (e.g. for the COM port) involve some additional steps.

### 1. STM32 USB Device Library files

The STM32 USB Device Library files need to be copied to the target folder. Two approaches are availabe.

- Targets which are registered in run_make_firmware3.py can add 'STDSTM32_USE_USB' to 'extra_D_list'. Calling run_copy_drivers.py then also copies the STM32 USB Device Library files to the respective target.

- The STM32 USB Device Library files can be also copied manually to a specific target by calling run_copy_st_drivers.py with the command line option "--usb". Example: run_copy_st_drivers.py -t tx-diy-yourtarget-g431cb --usb, if the target name is "tx-diy-yourtarget-g431cb".

### 2. Include Paths and Defines

In the IDE, right-mouse click the target (e.g. "tx-diy-yourtarget-g431cb")

-> Properties

This opens the "Properties for tx-diy-yourtarget-g431cb" dialog.

-> C/C++ Build -> Settings

This brings you to the settings tab.

Three include paths need to be added to both the GCC and G++ compilers. Namely

- ../Drivers/STM32_USB_Device_Library/Core/Inc
- ../Drivers/STM32_USB_Device_Library/Class/CDC/Inc
- "${workspace_loc:/${ProjName}/modules/stm32-usb-device}"

-> MCU/MPU GCC Compiler -> Include Paths: select "+" icon, enter path, click down arrow to move it to last position

repeat this 3 times for each include path

-> MCU/MPU G++ Compiler -> Include Paths: select "+" icon, enter path, click down arrow to move it to last position

repeat this 3 times for each include path

The define STDSTM32_USE_USB needs to be added to both the GCC and G++ compilers:

-> MCU/MPU GCC Compiler -> Preprozessor: select "+" icon, enter STDSTM32_USE_USB, click down arrow to move it to last position
-> MCU/MPU G++ Compiler -> Preprozessor: select "+" icon, enter STDSTM32_USE_USB, click down arrow to move it to last position

### 3. Set HAL_PCD_MODULE_ENABLED

In the IDE, unfold the "Core" folder, unfold the "Inc" folder, and double-click "stm32yyxx_hal_conf.h" to open it in the editor.

Un-comment the line with #define HAL_PCD_MODULE_ENABLED.





