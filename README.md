<!--
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2023-09-11 16:13:14
 * @LastEditTime: 2025-10-16 14:05:57
 * @License: GPL 3.0
-->
<h1 align = "center">T-Echo-Card</h1>

<p align="center" width="100%">
    <img src="image/3.jpg" alt="">
</p>

## **English | [中文](./README_CN.md)**

## Version iteration:
| Version                              | Update date                       |
| :-------------------------------: | :-------------------------------: |
| T-Echo-Card_V1.0            | 2025-10-11                         |

## PurchaseLink
| Product                     | SOC           |  FLASH  |  PSRAM   | Link                   |
| :------------------------: | :-----------: |:-------: | :---------: | :------------------: |
| T-Echo-Card_V1.0   | nRF52840 |   1M   |256kB| NULL |

## Directory
- [Describe](#describe)
- [Preview](#preview)
- [Module](#module)
- [SoftwareDeployment](#SoftwareDeployment)
- [PinOverview](#pinoverview)
- [RelatedTests](#RelatedTests)
- [FAQ](#faq)
- [Project](#project)

## Describe

T-Echo-Card is a low-power board developed based on the nRF52840 chip. It features a solar panel for continuous power supply and is equipped with rich onboard functionalities, including an inertial sensor, speaker-microphone module, GPS, and more.

## Preview

### Actual Product Image

<!-- <p align="center" width="100%">
    <img src="image/1.jpg" alt="">
</p>

---

<p align="center" width="100%">
    <img src="image/2.jpg" alt="">
</p>

---

<p align="center" width="100%">
    <img src="image/3.jpg" alt="">
</p> -->

## Module

### 1. MCU

* Chip: nRF52840
* RAM: 256kB
* FLASH: 1MB
* Related Information:
    >[nRF52840_Datasheet](https://docs.nordicsemi.com/bundle/ps_nrf52840/page/keyfeatures_html5.html)

### 2. Display

* Resolution: 128x64px
* Display Type: OLED
* Driver Chip: SSD1306
* Bus Communication Protocol: I2C
* Dependent Libraries:
    >[Adafruit_BusIO](https://github.com/adafruit/Adafruit_BusIO)  
    >[Adafruit-GFX-Library](https://github.com/adafruit/Adafruit-GFX-Library)
* Related Information:
    >[SSD1306](./information/SSD1306.pdf)

### 3. GPS

* Chip: L76K
* Bus Communication Protocol: UART
* Dependent Libraries:
    > [cpp_bus_driver](https://github.com/Llgok/cpp_bus_driver)
* Related Information:
    >[L76K](./information/L76KB-A58.pdf)

### 4. IMU

* Chip: QCM5883P
* Bus Communication Protocol: I2C
* Dependent Libraries:
    >[Adafruit_QMC5883P](https://github.com/adafruit/Adafruit_QMC5883P)   
* Related Information:
    >[QCM5883P](./information/QMC5883P.pdf)

### 5. Flash

* Chip: ZD25WQ32CEIGR
* Bus Communication Protocol: SPI
* Dependent Libraries:
    >[Adafruit_BusIO](https://github.com/adafruit/Adafruit_BusIO)  
    >[Adafruit_SPIFlash](https://github.com/adafruit/Adafruit_SPIFlash)  
* Related Information:
    >[ZD25WQ32CEIGR](./information/ZD25WQ32CEIGR.pdf)

### 6. LED

* Chip: WS2812
* Dependent Libraries:
    > [Adafruit_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel)
* Related Information:
    >[WS2812](./information/WS2812C-2020.pdf)

### 7. Speaker

* Driver Chip: MAX98357
* Bus Communication Protocol: I2S
* Dependent Libraries:
    > [cpp_bus_driver](https://github.com/Llgok/cpp_bus_driver)
* Related Information:
    >[MAX98357](./information/MAX98357AETE+T.pdf)

### 8. Microphone

* Driver Chip: MP34DT05
* Bus Communication Protocol: PDM
* Dependent Libraries:
    > [cpp_bus_driver](https://github.com/Llgok/cpp_bus_driver)
* Related Information:
    >[MP34DT05](./information/mp34dt05-a.pdf)

## SoftwareDeployment

### Examples Support

| Example | `[Arduino IDE (Adafruit_nRF52_V1.6.1)]` <br /> `[PlatformIO (nordicnrf52_V10.6.0)]` <br /> Support | Description | Picture |
| ------  | ------  | ------ | ------ | 
| [buzzer](./examples/buzzer) | <p align="center">![alt text][supported]  |  |  |
| [Display](./examples/Display) | <p align="center">![alt text][supported]  |  |  |
| [Flash](./examples/Flash) | <p align="center">![alt text][supported]  |  |  |
| [Flash_Erase](./examples/Flash_Erase) | <p align="center">![alt text][supported]  |  |  |
| [Flash_Speed_Test](./examples/Flash_Speed_Test) | <p align="center">![alt text][supported]  |  |  |
| [gps](./examples/GPS) | <p align="center">![alt text][supported]  |  |  |
| [IIC_Scan_2](./examples/IIC_Scan_2) | <p align="center">![alt text][supported]  |  |  |
| [original_test](./examples/original_test) |<p align="center">![alt text][supported]  | Product factory original testing |  |
| [pdm](./examples/pdm) | <p align="center">![alt text][supported]  |  |  |
| [play_music](./examples/play_music) | <p align="center">![alt text][supported]  |  |  |
| [qmc5883p](./examples/qmc5883p) | <p align="center">![alt text][supported]  |  |  |
| [voice_speaker](./examples/voice_speaker) | <p align="center">![alt text][supported]  |  |  |
| [ws2812](./examples/ws2812) | <p align="center">![alt text][supported]  |  |  |

[supported]: https://img.shields.io/badge/-supported-green "example"

| Bootloader | Description | Picture |
| ------  | ------  | ------ |
| [bootloader](./bootloader/) |  |  |

| Firmware | Description | Picture |
| ------  | ------  | ------ |
| [original_test](./firmware/[T-Echo-Card_V1.0][original_test]_firmware/)| Product factory original testing |  |

### IDE and Flashing

#### PlatformIO
1. Install [VisualStudioCode](https://code.visualstudio.com/Download),choose installation based on your system type.

2. Open the "Extension" section of the Visual Studio Code software sidebar (Alternatively, use "<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>X</kbd>" to open the extension). Search for the "PlatformIO IDE" extension and download it.

3. During the installation of the extension, you can go to GitHub to download the program. You can download the main branch by clicking on the "<> Code" with green text, or you can download the program versions from the "Releases" section in the sidebar.

4. After the installation of the extension is completed, open the Explorer in the sidebar (Alternatively, use "<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>E</kbd>" go open it). Click on "Open Folder", locate the project code you just downloaded (the entire folder), and click "Add." At this point, the project files will be added to your workspace.

5. Open the "platformio.ini" file in the project folder (PlatformIO will automatically open the "platformio.ini" file corresponding to the added folder). Under the "[platformio]" section, uncomment and select the example program you want to burn (it should start with "default_envs = xxx") Then click "<kbd>[√](image/4.png)</kbd>" in the bottom left corner to compile. If the compilation is correct, connect the microcontroller to the computer and click "<kbd>[→](image/5.png)</kbd>" in the bottom left corner to download the program.

6. At this point, an error may occur, and you need to install [Python](https://www.python.org/downloads/). Open the folder "tool" -> "win10 vscode platformio start" sequentially, and execute the cmd command `python t-echo-card_setup.py` under the "win10 vscode platformio start" folder. This will complete the development board installation, and the compilation and flashing will no longer report errors.

#### Arduino

1. Install [Arduino](https://www.arduino.cc/en/software), and select the installation based on your system type.

2. Open the "example" directory of the project folder, select the example project folder, and open the file ending with ".ino" to open the Arduino IDE project workspace.

3. Open the "Tools" menu bar at the top right -> Select "Board" -> "Board Manager", find or search for "Adafruit_nRF52", and download the board file with the author named "Adafruit". Then return to the "Board" menu bar, select the board type under the "Adafruit_nRF52" board, and the selected board type is determined by the "board = xxx" header under the [env] directory in the "platformio.ini" file. If there is no corresponding board, you need to manually add the board under the "board" directory in the project folder. (If "Adafruit_nRF52" cannot be found, you need to open Preferences -> Add `https://www.adafruit.com/package_adafruit_index.json` to "Additional Board Manager URLs")
    
4. Open the menu bar "[File](image/6.png)" -> "[Preferences](image/6.png)", find the "[Project Folder Location](image/7.png)" section, and copy and paste all the library files along with the folders in the "libraries" folder under the project directory into the "libraries" folder in this directory.
    
5. Select the correct settings in the "Tools" menu, as shown in the table below.
    
| Setting                               | Value                                 |
| :-------------------------------: | :-------------------------------: |
| Board                                 | Nordic nRF52840 DK           |

6.  Select the correct port.

7. Entering Bootloader Download Mode: Press and release the RST (reset) chip button, wait for 1 second (this wait is essential), then press and release the RST button again. Once a new drive letter appears on the computer, it indicates that the device has successfully entered the bootloader download mode.

8.  Click the top right "[√](image/8.png)" to compile. If there are no errors, connect the microcontroller to the computer and click the top right "[→](image/9.png)" to start the flashing process.

#### JLINK Flashing Firmware and Bootloader

1.  Install the software [nRF-Connect-for-Desktop](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop/Download#infotabs)

2.  Install the software [JLINK](https://www.segger.com/downloads/jlink/)

3.  Connect the JLINK pins correctly as shown in the figure below

<p align="center" width="100%">
    <img src="image/12.jpg" alt="">
</p>

4.  Open the software nRF-Connect-for-Desktop and install the tool [Programmer](./image/10.png) and open it

5.  Add files, select both the bootloader file and the firmware file at the same time, click [Erase&write](./image/11.png), and the flashing will be completed.

## PinOverview

For pin definitions, please refer to the configuration file: 
<br />

[t_echo_card_config.h](./libraries/private_library/t_echo_card_config.h)

## FAQ

* Q. After reading the above tutorials, I still don't know how to build a programming environment. What should I do?
* A. If you still don't understand how to build an environment after reading the above tutorials, you can refer to the [LilyGo-Document](https://github.com/Xinyuan-LilyGO/LilyGo-Document) document instructions to build it.

<br />

* Q. Why does Arduino IDE prompt me to update library files when I open it? Should I update them or not?
* A. Choose not to update library files. Different versions of library files may not be mutually compatible, so it is not recommended to update library files.

<br />

* Q. Why is there no debug information output from my board's USB?
* A. Please enable the "DTR" option in your serial assistant software.

<br />

*   Q. Why does the board always fail to program when I directly use USB?
*   A. Please press and release the RST (reset) chip button, wait for 1 second (this wait is essential), then press and release the RST button again. Once a new drive letter appears on the computer, it indicates that the device has entered the bootloader download mode, and programming can now proceed.

<br />

## Project
