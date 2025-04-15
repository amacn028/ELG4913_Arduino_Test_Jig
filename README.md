# ELG4913_Arduino_Test_Jig
code for the Arduino automated test jig 

 you need to upload  a json file to your ESP32  before you will be able to flash program the ESP32 in Arduino IDE.
 You can follow the steps below to accomplish this:
 1) download and install Arduino IDE on your computer
 2) in Arduino IDE: Files > Preferences to open preferences menu
 3) in the preferences menu > settings tab
 4) copy the following link: https://espressif.github.io/arduino-esp32/package_esp32_index.json
 5) in the settings tab navigate to "Additional boards manager URLS" and paste the link above into there, click ok
 6) click ok on preferences menu
 7) In Arduino IDE navigate to Boards manager window, search esp32 in the search prompt to find the esp32 boards manager file from Espressif Systems.
 8) Make sure the version of the esp32 boards manager file is at minimum 2.0.9, click install
 9) wait for the installation to finish
 10) in Arduino IDE > Tools > Board : select ESP32 Dev Module
 11) now you will be able to flash program the ESP32 from Arduino IDE
