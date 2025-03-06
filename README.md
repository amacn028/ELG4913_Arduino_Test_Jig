# ELG4913_Arduino_Test_Jig
code for the Arduino automated test jig 

 you need to upload the 2 libraries (ina219.py and logging.py) to your ESP32 before you will be able to flash program the ESP32 in thonny ide.
 You can follow the steps below to accomplish this:
 1) download both the ina219.py and logging.py to your computer
 2) in thonny: View > Files ( make sure this is checked) : this will open the file viewer tab in thonny
 3) in the file viewer navigate to the directory where you stored the files
 4) in the file viewer tab, right click ina219.py : upload to micropython device ( wait for it to finish flashing to the ESP32)
 5) in the file viewer tab, right click logging.py : upload to micropython device ( wait for it to finish flashing to the ESP32)
 6) now you will be able to flash program the ESP32 from thonny ide
