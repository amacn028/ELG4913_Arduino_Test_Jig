import TestLibrary as T
import tkinter as tk
from tkinter import messagebox
import RPi.GPIO as GPIO
import time
import threading
import concurrent.futures

# LED pins
RED_LED = 16
YELLOW_LED = 20
GREEN_LED = 21

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(RED_LED, GPIO.OUT)
GPIO.setup(YELLOW_LED, GPIO.OUT)
GPIO.setup(GREEN_LED, GPIO.OUT)

def reset_leds():
    GPIO.output(RED_LED, GPIO.LOW)
    GPIO.output(YELLOW_LED, GPIO.LOW)
    GPIO.output(GREEN_LED, GPIO.LOW)

def run_with_timeout(func, timeout=10):
    try:
        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(func)
            return future.result(timeout=timeout)
    except concurrent.futures.TimeoutError:
        print("❌ Test timed out.")
        return False
    except Exception as e:
        print(f"❌ Test error: {e}")
        return False

def run_tests():
    reset_leds()
    GPIO.output(YELLOW_LED, GPIO.HIGH)
    serial_port = T.detect_arduino()

    if serial_port is None:
        GPIO.output(RED_LED, GPIO.HIGH)
        messagebox.showerror("❌ No Arduino", "Arduino was not detected. Test failed.")
        time.sleep(5)
        reset_leds()
        return

    def run_timer_test():
        hexfile = "/home/aaronm/CapstoneProjectFiles/TimerTest.ino.hex"
        AF = T.ArduinoFlasher(hexfile, serial_port, baud_rate="115200")
        AF.flash()
        test = T.TimerTest(port=serial_port, hexfile=hexfile)
        return test.run_timer_test()

    def run_EEPROM_test():
        hexfile = "/home/aaronm/CapstoneProjectFiles/EEPROM_clean.ino.hex"
        AF = T.ArduinoFlasher(hexfile, serial_port, baud_rate="115200")
        AF.flash()
        test = T.EEPROMTest(port=serial_port, hexfile=hexfile)
        return test.run_eeprom_test()

    def run_SPI_test():
        return T.test_spi(serial_port)

    def run_I2C_test():
        hexfile = "/home/aaronm/CapstoneProjectFiles/i2ctest.ino.hex"
        AF = T.ArduinoFlasher(hexfile, serial_port, baud_rate="115200")
        AF.flash()
        return T.test_i2c()
    def run_voltage_test():
        hexfile = "/home/aaronm/CapstoneProjectFiles/AnalogueRead.ino.hex"
        AF = T.ArduinoFlasher(hexfile, serial_port, baud_rate="115200")
        AF.flash()
        return T.VoltageTest(port=serial_port)
    def run_PWM_test():
        hexfile = "/home/aaronm/CapstoneProjectFiles/ArduinoPWM.ino.hex"
        AF = T.ArduinoFlasher(hexfile, serial_port, baud_rate="115200")
        AF.flash()
        return T.PWMTest(port=serial_port)

    # Run each test with timeout
    results = [
        run_with_timeout(run_timer_test),
        run_with_timeout(run_EEPROM_test),
        run_with_timeout(run_SPI_test),
        run_with_timeout(run_I2C_test),
        run_with_timeout(run_voltage_test),
        run_with_timeout(run_PWM_test)
    ]

    all_passed = all(results)

    reset_leds()
    if all_passed:
        GPIO.output(GREEN_LED, GPIO.HIGH)
        messagebox.showinfo("✅ Test Result", "All tests passed successfully!")
    else:
        GPIO.output(RED_LED, GPIO.HIGH)
        messagebox.showerror("❌ Test Result", "One or more tests failed.")

    time.sleep(5)
    reset_leds()
    print("LEDs turned off.")

# GUI Setup
root = tk.Tk()
root.title("Capstone Test Suite")
root.geometry("480x320")

button = tk.Button(
    root,
    text="Run Tests",
    font=("Arial", 24),
    width=15,
    height=3,
    bg="green",
    fg="white",
    command=run_tests
)
button.pack(expand=True)

root.mainloop()
