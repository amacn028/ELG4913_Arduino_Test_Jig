import subprocess
import glob
from typing import Optional
import serial
import pigpio
import time
import numpy as np
import spidev
import smbus
import sys
import RPi.GPIO as GPIO
 
def detect_arduino() -> Optional[str]:
    serial_ports = sorted(glob.glob("/dev/ttyACM*"))
    if serial_ports:
        detected_port = serial_ports[-1]  # Use the last detected port
        print(f"Arduino successfully detected at: {detected_port}")
        return detected_port
    else:
        print("No ArduinAo found")
        return None

serial_port = detect_arduino()

class ArduinoFlasher:
    def __init__(self, hex_file: str, serial_port: str, baud_rate: str = "115200"):
        baud_rate: str = "115200"
        self.hex_file = hex_file
        self.baud_rate = baud_rate
        self.serial_port = serial_port  # Use the global function
    def get_avrdude_command(self):
        if not self.serial_port:
            raise ValueError("No Arduino detected. Cannot proceed with flashing.")
        return [
            "avrdude",
            "-c", "arduino",
            "-p", "m328p",
            "-P", self.serial_port,
            "-b", self.baud_rate,
            "-D",
            "-U", f"flash:w:{self.hex_file}:i"
        ]
    
    def flash(self):
        if not self.serial_port:
            print("Cannot flash: No Arduino detected.")
            return
        print("Flashing Arduino...")
        try:
            subprocess.run(
                self.get_avrdude_command(),
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            print("Flashing complete")
        except subprocess.CalledProcessError as e:
            print("Flashing failed:", e)

class TimerTest:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=115200, timeout=2, hexfile=None, flasher=None):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
        self.expected = 16000000
        self.margin = self.expected * 0.10
        self.test_count = 0
        self.passed_count = 0
        self.max_tests = 9
        self.hexfile = hexfile
        self.flasher = flasher

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
        except serial.SerialException as e:
            print(f"Serial connection failed: {e}")
            exit(1)

    def parse_timer_line(self, line):
        try:
            parts = line.strip().split(":")
            if len(parts) != 2:
                return None
            iteration_part = parts[0].strip().replace("Iteration ", "")
            values_part = parts[1].strip().split(",")
            iteration = int(iteration_part)
            values = [int(v.strip()) for v in values_part]
            if len(values) != 3:
                return None
            return (iteration, *values)
        except:
            return None

    def check_value(self, value, timer_name):
        self.test_count += 1
        if abs(value - self.expected) <= self.margin:
            print(f"Test {self.test_count}: {timer_name} Passed")
            self.passed_count += 1
        else:
            print(f"Test {self.test_count}: {timer_name} Failed (Value: {value})")

    def listen_and_test(self):
        self.connect()
        print("Listening for timer output from Arduino...\n")
        try:
            while self.test_count < self.max_tests:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    result = self.parse_timer_line(line)
                    if result:
                        iteration, t0, t1, t2 = result
                        self.check_value(t0, f"Timer0 (Iteration {iteration})")
                        self.check_value(t1, f"Timer1 (Iteration {iteration})")
                        self.check_value(t2, f"Timer2 (Iteration {iteration})")
                    else:
                        print(f"[Unparsed] {line}")
            print("\n--- Test Summary ---")
            print(f"✅ {self.passed_count}/{self.max_tests} tests passed.")
            if self.passed_count == self.max_tests:
                print("✔️ Timer test completed successfully.")
            else:
                print("❌ Timer test failed.")
        except KeyboardInterrupt:
            print("\n[!] Stopped by user.")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("Timer Test Finished")

    def run_timer_test(self):
        if self.flasher and self.hexfile:
            print("[Flashing Arduino with hex file...]")
       
        self.listen_and_test()

        return self.passed_count == self.max_tests

class EEPROMTest:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=115200, timeout=2, hexfile=None, flasher=None):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
        self.hexfile = hexfile
        self.flasher = flasher

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
        except serial.SerialException as e:
            print(f"Serial connection failed: {e}")
            exit(1)

    def listen_for_result(self):
        self.connect()
        print("Listening for EEPROM test result...\n")

        try:
            while True:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[Serial Output] {line}")
                    if line.startswith("Passed:"):
                        return True
                    elif line.startswith("Failed:"):
                        return False
        except KeyboardInterrupt:
            print("\n[!] Stopped by user.")
            return False
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("EEPROM Test Finished")

    def run_eeprom_test(self):
        if self.flasher and self.hexfile:
            print("[Flashing Arduino with EEPROM test hex file...]")
        

        return self.listen_for_result()
    
def test_spi(serial_port):
    HEX_PATH = "/home/aaronm/CapstoneProjectFiles/spitest.ino.hex"
    PORT = serial_port

    print("Flashing Arduino...")
    result = subprocess.run([
        "avrdude",
        "-c", "arduino",
        "-p", "m328p",
        "-P", PORT,
        "-b", "115200",
        "-D",
        "-U", f"flash:w:{HEX_PATH}:i"
    ], capture_output=True, text=True)

    if result.returncode == 0:
        print("Flashing complete.")
    else:
        print("Flashing failed!")
        print(result.stderr)
        return False

    print("Waiting for Arduino SPI to become active...")
    time.sleep(4)

    # Initialize SPI
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 50000

    # Send 64 dummy bytes to receive response
    response = spi.xfer([0x00] * 64)
    print("SPI Received:", response)

    # Expected full array from 0 to 63
    expected = list(range(64))

    print("SPI Expected:", expected)



    if response == expected:
        print("✅ SPI Test completed successfully")
        return True
    else:
        print("❌ SPI Test failed")
        print("Expected:", expected)
        print("Received:", response)
        return False

def test_i2c():
    bus = smbus.SMBus(1)  # I2C bus 1 on Raspberry Pi
    address = 0x08        # Arduino I2C address
    expected = list(range(32))

    time.sleep(1)  # Wait briefly to avoid 121 IO Error

    try:
        print("Requesting 32 bytes from Arduino...")
        data = bus.read_i2c_block_data(address, 0, 32)
        print("Received:", data)
        print("Expected:", expected)

        if data == expected:
            print("✅ I2C Test completed successfully")
            return True
        else:
            print("❌ I2C Test failed.")
            return False

    except OSError as e:
        print("❌ I2C Read Failed:", e)
        return False

class VoltageTest:
    def __init__(self, port, baud_rate=115200, timeout=3, duration=2):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.duration = duration
        self.v5_list = []
        self.v3_list = []
        self.avg_5v = None
        self.avg_3v = None

    def read_voltages(self):
        try:
            ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            return False

        print(" Reading voltage data from Arduino...")
        start_time = time.time()

        while time.time() - start_time < self.duration:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    if "Measured Voltage 5V" in line:
                        voltage = float(line.split(":")[1].strip().replace("V", ""))
                        self.v5_list.append(voltage)
                    elif "Measured Voltage 3.3V" in line:
                        voltage = float(line.split(":")[1].strip().replace("V", ""))
                        self.v3_list.append(voltage)
            except Exception as e:
                print(f"⚠️ Error while reading: {e}")
                break

        ser.close()
        return True

    def analyze_results(self):
        if self.v5_list and self.v3_list:
            self.avg_3v = np.average(self.v3_list)
            self.avg_5v = np.average(self.v5_list)

            print(f"3.3V Samples: {self.v3_list}")
            print(f"5V Samples: {self.v5_list}")
            print(f"Average 3.3V: {self.avg_3v:.3f} V")
            print(f"Average 5V: {self.avg_5v:.3f} V")

            if 3.2 < self.avg_3v < 3.45 and 4.8 < self.avg_5v < 5.1:
                print("✅ Voltage Test completed successfully")
                return True
            else:
                print("❌ Voltage Test failed.")
                return False
        else:
            print("❌ Final status: Voltage Test Failed (Missing data)")
            return False
    def run_voltage_test(self):
        if self.read_voltages():
            return self.analyze_results()
        else:
            print("❌ Final status: Voltage Test Failed (Serial error)")
            return False
class PWMTest:
    def __init__(self, pin=18, timeout=3.0, cycles=10):
        self.pin = pin
        self.timeout = timeout
        self.cycles = cycles
        self.freq = None
        self.duty = None

    def _start_pigpiod(self):
        """Start pigpiod if it's not running."""
        try:
            result = subprocess.run(["pgrep", "pigpiod"], stdout=subprocess.DEVNULL)
            if result.returncode != 0:
                print("Starting pigpiod...")
                subprocess.run(["sudo", "pigpiod"])
                time.sleep(0.2)
        except Exception as e:
            raise RuntimeError(f"Failed to start pigpiod: {e}")

    def read_pwm(self):
        self._start_pigpiod()
        pi = pigpio.pi()

        if not pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon.")

        pi.set_mode(self.pin, pigpio.INPUT)

        periods = []
        high_times = []
        state = {"last_rising": None}

        def cbf(gpio, level, tick):
            nonlocal state
            if level == 1:
                if state["last_rising"] is not None:
                    period = tick - state["last_rising"]
                    if period > 0:
                        periods.append(period)
                state["last_rising"] = tick
            elif level == 0:
                if state["last_rising"] is not None:
                    high_time = tick - state["last_rising"]
                    if high_time > 0:
                        high_times.append(high_time)

        cb = pi.callback(self.pin, pigpio.EITHER_EDGE, cbf)

        start_time = time.time()
        try:
            while (len(periods) < self.cycles or len(high_times) < self.cycles) and (time.time() - start_time < self.timeout):
                time.sleep(0.001)
            if len(periods) < self.cycles or len(high_times) < self.cycles:
                raise RuntimeError("Not enough PWM cycles detected.")
        finally:
            cb.cancel()
            pi.stop()

        avg_period = sum(periods[-self.cycles:]) / self.cycles
        avg_high = sum(high_times[-self.cycles:]) / self.cycles

        self.freq = 1e6 / avg_period
        self.duty = min((avg_high / avg_period) * 100, 100.0)

        return self.freq, self.duty

    def run_pwm_test(self):
        try:
            freq, duty = self.read_pwm()
            print(f"✅ PWM Test Passed\n - Frequency: {freq:.2f} Hz\n - Duty Cycle: {duty:.2f}%")
            return True
        except Exception as e:
            print(f"❌ PWM Test Failed: {e}")
            return False
