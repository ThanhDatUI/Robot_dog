# RObot dog
(tham khao tu https://www.instructables.com/Nova-Spot-Micro-a-Spot-Mini-Clone)
## How to set up ?
SpotMicro Robot Dog — Installation Guide
![Banner](image/z7617608427412_38e81755e0fdc324a346715e398cc415.jgp)
1.  System Requirements

-   Windows 10 / Windows 11
-   Python 3.10 or newer
-   ESP32-S3 flashed with MicroPython v1.27+

  -------------------
  2. Install Python
  -------------------

Download Python: https://www.python.org/downloads/

During installation, check: Add Python to PATH

Verify installation: python –version

  --------------------------------
  3. Install Python Dependencies
  --------------------------------

Open PowerShell and run:

pip install pyserial mpremote

Libraries used: - pyserial : serial communication with ESP32 - mpremote
: upload files to MicroPython

  -----------------------------------------------
  4. Install USB Driver (if ESP32 not detected)
  -----------------------------------------------

Download CH340 driver:
https://www.wch-ic.com/downloads/CH341SER_EXE.html

After installing: 1. Plug ESP32 2. Open Device Manager 3. Check COM port

Example: COM9

  -----------------------------
  5. Upload Firmware to ESP32
  -----------------------------

Open PowerShell in the project folder:

cd C:...-Dog-V1.3

Upload firmware files:

python -m mpremote connect COM9 cp micropython/pca9685.py :pca9685.py
python -m mpremote connect COM9 cp micropython/profiler.py :profiler.py
python -m mpremote connect COM9 cp micropython/state.py :state.py python
-m mpremote connect COM9 cp micropython/crawl.py :crawl.py python -m
mpremote connect COM9 cp micropython/esp32_robot_receiver.py :main.py
python -m mpremote connect COM9 reset

Replace COM9 with your ESP32 COM port.

  ----------------------
  6. Verify ESP32 Boot
  ----------------------

python -m mpremote connect COM9 repl

If you see:

READY WIFI IP: xxx.xxx.xxx.xxx

The firmware is running correctly.

Exit REPL: Ctrl + ]

  ------------------------
  7. Run Robot Dashboard
  ------------------------

python robot_dashboard.py

  ---------------------
  8. Robot Connection
  ---------------------

Serial Mode (USB cable)

1.  Select Serial
2.  Choose COM port
3.  Click Connect

WiFi Mode

Enable hotspot:

SSID:  ............. Password: .........

In Dashboard:

1.  Select WiFi
2.  Enter robot IP
3.  Port: 8888
4.  Click Connect

  ----------------------------
  9. Firmware Files on ESP32
  ----------------------------

micropython/pca9685.py -> pca9685.py micropython/profiler.py ->
profiler.py micropython/state.py -> state.py micropython/crawl.py ->
crawl.py micropython/esp32_robot_receiver.py -> main.py

  ------------------------
  10. WiFi Configuration
  ------------------------

Edit file:

micropython/esp32_robot_receiver.py

Configuration:

WIFI_SSID = ............. WIFI_PASS = .............. TCP_PORT = 8888

Re-upload main.py after modification.

  -----------
  11. Notes
  -----------

If modifying: - esp32_robot_receiver.py - crawl.py

You must upload firmware again.

If mpremote shows “Up to date”, remove the file first:

python -m mpremote connect COM9 exec “import os; os.remove(‘main.py’)” +
cp micropython/esp32_robot_receiver.py :main.py + reset
