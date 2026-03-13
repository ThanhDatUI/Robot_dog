SpotMicro Robot

SpotMicro là một mô hình robot bốn chân được phát triển dựa trên thiết
kế 3D từ Thingiverse. Dự án tập trung vào việc xây dựng một hệ thống
robot hoàn chỉnh gồm phần cứng, phần mềm điều khiển và môi trường mô
phỏng.

Video: https://rogerparks.com/projects/sportmicro/

  ----------
  HARDWARE
  ----------

Danh sách linh kiện:

12 - DS3235 180° Coreless Servo (35kg) 1 - MicroPython PyBoard v1.1 1 -
MPU-6050 6DOF Gyroscope + Accelerometer 1 - LCD I2C 16x2 1 - PCA9685
16-Channel I2C Servo Driver 1 - YPG 20A HV SBEC / XL4015 DC-DC Step Down
1 - FrSky X4R SBUS RC Receiver 1 - 3S 40A BMS cho pin 18650 3 - Sony
Murata VTC5A 18650 2600mAh 25A Battery

  -----------------------
  SOFTWARE ARCHITECTURE
  -----------------------

Chu kỳ vòng lặp điều khiển: 19.65 ms

Trong mỗi vòng lặp:

1.  Đọc điện áp pin thông qua ADC của PyBoard.
2.  Cập nhật LCD hiển thị điện áp pin và thời gian vòng lặp.
3.  Đọc tín hiệu RC thông qua SBUS UART.
4.  Cập nhật State Machine.
5.  Nhận mục tiêu vị trí chân robot (x, y, z).
6.  Profiler tính toán inverse kinematics.
7.  Gửi lệnh servo qua PCA9685 bằng I2C.
8.  Xuất packet debug qua UART.

  --------------------
  UART PACKET FORMAT
  --------------------

Packet bắt đầu: STX () Packet kết thúc: ETX ()

Các giá trị phân tách bằng TAB.

  -------------------
  RC COMMAND PACKET
  -------------------

Packet type: 2

Ví dụ: 2 980 980 980 3 4 5 500 499 8 9 10 11 12 13 14 15 16 17 18 123456

  ----------------------
  SERVO COMMAND PACKET
  ----------------------

Packet type: 3

12 giá trị đầu: góc servo Giá trị tiếp: state machine Giá trị tiếp: số
target còn trong queue Giá trị cuối: thời gian vòng lặp

Ví dụ: 3 515 1 -54 -54 140 140 51 51 -54 -54 140 140 4 0 123456

Thứ tự servo:

front_right_shoulder front_left_shoulder front_right_leg front_left_leg
front_right_foot front_left_foot rear_right_shoulder rear_left_shoulder
rear_right_leg rear_left_leg rear_right_foot rear_left_foot

  ------------------
  SIMULATION SETUP
  ------------------

Clone robot model:

git clone
https://gitlab.com/custom_robots/spotmicro/nvidia-jetson-nano.git cd
nvidia-jetson-nano git checkout 108060ee1e182b1e928ee04db1a5c739a9209621
cd ..

Sau khi clone cần xóa lidar trong:

urdf/spotmicroai_gen.urdf.xml

Xóa các node:

  -------------------
  PYTHON SIMULATION
  -------------------

Yêu cầu: Python 3.7.8 (32-bit)

Cài đặt:

cd spot_sim pipenv install –deploy pipenv run python spot_sim.py cd ..

  ------------------
  SIMULATION MODES
  ------------------

1.  Software Simulation (GUI RC) RC Simulator GUI ghi dữ liệu vào file:
    spot_sim_data.txt

2.  Software Simulation (RC Transmitter) RC transmitter thật gửi dữ liệu
    qua receiver.

3.  Hardware Simulation Robot xử lý lệnh và gửi servo command packet ra
    UART.

  -------------
  UPLOAD CODE
  -------------

Script:

micropython/pyboard.py micropython/upload.py

Upload:

cd spot_sim pipenv run upload.py COM_PORT cd ..

Ví dụ:

pipenv run upload.py COM5

  -------------
  FUTURE WORK
  -------------

-   Điều chỉnh timeout trong rc_sim.py.
-   Xóa debug flag trong thư mục micropython.
-   Kiểm tra lại upload script.
