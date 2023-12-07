# Notatki z walki z inżynierką

## 1.12

### Motive

Aby modyfikować pivot point zasobu należy wybrać zasób, wejść w zakładkę `Builder->Modify`. Używając narzędzi `Location` i `Orientation` można zmieniać położenie i orientację pivot pointu.

### mocap4ros2

Paczki się nie budują, prawdopodobnie brakuje bibliotek z Pythona 3.8, których `rosidl` chce używać:

```bash
make[2]: *** No rule to make target '/usr/lib/x86_64-linux-gnu/libpython3.8.so', needed by 'rosidl_generator_py/mocap_msgs/libmocap_msgs__rosidl_generator_py.so'.  Stop.
```

```bash
ros2-humble@mrlab-server:/usr/lib/x86_64-linux-gnu$ ls | grep python
libboost_mpi_python310.a
libboost_mpi_python310.so
libboost_mpi_python310.so.1.74.0
libboost_python310.a
libboost_python310.so
libboost_python310.so.1.74.0
libpyldb-util.cpython-310-x86-64-linux-gnu.so.2
libpyldb-util.cpython-310-x86-64-linux-gnu.so.2.4.4
libpyside2.cpython-310-x86_64-linux-gnu.so
libpyside2.cpython-310-x86_64-linux-gnu.so.5.15
libpyside2.cpython-310-x86_64-linux-gnu.so.5.15.2
libpytalloc-util.cpython-310-x86-64-linux-gnu.so.2
libpytalloc-util.cpython-310-x86-64-linux-gnu.so.2.3.3
libpython3.10.a
libpython3.10.so
libpython3.10.so.1
libpython3.10.so.1.0
libsamba-policy.cpython-310-x86-64-linux-gnu.so.0
libsamba-policy.cpython-310-x86-64-linux-gnu.so.0.0.1
libshiboken2.cpython-310-x86_64-linux-gnu.so
libshiboken2.cpython-310-x86_64-linux-gnu.so.5.15
libshiboken2.cpython-310-x86_64-linux-gnu.so.5.15.2
```

Co powoduje brak pliku nagłówkoego `Python.h`.

Odpalanie już zbudowanych wersji paczek powoduje ten błąd:

```bash
ros2-humble@mrlab-server:~/mocap2_ws$ source install/setup.bash 
ros2-humble@mrlab-server:~/mocap2_ws$ ros2 launch mocap_optitrack_driver optitrack2.launch.py 
[INFO] [launch]: All log files can be found below /home/ros2/.ros/log/2023-12-01-16-42-14-171318-mrlab-server-166850
[INFO] [launch]: Default logging verbosity is set to INFO
RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED is now ignored. Please set RCUTILS_LOGGING_USE_STDOUT and RCUTILS_LOGGING_BUFFERED_STREAM to control the stream and the buffering of log messages.
[INFO] [mocap_optitrack_driver_main-1]: process started with pid [166860]
[mocap_optitrack_driver_main-1] /home/ros2/mocap2_ws/install/mocap_optitrack_driver/lib/mocap_optitrack_driver/mocap_optitrack_driver_main: symbol lookup error: /home/ros2/mocap2_ws/install/mocap_control/lib/libmocap_control.so: undefined symbol: _ZN6rclcpp19QOSEventHandlerBase15add_to_wait_setEP14rcl_wait_set_t
[ERROR] [mocap_optitrack_driver_main-1]: process has died [pid 166860, exit code 127, cmd '/home/ros2/mocap2_ws/install/mocap_optitrack_driver/lib/mocap_optitrack_driver/mocap_optitrack_driver_main --ros-args -r __node:=mocap_optitrack_driver_node -r __ns:=/ --params-file /home/ros2/mocap2_ws/install/mocap_optitrack_driver/share/mocap_optitrack_driver/config/mocap_optitrack_driver_params.yaml'].
```

Rozwiązanie:

```bash
sudo apt install ros-${ROS_DISTRO}-ament-cmake-clang-format
```

## 4.12

### MoCap

Na topic `/rigid_bodies` wysyłane są informacje o aktywowanych RigidBodies. `rigid_body_name` z jakiegoś powodu jest zamienione na indeks w postaci liczby całkowitej.

- `15` - `tb2_3`
- `16` - `tb2_6`

Na ww. topic wysyłane są wiadomości typu `mocap_msgs/msg/RigidBodies` skąd można odczytać informacje dotyczące położenia i orientacji w globalnym układzie współrzędnych każdego z przesłanych obiektów typ `RigidBody`. Położenie i orientacja są określane względem `pivot point` każdego z obiektów (domyślnie jest to geometryczny środek obiektu wyznaczoneho na podstawie markerów.)

- `16` - `tb2_6`
- `17` - `tb2_5`

**Trzeba zmienić tb2_3 na tb2_5 z racji tego, że na nr 3 nie ma paczek.**

#### Brakujące paczki

- ros-testing
- laser-proc
- urg-c
- urg-node-msgs
- joint-state-publisher

#### Downgrade wersji setuptools

żeby zlikwidować błąd o `deprecated easy_install`

- sudo apt install python3-pip
- pip install setuptools==58.2.0

#### Ustawienie manualne ipv4 przez cli

```bash
sudo nmcli connection modify "Hokuyo" ipv4.addresses 192.168.0.15/24 ipv4.gateway 192.168.0.1 ipv4.method manual
```
