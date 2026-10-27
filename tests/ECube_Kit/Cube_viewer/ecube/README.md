# E-Cube: firmware + 3D CubeSat viewer

```
ecube/
  firmware/   PlatformIO project for the E-Cube's ESP32 (streams sensor data over USB)
  viewer/     Python viewer: a 1U CubeSat that moves like the real board
```

## 1. Try the viewer without hardware

```bash
cd viewer
python -m venv .venv && source .venv/bin/activate     # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python ecube_viewer.py --demo
```

## 2. Flash the E-Cube

```bash
cd firmware
pio run -t upload            # or open the folder in VS Code + PlatformIO and click Upload
pio device monitor           # 460800 baud; look at the lines starting with '#'
```

The boot log shows what was found, for example:

```
# i2c addresses: 0x1E 0x68 0x77
# found ;MPU(id=0x68)@0x68;HMC5883L@0x1E;BMP180@0x77;
# READY
```

Keep the cube still for about 2 s after reset (gyro calibration).
If it prints `no accelerometer found`, the I2C pins are different on your mission
board: change `I2C_SDA` / `I2C_SCL` at the top of `src/main.cpp`.

## 3. Run the viewer live

Close the serial monitor first (only one program can use the port), then:

```bash
cd viewer
python ecube_viewer.py --port /dev/ttyUSB0        # Windows: --port COM5
```

On Linux you may need `sudo usermod -aG dialout $USER` (Void: group `uucp`), then log out/in.
Opening the port resets the ESP32, so keep the cube still for the first 2 s.

Keys: `m` magnetometer calibration, `z` zero yaw, `space` record CSV, `q` quit.

## 4. Calibrate the compass (important)

A CubeSat has batteries, solar panels and magnets, so the compass is biased.
Press `m`, rotate the cube slowly through **all** orientations (like a figure 8 in 3D),
press `m` again. The result is saved to `mag_cal.json` and reused next time.
In the demo this reduced the attitude error from about 11 degrees to 0.3 degrees.

## 5. If the cube moves the wrong way

Sensor axes on the board may differ from the cube axes. Remap them:

```bash
python ecube_viewer.py --port /dev/ttyUSB0 --imu-map -y,x,z   # accel + gyro
python ecube_viewer.py --port /dev/ttyUSB0 --mag-map -y,x,z   # magnetometer
```

Each name says where the new X, Y, Z come from (`-y,x,z`: new X = -old Y, new Y = old X).
Test: tilt the cube forward/sideways and check the model follows; then rotate it flat about
vertical and check the heading follows your turn. If tilts are right but the heading is wrong or
fights the gyro, fix `--mag-map`. If tilts are wrong, fix `--imu-map`. Use `--no-mag` to ignore the compass.

## Use it with the plotter too

The firmware stream also works with the `imuplot` tool:

```bash
python -m imuplot --port /dev/ttyUSB0 --baud 460800 \
  --fields t_ms,ax,ay,az,gx,gy,gz,mx,my,mz,pres,temp,alt \
  --panel "Accel=ax,ay,az" --panel "Gyro=gx,gy,gz" --panel "Mag=mx,my,mz"
```

## Notes

* Supported chips (auto-detected): MPU6050/6500/9250 (accel+gyro), ADXL345 (accel),
  HMC5883L / QMC5883L (compass), BMP180 (pressure, temperature, relative altitude).
  Missing sensors are sent as `nan` and the viewer adapts.
* The DHT11 humidity sensor is not read (it needs a GPIO pin number from your board).
* Altitude is relative to the pressure measured at boot.
