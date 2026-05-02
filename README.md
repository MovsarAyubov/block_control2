# block_control2

ESP32 field controller firmware.

## Device modes

Open `idf.py menuconfig` and use:

`Greenhouse block controller -> Device mode`

- `Physical hardware`: reads real sensors and drives real GPIO outputs.
- `Simulated hardware`: generates synthetic telemetry and does not require the sensor/actuator hardware.

## USB bridge

`Greenhouse block controller -> Enable USB serial bridge` exposes a simple line protocol on the ESP32 USB serial port.

The PC host runtime uses this bridge to read/write the same holding registers that the old STM32 path expected, while Flutter continues to talk to the PC via Modbus TCP.

Protocol examples:

```text
GH PING
GH READ 0 9
GH WRITE 110 1 600 2200
```

Responses are prefixed with `GH`, so host tools can ignore normal ESP-IDF log lines on the same serial port.
