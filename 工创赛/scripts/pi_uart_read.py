import serial, time
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)
s.reset_input_buffer()
end = time.time() + 40
buf = b''
while time.time() < end:
    buf += s.read(64)
print("heard:", buf.hex() if buf else "(none)", flush=True)
s.close()
