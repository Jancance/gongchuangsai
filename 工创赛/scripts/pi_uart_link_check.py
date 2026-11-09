import serial, time, sys
port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
s = serial.Serial(port, 115200, timeout=0.5)
time.sleep(0.2)
s.reset_input_buffer()
print("opened", port)
s.write(bytes([0xCC, 0x02, 0x01, 0x02, 0x03, 0xDD]))
time.sleep(0.5)
h = s.read(32)
print("heard:", h.hex() if h else "(none)")
s.close()
