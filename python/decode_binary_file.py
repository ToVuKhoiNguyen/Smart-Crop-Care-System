x = open("./.pio/build/esp32doit-devkit-v1/encoded_firmware.TMRbin", "rb")
y = x.read()

a = y
mask = (1 << 8) - 1

b = bytearray(len(a))
for i in range(len(a)):
    b[i] = ((a[i] << 4) | (a[i] >> 4)) & mask
print(b)

x.close()  # Close the file