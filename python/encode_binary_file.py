x = open("./.pio/build/esp32doit-devkit-v1/firmware.bin", "rb")
y = x.read()

a = y
# print(a)
mask = (1 << 8) - 1

b = bytearray(len(a))
for i in range(len(a)):
    b[i] = ((a[i] << 4) | (a[i] >> 4)) & mask
# print(b)

with open('./.pio/build/esp32doit-devkit-v1/encoded_firmware.TMRbin', 'wb') as f:
    f.write(b)

f.close()  # Close the file
x.close()  # Close the file