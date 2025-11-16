x = open("./.pio/build/esp32doit-devkit-v1/encoded_firmware.TMRbin", "rb")
# x = open("./.pio/build/esp32doit-devkit-v1/firmware.bin", "rb")
y = x.read()

# print(y)

x.close()  # Close the file