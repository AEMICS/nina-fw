"""
@author: grobben
"""


from machine import SPI, Pin
import time

spi = SPI(2, baudrate=400000)


rstn = Pin(Pin.cpu.C1, mode=Pin.OUT, value=1)
cs = Pin(Pin.cpu.B12, mode=Pin.OUT, value=1)
drdy = Pin(Pin.cpu.C0, mode=Pin.IN)

rstn.low()
time.sleep_ms(8)
rstn.high()

txdata1 =b"\xE0\x37\x00\xEE\xFF\xFF\xFF"
txdata2 =b"\x00\x00\x00"
rxdata2 = bytearray(3)
spi.write(txdata2)
length = 0
#for _ in range(0, 5):
try:
    rstn.low()
    time.sleep_ms(8)
    rstn.high()
    time.sleep(1)
    while drdy.value():
        pass
    cs(0)
    spi.write(txdata1)
    cs(1)
    while drdy.value():
        pass
    cs(0)
    spi.write_readinto(txdata2, rxdata2)
    length = 3

    if rxdata2[1] != (0x37 |(1 << 7)):
        padding = spi.read(1)
        print(f'{rxdata2}, pad:{padding}')
    else:
        print(f'{rxdata2}')


    size = spi.read(1)
    print(f'size {size}')
    size = int.from_bytes(size, 'little')
    data = b''
    for j in range(size):
        data += spi.read(1)
        length += 1

    print(data)

    for g in range((length+1) % 4 + 1):
        spi.read(1)

finally:
    cs(1)
    #print(rxdata2)
    time.sleep_ms(1)
