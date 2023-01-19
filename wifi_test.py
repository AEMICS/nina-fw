import network

wifi = network.WLAN(network.STA_IF)

wifi.active(True)
retval = wifi.scan()

for name in retval:
    print(name[0])

ssid = 'AEMICS_GUEST'
key = 'speciallyforyou'
wifi.connect(ssid, key)
# Error not connected...
wifi.status()
wifi.ifconfig()
