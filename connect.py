import network
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
ssid = 'AEMICS_GUEST'
key = 'speciallyforyou'
wifi.connect(ssid, key)
