bluetoothctl -- pair D1:FC:A0:92:D5:19
sleep 0.5s
bluetoothctl -- pair EC:73:F2:19:0E:CA
sleep 0.5s
bluetoothctl -- pair C8:2E:9A:E9:37:16
sleep 0.5s
bluetoothctl -- connect D1:FC:A0:92:D5:19
sleep 0.5s
bluetoothctl -- connect EC:73:F2:19:0E:CA
sleep 0.5s
bluetoothctl -- connect C8:2E:9A:E9:37:16
sleep 1s
bluetoothctl -- exit