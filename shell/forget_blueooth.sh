bluetoothctl -- remove D1:FC:A0:92:D5:19
sleep 0.1s
bluetoothctl -- remove EC:73:F2:19:0E:CA
sleep 0.1s
bluetoothctl -- remove C8:2E:9A:E9:37:16
sleep 0.1s
bluetoothctl -- power off
sleep 0.1s
bluetoothctl -- power on
sleep 0.1s
bluetoothctl -- scan on
sleep 0.1s
bluetoothctl -- pair D1:FC:A0:92:D5:19
sleep 0.1s
bluetoothctl -- pair EC:73:F2:19:0E:CA
sleep 0.1s
bluetoothctl -- pair C8:2E:9A:E9:37:16
sleep 0.1s
# bluetoothctl -- connect D1:FC:A0:92:D5:19
# sleep 0.5s
# bluetoothctl -- connect EC:73:F2:19:0E:CA
# sleep 0.5s
# bluetoothctl -- connect C8:2E:9A:E9:37:16
# sleep 1s
bluetoothctl -- exit