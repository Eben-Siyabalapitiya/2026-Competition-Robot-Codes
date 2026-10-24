# Bluetooth testing

1. Connect the robot by USB temporarily and open sketches/MiniBot_Bluetooth/MiniBot_Bluetooth.ino in Arduino IDE. Select ESP32 Dev Module from **esp32 by Espressif Systems**, version 3.3.11, and the robot USB port. Upload. The existing AUTO sketch is still the ESP-NOW version; do not upload that for Bluetooth testing.
2. Disconnect robot USB, power it normally, and leave the camera connected. The dongle is no longer used.
3. In Windows Settings > Bluetooth & devices > Add device > Bluetooth, pair **MiniBot-Control**. Confirm any pairing prompt yourself. In More Bluetooth settings > COM Ports, find its **Outgoing** COM port.
4. In VS Code press F5 and choose **MiniBot Bluetooth control page**, or double-click MiniBot Control.bat in MiniBot/shortcuts. Open http://localhost:8000.
5. Refresh ports on the page, select the outgoing port, and Connect Bluetooth. Wait for **Robot connected** and a battery voltage. An open COM port alone is not proof of connection.
6. With wheels raised, press Enable: robot LEDs should stay blue and page should report ENABLED after receiving robot status. Disable should make them red. Missing camera pose prevents driving.
7. Calibrate with the tag visible, zero heading facing field +X, then choose a nearby destination. Keep wheels raised for initial direction checks. A disconnected Bluetooth link disables within 900 ms; stale pose stops drive and climb. The firmware stops after 90 seconds from Enable.

The page shows actual robot enabled status. Reconnects never automatically enable it. No physical Bluetooth/radio/motor testing has been performed by the software checks. Existing platform geometry remains unverified; test flat-floor destinations first. The previous idle-twitch changes are included but need hardware confirmation.
