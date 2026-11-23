# Wiring the XL330-M288

Use the **three-pin TTL** connection and a 5 V regulated servo supply sized for the connected servos. The XL330-M288’s specified supply range is 3.7–6.0 V. Keep supply drops and transients within that range, including when several joints start or brake together. Use the power connector and pinout shown for the M288. [XL330-M288 hardware reference](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/)

With servo power disconnected, connect the adapter’s TTL data and ground to the matching bus signals. Connect the servo supply’s ground to the same reference and route its positive output to the servo power rail. Check polarity at the connector before applying power. The U2D2 handles data and leaves servo power to the external supply. [U2D2 wiring reference](https://emanual.robotis.com/docs/en/parts/interface/u2d2/)

| Bus signal | Connect to |
| --- | --- |
| DATA | TTL data at the adapter and DATA on each servo |
| GND | Adapter ground, supply negative and GND on each servo |
| VDD | Regulated supply positive and VDD on each servo |

For first installation, connect one servo at a time. Once the servos are installed, daisy-chain only units with unique IDs. Use one bus master and keep the data and ground connections secure. Disconnect power before changing cables. See [adapters](adapters.md) for port names and baud-rate settings.

During motion tests, secure the servo, keep people and cables clear of its travel, and support any mechanism that would drop when torque is switched off. Arrange a way to cut servo power without reaching into the moving assembly.
