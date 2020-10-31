VISCA Wiring
============

VISCA DIN serial cables can be either straight through (1-->1, 2-->2, etc.),
or cross-over (TX(3)->RX(5), DTR(1)->DSR(2)).
As far as I can tell, controller to camera cables tend to be straight through.
Camera daisy chain cables seem to be crossover.

VISCA Cable Pinout
------------------

| Signal     | DIN-8 | DIN-8 Crossover | DB9 |
|------------|-------|-----------------|-----|
| DTR        | 1     | 2               | 6   |
| DSR        | 2     | 1               | 4   |
| TXD        | 3     | 5               | 2   |
| GND        | 4     | 4               | 5   |
| RXD        | 5     | 3               | 3   |
| GND        | 6     | 6               |     |
| IROUT      | 7     |                 |     |
| MD Caution | 8     |                 |     |
