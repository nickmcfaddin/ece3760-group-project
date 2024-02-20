# ECE 3760 - B01 - 1

---


## ESP32 WeMos LOLIN32 Development Board

```
                          ┏━━━━━━━━━━━┓
                3.3V [  ] ┃ ┏━━━━━━━┓ ┃ [  ] GND
                  EN [  ] ┃ ┃ CPU   ┃ ┃ [41] GPIO 01
(Input Only) GPIO 36 [05] ┃ ┃       ┃ ┃ [40] GPIO 03
(Input Only) GPIO 39 [08] ┃ ┗━━━━━━━┛ ┃ [  ] 3.3V
             GPIO 32 [12] ┃           ┃ [39] GPIO 22
             GPIO 33 [13] ┃ Reset ┏━┓ ┃ [42] GPIO 21
(Input Only) GPIO 34 [10] ┃       ┗━┛ ┃ [  ] GND
(Input Only) GPIO 35 [11] ┃   LED ┏━┓ ┃ [  ] GND
             GPIO 25 [14] ┃       ┗━┛ ┃ [38] GPIO 19
             GPIO 26 [15] ┃           ┃ [36] GPIO 23
             GPIO 27 [16] ┃           ┃ [35] GPIO 18
             GPIO 14 [17] ┃           ┃ [34] GPIO 05 (On Board LED)
             GPIO 12 [18] ┃           ┃ [  ] 3.3V
             GPIO 13 [20] ┃           ┃ [27] GPIO 17
                  5V [  ] ┃           ┃ [25] GPIO 16
                 GND [  ] ┃           ┃ [24] GPIO 04
                         ┏┻━━┓        ┃ [23] GPIO 00
        LiPo Battery (+) ┃ + ┃        ┃ [  ] GND
        LiPo Battery (-) ┃ - ┃        ┃ [22] GPIO 02
                         ┗┳━━┛ ┏━━━┓  ┃ [21] GPIO 15
                          ┗━━━━┫   ┣━━┛
                               ┗━━━┛
                             Micro USB
```
