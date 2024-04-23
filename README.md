# ECE 3760 - B01 - 1

> "Our aim is to help level the playing field for deaf curlers. Our product allows _skip_ and _sweepers_ to easily communicate non-verbally through **intuitive**, **non-invasice**, and cost **cost-effective** design."

## About The Project

All code was initially done on @Kameroni33's GitHub, added to here is my personal lab book for the term which received the best mark in the class.\

Our group's project won the award for best equitable design in the class.

### Project Setup

Clone the project
```shell
git clone git@github.com:Kameroni33/ECE3760-B01-1.git  # NOTE: requires a configured SSH key (alternatively use HTTPS)
```

> **Note:** this repository is public, but in order to contribute to the source code, you will need to be added as a _contributor_. For repository access, please email your GitHub user information to [ronaldk1@myumanitoba.ca]().

Download and install [Arduino IDE](https://www.arduino.cc/en/software) for your operating system.

In order to use the NeoPixel library, you must install the **Adafruit NeoPixel** library which can be done from within the Arduino IDE. For further instructions, check out the [Adafruit_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) GitHub repository.

### Physical Configuration

For this project, we are using an _ESP32 WeMos LOLIN32 Development Board_. The pin connections for this project are shown below.
```
                                      ┏━━━━━━━━━━━┓
                                  [ ] ┃   WIFI    ┃
                                  [ ] ┃ ┏━━━━━━━┓ ┃ [GPIO_1] LED_SIG (sweep)
                PB_1 (skip) [GPIO_36] ┃ ┃  CPU  ┃ ┃ [ ]
                PB_2 (skip) [GPIO_39] ┃ ┃       ┃ ┃ [3.3V]
   PB_3 / JS_X (sweep/skip) [GPIO_32] ┃ ┗━━━━━━━┛ ┃ [ ]
          JS_Y       (skip) [GPIO_33] ┃ RESET ┏━┓ ┃ [ ]
                                  [ ] ┃       ┗━┛ ┃ [ ]
                                  [ ] ┃  LED  ┏━┓ ┃ [GND] GND
SW_1 / RGB_1_R (sweep/skip) [GPIO_25] ┃       ┗━┛ ┃ [ ]
SW_2 / RGB_1_G (sweep/skip) [GPIO_26] ┃           ┃ [ ]
       RGB_1_B       (skip) [GPIO_27] ┃           ┃ [ ]
       RGB_2_R       (skip) [GPIO_14] ┃           ┃ [ ]
       RGB_2_G       (skip) [GPIO_12] ┃           ┃ [ ]
       RGB_2_B       (skip) [GPIO_13] ┃           ┃ [ ]
                                  [ ] ┃           ┃ [ ]
                                  [ ] ┃           ┃ [GPIO_4] JS_SW (skip)
                                     ┏┻━━┓        ┃ [ ]
                    LiPo Battery (+) ┃ + ┃        ┃ [ ]
                    LiPo Battery (-) ┃ - ┃        ┃ [ ]
                                     ┗┳━━┛ ┏━━━┓  ┃ [ ]
                                      ┗━━━━┫   ┣━━┛
                                           ┗━━━┛
                                         Micro USB (for programming)
```
