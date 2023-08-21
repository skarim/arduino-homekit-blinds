# Smart Blinds

Motorized blinds using ESP 32 board & stepper motor run by TMC 2209 driver.
Connects to HomeKit via [Homebridge](https://github.com/homebridge/homebridge).

---

## Getting Started

### Install dependencies

This project uses [PlatformIO](https://platformio.org) for managing packages & dependencies.

If you don't have it already, [install the platformio cli](https://docs.platformio.org/en/latest/installation.html).

Dependencies are listed in the `platformio.ini` file. Run this in the project root folder to install packages:
```
platformio lib install
```

### Set configurations

Make a copy of the sample config file and edit as needed:
```
cp Config.sample.h Config.h
vi Config.h
```

- Enter your WiFi SSID and password
- Define the number of total rotations it takes to go from fully open to fully closed
- Adjust your speed for opening and closing — typically you will need to go slower during opening because it requires more torque to pull up your blinds
- The `MAX_SPIN_TIME` is used to set the Task Watchdog Timer (TWDT) so FreeRTOS doesn't kill the task for controlling the stepper motor driver while it is running

### Build & upload to board
```
platformio run --target upload
```

Once the code has been uploaded to your board, you can view the serial monitor output using:
```
platformio device monitor
```

---

## Usage

On start, the board will print out its local IP address in serial monitor. Take note of this IP, since you'll need it for later. I'd recommend going into your router settings and assigning the board a static IP.

### Get current position
`GET /position`

Returns a plain-text response with a float (0.0 to 100.0) of where the blinds are currently at. 0 is closed, 100 is fully open.

### Set Position
`POST /set?position=N`

POST to this endpoint (no payload required, just pass in position as URL query param).

Accepts an integer from 0 to 100.

Returns a 204 on success.

---

## Connecting to HomeKit

I used [Homebridge](https://github.com/homebridge/homebridge), running on a Raspberry Pi, to act as a bridge between the board & HomeKit / Siri. Follow the instructions to setup and configure Homebridge for your home.
```
sudo npm install -g homebridge
```

Once you have Homebridge set up, you'll need to install the [Homebridge Blinds](https://github.com/dxdc/homebridge-blinds) plugin to tell Homebridge how to interact with your board.
```
sudo npm install -g homebridge-blinds
```

In your homebridge `config.json`, add the following to your accessories:
```json
"accessories": [
    {
        "name": "My Blinds",
        "accessory": "BlindsHTTP",
        "up_url": {
            "url": "http://YOUR_BOARD_LOCAL_IP/set?position=%%POS%%",
            "method": "POST"
        },
        "down_url": {
            "url": "http://YOUR_BOARD_LOCAL_IP/set?position=%%POS%%",
            "method": "POST"
        },
        "pos_url": "http://YOUR_BOARD_LOCAL_IP/position",
        "pos_poll_ms": 5000,
        "http_success_codes": [
            200,
            204
        ],
        "motion_time_graph": {
            "up": [
                {
                    "pos": 0,
                    "seconds": 0
                },
                {
                    "pos": 100,
                    "seconds": 125 // how long it takes for your blinds to go up
                }
            ],
            "down": [
                {
                    "pos": 100,
                    "seconds": 0
                },
                {
                    "pos": 0,
                    "seconds": 80 // how long it takes for your blinds to go down
                }
            ]
        },
        "unique_serial": false
    }
]
```
