# Smart Blinds

Motorized blinds using ESP 8266 board & MG 996 servo.
Connects to HomeKit via [homebridge](https://github.com/nfarina/homebridge).

---

## Getting Started

### Install dependencies

This project uses [PlatformIO](https://platformio.org/) for managing packages & dependencies.

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

Returns a plain-text response with a float (0.0 to 100.0) of where the blinds are currently at. 100 is fully open.

### Get current state
`GET /state`

Returns a plain-text response with one of three integers:
- 0: blinds are moving towards closing
- 1: blinds are moving towards opening
- 2: blinds are not moving

### Set Position
`POST /set?position=N`

POST to this endpoint (no payload required, just pass in position as URL query param).

Accepts an integer from 0 to 100.

Returns a 204 on success.

---

## Connecting to HomeKit

I used [homebridge](https://github.com/nfarina/homebridge), running on a Raspberry Pi, to act as a bridge between the board & HomeKit / Siri.

```
sudo npm install -g --unsafe-perm homebridge
```

You'll also need the [Minimal HTTP Blinds](https://github.com/Nicnl/homebridge-minimal-http-blinds) plugin to tell homebridge how to interact with your blinds.
```
npm install -g homebridge-minimal-http-blinds
```

In your homebridge `config.json`, add the following to your accessories:
```json
"accessories": [
    {
        "name": "My Blinds",
        "accessory": "MinimalisticHttpBlinds",
        
        "get_current_position_url": "http://YOUR_BOARD_LOCAL_IP/position",
        "set_target_position_url": "http://YOUR_BOARD_LOCAL_IP/set?position=%position%",
        "get_current_state_url": "http://YOUR_BOARD_LOCAL_IP/state"
    }
]
```
