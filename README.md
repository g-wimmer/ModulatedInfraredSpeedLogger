# Modulated Infrared Speed Logger
The Modulated Infrared Speed Logger or M.I.S.L. is a low-profile, battery powered, modulated infrared speedometer for [Fringe's](https://fringe.org/) CMU [buggies](https://www.cmu.edu/buggy/). 

## Context
The speed of a buggy is arguably one of the most important factors in determining the winner of a buggy race. Having an accurate measure of speed would also be useful in determining how well each pusher performs. Being able to measure measure how different buggy engineering choices affect top speed would allow for data-driven designs.
Other teams have tackled this challenge through the use of GPS beacons to track location and speed at the same time. However, if you have used a map on your phone, you likely know that GPS isn't perfectly accurate and can jump around, leading to inaccurate speed values.

## The Problem
Speed is currently measured by eyeballing speed from the follow car's speedometer or, during race day, when Sweepstakes sets up a speed checkpoint near the chute turn of the course.
Currently there is:
- No accurate measure of speed (other teams infer speed through GPS data)
- No continuous logging of speed throughout the entire roll
- No accurate measure of each pusher's pushing strength (besides measuring the total time to complete a hill) 

What is needed: a device that can accurately measure and store real buggy speeds while the buggy rolls, that can fit within the constrained spaces within a buggy, does not add extra mass or interfere with the wheel and can survive the intense vibrations of a buggy rolling at top speed.

## Design
**Space Constraints:** Fringe's buggies are generally reverse trikes, meaning there are 2 wheels in the front and 1 in the rear. This rear wheel is fully enclosed within the buggy and does not turn for steering and is an optimal choice to place a device to measure its speed. However, each buggy’s shell shape is unique and there is very little space around the rear wheel as the driver's feet can go right up against the front of the rear frame holding the wheel. This leaves the <1in gaps besides the wheel and the ~2in gap behind the wheel. Getting the device into place also means not all of that space is usable, since the device must fit through the tight rear hatch.

**Microprocessor:** Arduino Pro Mini 3.3V (ATmega328P, 8 MHz) — low profile and has a low current draw while still having enough pins to allow for all of the device's capabilities.

**Power:** 3 AAA Batteries in series - 4.5V at full charge, feeding the Arduino's voltage regulator. 47µF tantalum capacitor + 0.1µF ceramic across the SD card reader to accommodate its sporadic, but high current draw.
- Use alkaline AAAs only. NiMH cells (about 3.6V for three) are too low for the voltage regulator.
 
**Storage:** Datapoints are buffered in RAM and then written to the microSD card every 2 seconds via the SPI SD card reader. A new CSV file is created per saved run in the format of (`RUN001.CSV`, `RUN002.CSV`, ...).

**Measurement:** Infrared was chosen as it wouldn't require additional mass attached to the wheel (like a hall effect sensor would). An IR LED emits a carrier signal at 30kHz, driven by an NPN transistor. The light is sent through the gaps between the spokes of the wheel where it is received by a phototransistor. The spokes obscure the signal, which is used to determine whether a spoke is passing by. Modulated IR was chosen to allow the device to work regardless of ambient light conditions.

**Signal Processing:** The phototransistor is connected to a AC-coupler/high-pass filter to isolate the 30kHz carrier signal and remove any noise from ambient light.
Prototypes of the device made use of a gain stage, half-wave rectifier and envelope detector, but the gain was unnecessary and the envelope detector didn't function consistently enough at these voltage levels (a diode needs about 0.2-0.3V to conduct, and the signal swing is about 0.3V p-p). The gain stage was converted into a unity-gain buffer (LM358) to isolate the high-impedance AC-coupling node from the comparator’s hysteresis network. The signal was then fed into a comparator (LM393) set up as a Schmitt trigger. This results in a clean toggling, on-off, signal. This digital signal was then fed into the Arduino Pro Mini's hardware counter to count the number of flips (pulses from the IR LED).

**Case and Potting:** The case is printed in PLA+ filament. The rear portion of the case holds the Arduino, SD card reader, display and other circuitry. The battery case is mounted to the right side. Wires extending from the rear portion of the case go to the IR LED and phototransistor positioned opposite each other on the sides of the wheel well. The rear portion of the case is potted in a 2-part silicone compound (McMaster-Carr 74965A52) chosen for its gel-like consistency to reduce vibration.

## How it Works

The code (`speedometer_logger.ino`) is built around one idea: **a spoke is detected when the carrier signal disappears.** While the beam is clear, the comparator toggles at the carrier frequency, about 30,000 times a second. When a spoke blocks the beam, the toggling stops. The Arduino counts the toggles in hardware, checks the count every 200 µs, and turns "the toggling just stopped" into a timestamp for each spoke. Both the speed readout and the log file are built from those timestamps.
 
```
Timer2 carrier (D3) -> IR LED -> [wheel] -> phototransistor -> analog circuit -> comparator -> D5
   -> Timer1 counts edges -> Timer0 checks every 200 µs -> spoke detector -> buffer -> speed / display / SD card
```
 
| Pin | Function |
|---|---|
| D2 | Record button (to GND), interrupt |
| D3 | 30 kHz carrier out to the LED driver transistor (Timer2) |
| D5 | Comparator output, clock input of the edge counter (Timer1) |
| D10-D13 | SD card reader (CS, MOSI, MISO, SCK) |
| A4 / A5 | Display (I2C data / clock) |
 
**Carrier:** Timer2 runs as a hardware PWM output on D3 at about 30 kHz and 50% duty. It switches the NPN transistor that drives the IR LED and keeps running on its own, without using the processor.
 
**Counting:** Timer1 uses D5 as its clock input, so it counts every rising edge from the comparator in hardware. The code never has to react to individual edges.
 
**Sampling:** A Timer0 interrupt fires every 200 µs (one "window") and reads how many edges Timer1 counted since the last window. A clear beam gives about 6 edges per window. A blocked beam gives 0, with an occasional stray single edge.
 
**Clear or blocked?** Each window is classified:
- 3 or more edges: beam clear.
- 1 or fewer edges for 2 windows in a row (0.4 ms): beam blocked, so a spoke is passing.
- Exactly 2 edges: unsure, so the previous state is kept.

The 2-window rule keeps stray edges from being counted as spokes, and it still catches every spoke at 50 mph, where a spoke blocks the beam for about 1.2 ms.
 
**Spoke events:** A spoke is recorded at the moment the beam goes from clear to blocked. The timestamp (in 200 µs ticks) goes into a 64-entry buffer, and events closer than 3 ms apart are discarded as noise. Detection happens inside the interrupt and the main loop only empties the buffer, so slow SD card writes cannot make the device miss a spoke.
 
**Speed:** The wheel has 5 spokes and one rotation of the 7.2 in wheel is π × 7.2 in = 22.6 in, so each spoke is 4.52 in of rim travel:
 
`speed (mph) = 4.52 in ÷ time between spokes (s) ÷ 17.6`
 
- The display updates every 250 ms using the average spoke spacing over that period.
- With no spoke for 1 second the speed reads 0, and the reading is not allowed to exceed what the time since the last spoke permits, so it fades smoothly as the buggy stops.
- At very low speeds a spoke edge can sit in the beam and cause a double trigger. The displayed speed ignores any interval shorter than 40% of the previous one (a 2.5x speed-up in a single spoke) and re-syncs if that happens 3 times in a row. This only affects the number on the display: every spoke is still logged.

**Display:** The 4-digit display shows speed like `40.0`. The device runs a logo sweep for about 3.5 seconds at startup, and the SD card is initialized, and shows `NOSD` if no card is found.
 
**Recording:** The button on D2 starts and stops a recording (debounced, 250 ms).
- Start: the next free file (`RUN001.CSV`, `RUN002.CSV`, ...) is created, and an `R` appears in the leftmost digit.
- During the run: every spoke is written to the file, and the file is saved to the card every 2 seconds, so a power loss only costs the last couple of seconds.
- Stop: the file is closed and the highest speed of the run (best 250 ms average) alternates between `HI` and its value for 10 seconds, then the display goes back to live speed.
- If no card is detected when the button is pressed, `NOSD` is shown and a card inserted later is picked up on the next press.

**Log File:** Each row of the CSV is one spoke:
 
| Column | Meaning |
|---|---|
| `spoke` | Spoke counter, starting at 1 each run (5 per wheel rotation) |
| `time_ms` | Milliseconds since the button was pressed, in 0.2 ms steps |
| `dt_ms` | Time since the previous spoke (blank on the first row) |
| `used` | 1 = counted toward the displayed speed, 0 = ignored by the low-speed filter (still logged) |
 
To get accurate speed from the log, treat the rows as position markers: plot `spoke` × 4.52 in against `time_ms`, smooth it, and differentiate. A single `dt_ms` is only good to about ±1.5% at 20 mph and ±3% at 40 mph because of the 0.2 ms steps, so smoothing over several spokes gives cleaner results. The real distance per spoke (polyurethane squish included) can be measured by rolling the buggy a known distance and dividing by the number of spokes logged.
 
**Firmware Notes:**
- Board setting: Tools > Board > Arduino Pro or Pro Mini, Processor ATmega328P (3.3V, 8 MHz). All of the timing assumes 8 MHz, and the sketch will refuse to compile with any other clock.
- Libraries: Adafruit GFX, Adafruit BusIO, Adafruit LED Backpack, and SdFat (version 2.x).
- Timer0 is used for the 200 µs sampling interrupt, so `millis()` and `delay()` still work, but `analogWrite()` on pins 5 and 6 does not.
- Settings at the top of the sketch: `CARRIER_HZ`, `CLEAR_MIN`, `BLOCK_MAX`, `BLOCK_CONFIRM`, `MIN_INTERVAL_PCT`, `WHEEL_DIAMETER_IN`, and `SPOKES`. `WINDOW_US` must stay at 200 since the log timestamps assume it.

## Usage
0. If you wish to log the data, ensure that a FAT32-formatted microSD card is in place. (other file system formats will not work) When turning on the device, a `NOSD` warning will appear if the SD card is not detected.

1. Remove the rear wheel and rear wheel bolt. Place the device over the rear wheel frame. For some buggies the display may need to be slid out of place to get in to the rear hatch. For other buggies, the device may need to go in through the front and slid all the way to the back. 
    - Be careful to not allow the wires to get caught on anything as they could be easily yanked out and are near impossible to reattach to the core of the device.

2. Once in place, put the rear wheel in place and tighten down the rear wheel bolt (ensuring it goes through the opening in the M.I.S.L.'s case).

3. Flip the power switch to turn on the device. It will immediatly begin displaying the speed. 

4. If you wish to record the speed data or find the high speed of the buggy, press the small button on the right side of the case. An `R` should appear in the leftmost display segment indicating that a recording has started. Upon pressing the button again the recording will stop and the highest speed of the run will be shown for 10 seconds.

5. Remove the SD card for viewing and processing the data.


## Troubleshooting
- If the device is reading `00.0` even if a wheel is spinning, the most likely cause is the phototransistor being obscured. Take a q-tip and clean out the phototransistor and LED holding tubes. If that doesnt work, use a small amount of IPA on the q-tip. As a last resort, use a very small amount of acetone on the q-tip to clean the phototransistor's surface. (make sure to dry quickly)

- If the device is powered on, but the display does not turn on, the most likely cause is a display wire snapping. (the red LED inside of the case will still be on if this is the case) The display wires arent potted and are likely to break. Simply resolder it to the back of the display.

- If a recording was started before a roll, but upon the roll's completion the display states that it is no longer recording, the most likely cause is that the batteries (due to the vibration of the buggy) lost contact with the leads momentarily, causing a restart of the device. To remedy, either tape the batteries into place, or use metal shims to compress the battery springs.

- If the display shows `NOSD` at startup, ensure that a microSD card is inserted and that the SD card is formatted with the FAT32 filesystem. Generally, lower size SD cards should be used as they work better with FAT32 and draw less current.
    - The device will still display speeds without the SD card.

- If the display appears dim or the device appears to restart itself, replace the batteries.
