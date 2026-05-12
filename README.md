# Very simple LoRa pulse transmitter #
## (with basic self-checks but without any error correction and safety) ##

The need was to transmit S0 pulses (i.e. energy meters open collector output pulses, similar to the red blinky diode you might see on your smart meter), from A to B.

The problem? The distance was too far for a cable. UTP would have to be joined in the middle, as one roll would not be enough. 

### Hardware: ###
* LilyGo T3S3 LoRa ESP32, with SX1262 radio, simple antenna. Basically, any LoRa Radio will do, but update PlatformIO accordingly!
* PC817 Optocoupler-based module for receiving the impulses, ready with resistors and LEDs, great thing! Extremely cheap and allows for the quickest setup of this kind of connection.
* AOD4184 Mosfet-based module for transmitting a 12V signal to further devices at the destination. To put it lightly, not the best choice, but it is what I had. You can use anything, even a relay.
* Basic power supply (230V->12V->5v), and that is it generally

### Using it: ###
If, for some godforsaken reason, you need this exact problem solved, I hereby grant you the ability to do this. I do not, however, take responsibility for any frustration that stems from it. 
This is not a particularly adaptable piece of software, nor is it an effect of years of experience (I do not have much in radio). 
It is, however, a remarkably well-working piece.

Once implemented at the target site, I will keep a (manual) counter of how reliable it is. I will also try to upload some photos if I get permission to do it. 
