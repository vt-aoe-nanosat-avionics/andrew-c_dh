# Tartan Artibeus Experiment Board Low Power Software

```bash
cd ../../scripts/
source sourcefile.txt
cd ../software/low-power/
make
st-flash write low-power.bin 0x8000000
```

## Connecting USART

1. Connect the RX pin of the FTDI adapter to the TX pin of the ta-expt board
2. Connect the TX pin of the FTDI adapter to the RX pin of the ta-expt board
3. Connect the GND pin of the FTDI adapter to the GND pin of the ta-expt board
4. Setup the serial port in minicom so data can be sent to the board. To setup the serial port follow these instructions

```bash
minicom -s ta-expt
# Serial port setup
#   Serial Device: /dev/ttyUSB0
#   Bps/Par/Bits:  38400 8N1
#   Hardware Flow Control: No
#   Software Flow Control: No
#   Press Enter to accept
# Test that the configuration was successful:
minicom ta-expt
# Local echo (shows the chars you send over serial):
#   Ctrl-a e
# Exit minicom:
#   Ctrl-a x
```

## Running The Demo

Once the serial port is setup and the ta-expt connected, the board can be powered. Once the board turns on the leds will blink a few times before turning off completely. The LEDs will stay off until any data is sent over the USART. When the board recieves data from the USART, minicom will display 'Woke Up' and the LEDs will flash again. This cycle repeats indefinitely. To send data over the USART with minicom, just hit any key and it will be sent; waking the board.