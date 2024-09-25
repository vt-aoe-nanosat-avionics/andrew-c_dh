# Python TX Command

A Python script to transmit TAB commands over a serial connection

Usage:
* Ensure the dependencies outlined in the Python examples [README](../README.md)
  are installed.
* Ensure the [setup_dependencies.sh](../scripts/setup_dependencies.sh) has run
  successfully.
* Then execute the following `bash` commands:

```bash
source ../p3-env/bin/activate
python3 tx_command.py /dev/ttyUSB0
```

Once the script is running, an arrow will appear on the left side of the terminal, indicating that commands can be entered. The Following Commands are valid however not every command is fully implemented

- common_ack
- common_nack
- common_debug
- common_data
- bootloader_ack
- bootloader_nack
- bootloader_ping
- bootloader_erase
- bootloader_write_page
- bootloader_write_page_addr32
- bootloader_jump
- bootloader_sleep

A more in depth explaination of each command can be found in the [Documentation](../../DOCUMENTATION.md)

To deactivate the Python virtual environment, use the following command:

```bash
deactivate
```

## License

Written by Andrew McGrellis  
Other contributors: None

See the top-level LICENSE file for the license.
