# tx_example.py
#
# Usage: python3 tx_example.py /path/to/dev
# Parameters:
#  /path/to/dev: path to device, e.g. /dev/ttyUSB0
# Output:
#  Prints results to the command line
#
# Written by Bradley Denby
# Other contributors: Chad Taylor
#
# See the top-level LICENSE file for the license.

# import Python modules
import os     # path
import serial # serial
import sys    # accessing script arguments
import time   # sleep
import datetime # time
import math   # math

# Make Python TAB implementation visible
sys.path.append(os.path.abspath('../../python-implementation/'))

# import TAB support
from tab import *

################################################################################

# initialize script arguments
dev = '' # serial device

# parse script arguments
if len(sys.argv)==2:
  dev = sys.argv[1]
else:
  print(\
   'Usage: '\
   'python3 tx_example.py '\
   '/path/to/dev'\
  )
  exit()

# Create serial object
try:
  serial_port = serial.Serial(port=dev,baudrate=115200)
except:
  print('Serial port object creation failed:')
  print('  '+dev)
  exit()

################################################################################

# Set up test support variables
HWID = 0x0012
msgid = 0x0000
rx_cmd_buff = RxCmdBuff()

while(1):
  command = input(">")
  code = command.split(' ')[0]
  opts = command.split(' ')[1:]
  if code == "common_ack":
    cmd = TxCmd(COMMON_ACK_OPCODE, HWID, msgid, GND, CDH)
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)
  
  elif code == "common_nack":
    cmd = TxCmd(COMMON_NACK_OPCODE, HWID, msgid, GND, CDH)
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)

  elif code == "common_debug":
    cmd = TxCmd(COMMON_DEBUG_OPCODE, HWID, msgid, GND, CDH)
    cmd.common_debug('Hello, world!')
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)

  elif code == "common_data":
    cmd = TxCmd(COMMON_DATA_OPCODE, HWID, msgid, GND, CDH)
    cmd.common_data([0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b])
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)

  elif code == "bootloader_ack":
    cmd = TxCmd(BOOTLOADER_ACK_OPCODE, HWID, msgid, GND, CDH)
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)


  elif code == "bootloader_nack":
    cmd = TxCmd(BOOTLOADER_NACK_OPCODE, HWID, msgid, GND, CDH)
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)

  elif code == "bootloader_ping":
    cmd = TxCmd(BOOTLOADER_PING_OPCODE, HWID, msgid, GND, CDH)
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)

  elif code == "bootloader_erase":
    cmd = TxCmd(BOOTLOADER_ERASE_OPCODE, HWID, msgid, GND, CDH)
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)

  elif code == "bootloader_write_page":
    cmd = TxCmd(BOOTLOADER_WRITE_PAGE_OPCODE, HWID, msgid, GND, CDH)
    cmd.bootloader_write_page(page_number=0, page_data=128*[0])
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)

  elif code == "bootloader_write_page_addr32":
    cmd = TxCmd(BOOTLOADER_WRITE_PAGE_ADDR32_OPCODE, HWID, msgid, GND, CDH)
    cmd.bootloader_write_page_addr32(addr=0x08008000, page_data=128*[0])
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)

  elif code == "bootloader_jump":
    cmd = TxCmd(BOOTLOADER_JUMP_OPCODE, HWID, msgid, GND, CDH)
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)

  elif code == "bootloader_power":
    cmd = TxCmd(BOOTLOADER_POWER_OPCODE, HWID, msgid, GND, CDH)
    cmd.bootloader_power_select(opts[0])
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)

  elif code == "app_get_telem":
    cmd = TxCmd(COMMON_ASCII_OPCODE, HWID, msgid, GND, CDH)
    tle  = 'TLE'
    tle += 'FLOCK 3K-5              '
    tle += '1 43899U 18111Z   21284.66246111  .00014637  00000-0  51582-3 0  9994'
    tle += '2 43899  97.2179 176.7560 0018058 232.7758 127.1835 15.29226533155475'
    cmd.common_ascii(tle)
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)

  elif code == "app_get_time":
    cmd = TxCmd(APP_GET_TIME_OPCODE, HWID, msgid, GND, CDH)
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)

  elif code == "app_reboot":
    continue

  elif code == "app_set_time":
    cmd = TxCmd(APP_SET_TIME_OPCODE, HWID, msgid, GND, CDH)
    td = datetime.datetime.now(tz=datetime.timezone.utc) - J2000
    cmd.app_set_time(sec=math.floor(td.total_seconds()), ns=(td.microseconds*1000))
    byte_i = 0
    while rx_cmd_buff.state != RxCmdBuffState.COMPLETE:
      if byte_i < cmd.get_byte_count():
        serial_port.write(cmd.data[byte_i].to_bytes(1, byteorder='big'))
        byte_i += 1
      if serial_port.in_waiting>0:
        bytes = serial_port.read(1)
        for b in bytes:
          rx_cmd_buff.append_byte(b)
    print('txcmd: '+str(cmd))
    print('reply: '+str(rx_cmd_buff)+'\n')
    cmd.clear()
    rx_cmd_buff.clear()
    msgid += 1
    time.sleep(1.0)

  elif code == "exit":
    break

  else:
    print("Invalid command")


