from RFM69.radio import Radio
from RFM69.registers import RF69_868MHZ
import datetime
import time

node_id = 1
network_id = 100
recipient_id = 2

with Radio(RF69_868MHZ, node_id, network_id, isHighPower=True, verbose=True) as radio:
    print(radio.read_temperature())
    print(radio.read_registers())