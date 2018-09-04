from RFM69.radio import Radio
from RFM69.registers import RF69_868MHZ
import datetime
import time
print("1")
node_id = 1
network_id = 100
recipient_id = 2
print("2")
with Radio(RF69_868MHZ, node_id, network_id, verbose=True) as radio:
    print("3")
    print(radio.read_temperature())
    print(radio.read_registers())
    rx_counter = 0

    while True:

        # Every 10 seconds get packets
        if rx_counter > 10:
            rx_counter = 0

            # Process packets
            for packet in radio.get_packets():
                print(packet)

        print("Listening...", len(radio.packets), radio.mode_name)
        delay = 0.5
        rx_counter += delay
        time.sleep(delay)