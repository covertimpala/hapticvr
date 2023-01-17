from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
#import time
from bleak import BleakError


ble = BLERadio()

uart_connection = None

print("Trying to connect...")
g = "|"
err = ""
r = 0
while True:


    if uart_connection and uart_connection.connected:
        uart_service = uart_connection[UARTService]
        try:
            while uart_connection.connected:
                anglex = 0
                angley = 0
                anglez = 0
                s = "d"
                i = 0
                while i == 0:
                    #uart_service.write(s.encode("utf-8"))
                    b = uart_service.readline().decode("utf-8")
                    #if b:
                    print(b)


        except BleakError:
            print("disconnected")
            uart_connection = None
        #except Exception as n:
         #   print(n)
        
    elif not uart_connection:
        try:
            if r == 1:
                print(f"\tloading: {g}   {err}", end="\r")
            #time.sleep(.2)
            for adv in ble.start_scan(ProvideServicesAdvertisement):
                if UARTService in adv.services:
                    uart_connection = ble.connect(adv)
                    print("Connected")
                    break
            ble.stop_scan()
        except TypeError:
            if g == "|":
                g = "/"
            elif g == "/":
                g = "-"
            elif g == "-":
                g = "\\"
            elif g == "\\":
                g = "|"
            err = "searching for device"
            r = 1
        except Exception as e:
            print(e)
            err == "other error        "
            r = 1

