import usb.core
import usb.util
import sys

VENDOR_ID = 0x7513
PRODUCT_ID = 0x0002

dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)

if dev is None:
    sys.exit("Could not find Id System Barcode Reader.")
else:
    print('Device detected')



cmd_string = 'LISTPOINTS\r\n'

print("Write:", cmd_string, dev.write(4,cmd_string))

msg = None
while True:
# msg != cmd_string:
    response =  dev.read(0x83,200)
    msg = ''.join(chr(i) for i in response)
    print("Read:", msg)



#sudo echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="7513", MODE="0666"' >> /etc/udev/rules.d/50-myusb.rules
#R: bottom turning access
#Z: up and down
#p GRIPPER TURNING