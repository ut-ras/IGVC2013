import serial
import struct

def calc_checksum(inp):
    out = 0
    for i in inp:
        out+= i
    print "checksum: ",out
    return out

with serial.Serial(port = '/dev/UM6-LT', baudrate = 115200) as ser:
    pk = struct.pack('cccBB','s','n','p',0,172) #zero gyro
    pk = pk + struct.pack('H', calc_checksum(struct.unpack('5B',pk)))
    ser.write(pk)
    quit()
    while True:
        i = ser.read(1)[0]
        if i is 's' or i is 'n' or i is 'p':
            print i
        else:
            print ',',ord(i)


