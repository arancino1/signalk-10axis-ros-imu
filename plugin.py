"""
  Permission to use, copy, modify, and/or distribute this software for any purpose with
  or without fee is hereby granted, provided that the above copyright notice and this
  permission notice appear in all copies.

  THE SOFTWARE IS PROVIDED “AS IS” AND ISC DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS
  SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT
  SHALL ISC BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY
  DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
  OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH
  THE USE OR PERFORMANCE OF THIS SOFTWARE.

  Copyright © 1995-2003 by Internet Software Consortium
  Copyright © 2004-2013 by Internet Systems Consortium, Inc. (“ISC”)

"""


import sys
import json
import time
import math
import serial
import random
import logging
import threading

import numpy as np

from array import array


# .. global variables

devName = "/dev/ttyUSB0"
devRefresh = 2.0
devHdgOffset = 0.0

CmdPacket_Begin = 0x49
CmdPacket_End = 0x4D
CmdPacketMaxDatSizeRx = 73

i = 0
CS = 0
RxIndex = 0
cmdLen = 0

buf = bytearray(5 + CmdPacketMaxDatSizeRx)


class devConfig():
    def __init__(self, n, r, h):
        self.name = n
        self.refresh = r
        self.hdgOffset = h


def Cmd_RxUnpack(buf, DLen):
    if buf[0] == 0x11:
        ctl = (buf[2] << 8) | buf[1]

        L = 7

        if ((ctl & 0x0001) != 0): L += 6
        if ((ctl & 0x0002) != 0): L += 6
        if ((ctl & 0x0004) != 0): L += 6

        if ((ctl & 0x0008) != 0):
            scaleMag = 0.15106201171875

            tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L + 0]) * scaleMag
            tmpY = np.short((np.short(buf[L + 3]) << 8) | buf[L + 2]) * scaleMag
            #tmpZ = np.short((np.short(buf[L + 5]) << 8) | buf[L + 4]) * scaleMag

            L += 6

            #tmpAbs = math.sqrt((tmpX * tmpX) + (tmpY * tmpY) + (tmpZ * tmpZ))

            magHdg = math.atan2(tmpY, tmpX) * (180.0 / math.pi)

            if (magHdg < 0):
                magHdg = abs(magHdg)
            else:
                magHdg = 360.0 - magHdg

            magHdg += -1.0 	# apply compass installation error

            if (magHdg >= 360.0):
                magHdg -= 360.0
            if (magHdg < 0):
                magHdg += 360.0

            magDev = 0.0

            skData = {'updates': [{ 'values': [{'path': 'navigation.headingCompass', 'value': magHdg * math.pi / 180.0}]}]}
            sys.stdout.write(json.dumps(skData) + '\n' + json.dumps(skData) + '\n')
            skData = {'updates': [{ 'values': [{'path': 'navigation.magneticDeviation', 'value': magDev}]}]}
            sys.stdout.write(json.dumps(skData) + '\n' + json.dumps(skData) + '\n')
            skData = {'updates': [{ 'values': [{'path': 'navigation.headingMagnetic', 'value': magHdg * math.pi / 180.0}]}]}
            sys.stdout.write(json.dumps(skData) + '\n' + json.dumps(skData) + '\n')
            #skData = {'updates': [{ 'values': [{'path': 'navigation.headingTrue', 'value': (hdg - 5.8) * math.pi / 180.0}]}]}
            #sys.stdout.write(json.dumps(skData) + '\n' + json.dumps(skData) + '\n')

            sys.stdout.flush()


        if ((ctl & 0x0010) != 0): L += 8
        if ((ctl & 0x0020) != 0): L += 8

        if ((ctl & 0x0040) != 0):
            scaleAngle = 0.0054931640625

            tmpX = np.short((np.short(buf[L + 1]) << 8) | buf[L + 0]) * scaleAngle
            tmpY = np.short((np.short(buf[L + 3]) << 8) | buf[L + 2]) * scaleAngle
            tmpZ = np.short((np.short(buf[L + 5]) << 8) | buf[L + 4]) * scaleAngle

            L += 6

            roll = 0

            if (tmpX < 0):
                roll = -180.0 - tmpX
            else:
                roll = 180.0 - tmpX

            skData = {'updates': [{ 'values': [{'path': 'navigation.attitude.yaw', 'value': 0.0}]}]}
            sys.stdout.write(json.dumps(skData) + '\n' + json.dumps(skData) + '\n')
            skData = {'updates': [{ 'values': [{'path': 'navigation.attitude.roll', 'value': roll * math.pi / 180.0}]}]}
            sys.stdout.write(json.dumps(skData) + '\n' + json.dumps(skData) + '\n')
            skData = {'updates': [{ 'values': [{'path': 'navigation.attitude.pitch', 'value': -tmpY * math.pi / 180.0}]}]}
            sys.stdout.write(json.dumps(skData) + '\n' + json.dumps(skData) + '\n')

            sys.stdout.flush()


def Cmd_GetPkt(byte):

    global CS, i, RxIndex, buf, cmdLen

    CS += byte

    if RxIndex == 0:
        if byte == CmdPacket_Begin:
            i = 0
            buf[i] = CmdPacket_Begin
            i += 1
            CS = 0
            RxIndex = 1

    elif RxIndex == 1:
        buf[i] = byte
        i += 1
        if byte == 255:
            RxIndex = 0
        else:
            RxIndex += 1

    elif RxIndex == 2:
        buf[i] = byte
        i += 1
        if byte > CmdPacketMaxDatSizeRx or byte == 0:
            RxIndex = 0
        else:
            RxIndex += 1
            cmdLen = byte

    elif RxIndex == 3:
        buf[i] = byte
        i += 1
        if i >= cmdLen + 3:
            RxIndex += 1
    elif RxIndex == 4:
        CS -= byte
        if (CS&0xFF) == byte:
            buf[i] = byte
            i += 1
            RxIndex += 1
        else:
            RxIndex = 0
    elif RxIndex == 5:
        RxIndex = 0
        if byte == CmdPacket_End:
            buf[i] = byte
            i += 1
            hex_string = " " . join(f"{b:02X}" for b in buf[0:i])
            #print(f"U-Rx[Len={i}]:{hex_string}")
            Cmd_RxUnpack(buf[3:i-2], i-5)
            return 1
    else:
        RxIndex = 0
    return 0


def Cmd_PackAndTx(dev, pDat, DLen):
    if DLen == 0 or DLen > 19:
        return -1

    buf = bytearray([0x00] * 46) + bytearray([0x00, 0xff, 0x00, 0xff, 0x49, 0xFF, DLen]) + bytearray(pDat[:DLen])

    CS = sum(buf[51:51 + DLen + 2]) & 0xFF
    buf.append(CS)
    buf.append(0x4D)

    dev.write(buf)
    return 0


def sensor_data_loop(dev, delay):

    isCompassOn = 1
    barometerFilter = 2

    Cmd_ReportTag = 0x48

    params = bytearray([0x00 for i in range(0,11)])

    params[0] = 0x12
    params[1] = 5               # Stationary state acceleration threshold
    params[2] = 255             # Static zero return speed (unit cm/s) 0: No return to zero 255: Return to zero immediately
    params[3] = 0               # Dynamic zero return speed (unit cm/s) 0: No return to zero
    params[4] = ((barometerFilter & 3) << 1) | (isCompassOn & 1);
    params[5] = delay           # transmission frame rate of data actively reported [value 0-250HZ], 0 means 0.5HZ
    params[6] = 1               # Gyroscope filter coefficient [value 0-2], the larger the value, the more stable it is but the worse the real-time performance.
    params[7] = 3               # Accelerometer filter coefficient [value 0-4], the larger the value, the more stable it is but the worse the real-time performance.
    params[8] = 5               # Magnetometer filter coefficient [value 0-9], the larger the value, the more stable it is but the worse the real-time performance.
    params[9] = Cmd_ReportTag & 0xff
    params[10] = (Cmd_ReportTag >> 8) & 0xff

    Cmd_PackAndTx(dev, params, len(params))
    time.sleep(0.2)

    # wake up sensor
    Cmd_PackAndTx(dev, [0x03], 1)
    time.sleep(0.2)

    # enable proactive reporting
    Cmd_PackAndTx(dev, [0x19], 1)

    # start magnetometer calibration
    #Cmd_PackAndTx([0x32], 1)
    #time.sleep(15.0)

    # end magnetometer calibration
    #Cmd_PackAndTx([0x04], 1)

    # Z-axis angle reset to zero
    #Cmd_PackAndTx([0x05], 1)
    #time.sleep(0.5)

    while True:
        data = dev.read(1)
        if len(data) > 0:
            Cmd_GetPkt(data[0])


# main entry (enable logging and check device configurations)

logging.basicConfig(stream = sys.stderr, level = logging.DEBUG)

config = json.loads(input())
myDevList: list[devConfig] = []

for dev in config["rosDevices"]:
    myDevList.append(devConfig(dev["devName"], dev["devRefresh"], dev["devHdgOffset"]))

    ser = serial.Serial(devName, 115200, timeout = 2)

    if ser.isOpen():
        threading.Timer(1.0, sensor_data_loop, [ser, 2]).start()
    else:
        logging.debug("ERROR opening: '" + devName + "'")

for line in iter(sys.stdin.readline, b''):
    try:
        data = json.loads(line)
        sys.stderr.write(json.dumps(data))
    except:
        sys.stderr.write('error parsing json\n')
        sys.stderr.write(line)
