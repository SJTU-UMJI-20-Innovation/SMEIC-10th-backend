# Modified by SJTU-UMJI-20-Innovation. Removed class MLX90640. Disabled broken pixel count logging.

# MIT License
#
# Copyright (c) 2019 hansonCc
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import struct
import math
import time
from grove.i2c import Bus

# This relies on the driver written by siddacious and can be found at:
# https://github.com/adafruit/Adafruit_CircuitPython_MLX90640

eeData = [0] * 832
I2C_READ_LEN = 2048
SCALEALPHA = 0.000001
MLX90640_DEVICEID1 = 0x2407
OPENAIR_TA_SHIFT = 12

class RefreshRate:
    """ Enum-like class for MLX90640's refresh rate """
    REFRESH_0_5_HZ = 0b000  # 0.5Hz
    REFRESH_1_HZ = 0b001  # 1Hz
    REFRESH_2_HZ = 0b010  # 2Hz
    REFRESH_4_HZ = 0b011  # 4Hz
    REFRESH_8_HZ = 0b100  # 8Hz
    REFRESH_16_HZ = 0b101  # 16Hz
    REFRESH_32_HZ = 0b110  # 32Hz
    REFRESH_64_HZ = 0b111  # 64Hz
class MLX9064X_I2C_Driver:
    def __init__(self,address):
        self.bus = Bus()
        self.addr = address

    def I2CWriteWord(self, writeAddress, data):
        write = self.bus.msg.write(self.addr,[writeAddress>>8,writeAddress&0xFF,data>>8,data&0xFF])
        try:
            self.bus.i2c_rdwr(write)
        except OSError:
            print("Error:Please check if the I2C device insert in I2C of Base Hat")
            exit(1)

    def I2CReadWords(self, addr, buffer, *, end=None):
        if end is None:
            remainingWords = len(buffer)
        else:
            remainingWords = end
        write = self.bus.msg.write(self.addr,[addr>>8,addr&0xFF])
        read = self.bus.msg.read(self.addr,2*remainingWords)
        try:
            self.bus.i2c_rdwr(write, read)
        except OSError:
            print("Error:Please check if the I2C device insert in I2C of Base Hat")
            exit(1)
        result = list(read)
        for i in range(remainingWords*2):
            if i % 2 != 0:
                buffer[(i-1)//2] = (result[i-1]<<8)|result[i]&0xff

class grove_mxl90641(MLX9064X_I2C_Driver):
    """Interface to the MLX90640 temperature sensor."""
    kVdd = 0
    vdd25 = 0
    KvPTAT = 0
    KtPTAT = 0
    vPTAT25 = 0
    alphaPTAT = 0
    gainEE = 0
    tgc = 0
    resolutionEE = 0
    KsTa = 0
    ksTo = [0] * 8
    ct = [0] * 8
    alpha = [0] * 192
    alphaScale = 0
    offset = [[0 for i in range(192)] for i in range(2)]
    kta = [0] * 192
    ktaScale = 0
    kv = [0] * 192
    kvScale = 0
    cpAlpha = 0
    cpOffset = 0
    brokenPixels = [0xFFFF] * 5
    cpKta = 0
    cpKv = 0
    emissivityEE = 0
    def __init__(self,address=0x33):
        super(grove_mxl90641, self).__init__(address)
        self.refresh_rate = RefreshRate.REFRESH_0_5_HZ
        self.I2CReadWords(0x2400, eeData)
        self._HammingDecode()
        self._ExtractParameters()

    @property
    def refresh_rate(self):
        """ How fast the MLX90641 will spit out data. Start at lowest speed in
        RefreshRate and then slowly increase I2C clock rate and rate until you
        max out. The sensor does not like it if the I2C host cannot 'keep up'!"""
        controlRegister = [0]
        self.I2CReadWords(0x800D, controlRegister)
        return (controlRegister[0] & 0x0380) & 0x07

    @refresh_rate.setter
    def refresh_rate(self, rate):
        controlRegister = [0]
        value = (rate & 0x7) << 7
        self.I2CReadWords(0x800D, controlRegister)
        value |= controlRegister[0] & 0xFC7F
        self.I2CWriteWord(0x800d, value)

    def getFrame(self, framebuf):
        """ Request both 'halves' of a frame from the sensor, merge them
        and calculate the temperature in C for each of 12x16 pixels. Placed
        into the 192-element array passed in! """
        emissivity = 0.95
        tr = 23.15
        mlx90641Frame = [0] * 242

        for _ in range(2):
            mlx90641Frame = self._GetFrameData()
            if mlx90641Frame[241] < 0:
                raise RuntimeError("Frame data error")
            # For a MLX90641 in the open air the shift is -8 degC.
            tr = self._GetTa(mlx90641Frame) - OPENAIR_TA_SHIFT
            self._CalculateTo(mlx90641Frame, emissivity, tr, framebuf)

    def _GetFrameData(self):
        dataReady = 0
        cnt = 0
        statusRegister = [0]
        controlRegister = [0]
        subPage = 0
        while dataReady == 0:
            self.I2CReadWords(0x8000, statusRegister)
            dataReady = statusRegister[0] & 0x0008
        subPage = statusRegister[0] & 0x0001

        while (dataReady != 0) and (cnt < 5):
            self.I2CWriteWord(0x8000, 0x0030)
            #print("Read frame", cnt)
            recive_data = [0] * 32
            reciver = [0]
            if subPage == 0:
                # print(data)
                self.I2CReadWords(0x0400, recive_data, end=32)
                reciver = reciver + recive_data
                self.I2CReadWords(0x0440, recive_data, end=32)
                reciver = reciver + recive_data
                self.I2CReadWords(0x0480, recive_data, end=32)
                reciver = reciver + recive_data
                self.I2CReadWords(0x04C0, recive_data, end=32)
                reciver = reciver + recive_data
                self.I2CReadWords(0x0500, recive_data, end=32)
                reciver = reciver + recive_data
                self.I2CReadWords(0x0540, recive_data, end=32)
                reciver = reciver + recive_data
            else:
                self.I2CReadWords(0x0420, recive_data, end=32)
                reciver = reciver + recive_data
                self.I2CReadWords(0x0460, recive_data, end=32)
                reciver = reciver + recive_data
                self.I2CReadWords(0x04A0, recive_data, end=32)
                reciver = reciver + recive_data
                self.I2CReadWords(0x04E0, recive_data, end=32)
                reciver = reciver + recive_data
                self.I2CReadWords(0x0520, recive_data, end=32)
                reciver = reciver + recive_data
                self.I2CReadWords(0x0560, recive_data, end=32)
                reciver = reciver + recive_data
            recive_data = [0] * 48
            self.I2CReadWords(0x0580, recive_data, end=48)
            reciver = reciver + recive_data
            self.I2CReadWords(0x8000, statusRegister)
            dataReady = statusRegister[0] & 0x0008
            subPage = statusRegister[0] & 0x0001
            cnt += 1
        if cnt > 4:
            raise RuntimeError("Too many retries")
        self.I2CReadWords(0x800D, controlRegister)
        reciver.append(controlRegister[0])
        reciver.append(statusRegister[0] & 0x0001)
        del reciver[0]
        return reciver

    def _GetTa(self, frameData):
        vdd = self._GetVdd(frameData)
        ptat = frameData[224]
        if ptat > 32767:
            ptat -= 65536

        ptatArt = frameData[192]
        if ptatArt > 32767:
            ptatArt -= 65536
        ptatArt = (ptat / (ptat * self.alphaPTAT + ptatArt)) * math.pow(2, 18)

        ta = (ptatArt / (1 + self.KvPTAT * (vdd - 3.3)) - self.vPTAT25)
        ta = ta / self.KtPTAT + 25

        return ta

    def _GetVdd(self, frameData):
        vdd = frameData[234]
        if vdd > 32767:
            vdd -= 65536
        resolutionRAM = (frameData[240] & 0x0C00) >> 10
        resolutionCorrection = math.pow(2, self.resolutionEE) / math.pow(2, resolutionRAM)
        vdd = (resolutionCorrection * vdd - self.vdd25) / self.kVdd + 3.3

        return vdd

    def _CalculateTo(self, frameData, emissivity, tr, result):
        # pylint: disable=too-many-locals, too-many-branches, too-many-statements
        subPage = frameData[241]
        alphaCorrR = [0] * 8

        vdd = self._GetVdd(frameData)
        ta = self._GetTa(frameData)

        ta4 = (ta + 273.15)
        ta4 = ta4 * ta4
        ta4 = ta4 * ta4
        tr4 = (tr + 273.15)
        tr4 = tr4 * tr4
        tr4 = tr4 * tr4
        taTr = tr4 - (tr4-ta4) / emissivity

        ktaScale = math.pow(2, self.ktaScale)
        kvScale = math.pow(2, self.kvScale)
        alphaScale = math.pow(2, self.alphaScale)

        alphaCorrR[1] = 1 / (1 + self.ksTo[1] * 20)
        alphaCorrR[0] = alphaCorrR[1] / (1 + self.ksTo[0] * 20)
        alphaCorrR[2] = 1
        alphaCorrR[3] = (1 + self.ksTo[2] * self.ct[3])
        alphaCorrR[4] = alphaCorrR[3] * (1 + self.ksTo[3] * (self.ct[4] - self.ct[3]))
        alphaCorrR[5] = alphaCorrR[4] * (1 + self.ksTo[4] * (self.ct[5] - self.ct[4]))
        alphaCorrR[6] = alphaCorrR[5] * (1 + self.ksTo[5] * (self.ct[6] - self.ct[5]))
        alphaCorrR[7] = alphaCorrR[6] * (1 + self.ksTo[6] * (self.ct[7] - self.ct[6]))
        #--------- Gain calculation -----------------------------------
        gain = frameData[202]
        if gain > 32767:
            gain -= 65536
        gain = self.gainEE / gain
        #--------- To calculation -------------------------------------
        irDataCP = frameData[200]
        if irDataCP > 32767 :
            irDataCP -= 65536
        irDataCP *= gain
        irDataCP -= self.cpOffset*(1 + self.cpKta*(ta - 25)) * (1 + self.cpKv*(vdd - 3.3))
        for pixelNumber in range(192):
            irData = frameData[pixelNumber]
            if irData > 32767:
                irData -= 65536
            irData *= gain
            kta = self.kta[pixelNumber]/ktaScale
            kv = self.kv[pixelNumber]/kvScale

            irData -= (self.offset[subPage][pixelNumber] *
                        (1 + kta*(ta - 25)) *
                        (1 + kv*(vdd - 3.3)))
            irData = irData - self.tgc * irDataCP
            irData /= emissivity
            alphaCompensated = SCALEALPHA * alphaScale/self.alpha[pixelNumber]
            alphaCompensated *= (1 + self.KsTa * (ta - 25))
            Sx = (alphaCompensated * alphaCompensated *
                    alphaCompensated * (irData + alphaCompensated * taTr))
            try:
                Sx = math.sqrt(math.sqrt(Sx)) * self.ksTo[2]
            except ValueError:
                result[pixelNumber] = "nan"
                continue
            To = math.sqrt(math.sqrt(irData / (alphaCompensated * (1 - self.ksTo[2] * 273.15) + Sx) + taTr)) - 273.15
            if To < self.ct[1]:
                torange = 0
            elif To < self.ct[2]:
                torange = 1
            elif To < self.ct[3]:
                torange = 2
            elif To < self.ct[4]:
                torange = 3
            elif To < self.ct[5]:
                torange = 4
            elif To < self.ct[6]:
                torange = 5
            elif To < self.ct[7]:
                torange = 6
            else:
                torange = 7

            To = math.sqrt(math.sqrt(irData /
                                        (alphaCompensated * alphaCorrR[torange] *
                                        (1 + self.ksTo[torange] *
                                        (To - self.ct[torange]))) + taTr)) - 273.15

            result[pixelNumber] = To
    # pylint: enable=too-many-locals, too-many-branches, too-many-statements

    def _HammingDecode(self):
        parity = [0] * 5
        D = [0] * 16
        error = 0
        switcher = {
            16: 15,
            24: 14,
            20: 13,
            18: 12,
            17: 11,
            31: 10,
            30: 9,
            29: 8,
            28: 7,
            27: 6,
            26: 5,
            25: 4,
            23: 3,
            22: 2,
            21: 1,
            19: 0,
        }
        for addr in range(16,832):
            parity[0] = -1
            parity[1] = -1
            parity[2] = -1
            parity[3] = -1
            parity[4] = -1
            data = eeData[addr]
            mask = 1
            for i in range(16):
                D[i] = (data & mask) >> i
                mask = mask << 1
            parity[0] = D[0]^D[1]^D[3]^D[4]^D[6]^D[8]^D[10]^D[11]
            parity[1] = D[0]^D[2]^D[3]^D[5]^D[6]^D[9]^D[10]^D[12]
            parity[2] = D[1]^D[2]^D[3]^D[7]^D[8]^D[9]^D[10]^D[13]
            parity[3] = D[4]^D[5]^D[6]^D[7]^D[8]^D[9]^D[10]^D[14]
            parity[4] = D[0]^D[1]^D[2]^D[3]^D[4]^D[5]^D[6]^D[7]^D[8]^D[9]^D[10]^D[11]^D[12]^D[13]^D[14]^D[15]
            if (parity[0]!=0) or (parity[1]!=0) or (parity[2]!=0) or (parity[3]!=0) or (parity[4]!=0) :
                check = (parity[0]<<0) + (parity[1]<<1) + (parity[2]<<2) + (parity[3]<<3) + (parity[4]<<4)
                if (check > 15) and (check < 32):
                    D[switcher[check]] = 1 - D[switcher[check]]
                    if error == 0 :
                        raise RuntimeError("check error")
                    data = 0
                    mask = 1
                    for j in range(16):
                        data = data + D[j]*mask
                        mask = mask << 1
                else:
                    raise RuntimeError("cannot find check number")

            eeData[addr] = data & 0x07FF

    def _ExtractParameters(self):
        self._ExtractVDDParameters()
        self._ExtractPTATParameters()
        self._ExtractGainParameters()
        self._ExtractTgcParameters()
        self._ExtractEmissivityParameters()
        self._ExtractResolutionParameters()
        self._ExtractKsTaParameters()
        self._ExtractKsToParameters()
        self._ExtractAlphaParameters()
        self._ExtractOffsetParameters()
        self._ExtractKtaPixelParameters()
        self._ExtractKvPixelParameters()
        self._ExtractCPParameters()
        self._ExtractDeviatingPixels()
        # debug output
        # print('-'*40)
        # print("kVdd = %d, vdd25 = %d" % (self.kVdd, self.vdd25))
        # print("KvPTAT = %f, KtPTAT = %f, vPTAT25 = %d, alphaPTAT = %f" %
             # (self.KvPTAT, self.KtPTAT, self.vPTAT25, self.alphaPTAT))
        # print("Gain = %d, Tgc = %f, Resolution = %d" % (self.gainEE, self.tgc, self.resolutionEE))
        # print("KsTa = %f, ksTo = %s, ct = %s" % (self.KsTa, self.ksTo, self.ct))
        # print("cpAlpha:", self.cpAlpha, "cpOffset:", self.cpOffset)
        # print("alpha: ", self.alpha)
        # print("alphascale: ", self.alphaScale)
        # print("offset: ", self.offset)
        # print("kta:", self.kta)
        # print("ktaScale:", self.ktaScale)
        # print("kv:", self.kv)
        # print("kvScale:", self.kvScale)
        # print(eeData)
        # print('-'*40)
    def _ExtractVDDParameters(self):
        # extract VDD
        self.kVdd = eeData[39]
        if self.kVdd > 1023:
            self.kVdd = self.kVdd - 2048
        self.kVdd = 32 * self.kVdd

        self.vdd25 = eeData[38]
        if self.vdd25 > 1023:
            self.vdd25 = self.vdd25 - 2048
        self.vdd25 = 32 * self.vdd25

    def _ExtractPTATParameters(self):
        # extract PTAT
        self.KvPTAT = eeData[43]
        if self.KvPTAT > 1023:
            self.KvPTAT = self.KvPTAT - 2048
        self.KvPTAT = self.KvPTAT / 4096

        self.KtPTAT = eeData[42]
        if self.KtPTAT > 1023:
            self.KtPTAT = self.KtPTAT - 2048
        self.KtPTAT = self.KtPTAT / 8

        self.vPTAT25 = 32 * eeData[40] + eeData[41]
        self.alphaPTAT = eeData[44] / 128

    def _ExtractGainParameters(self):
        # extract Gain
        self.gainEE = 32 * eeData[36] + eeData[37]

    def _ExtractTgcParameters(self):
        # extract Tgc
        self.tgc = eeData[51] & 0x01FF
        if self.tgc > 255:
            self.tgc -= 512
        self.tgc /= 64
    def _ExtractEmissivityParameters(self):
        self.emissivityEE = eeData[35]
        if self.emissivityEE > 1023:
           self.emissivityEE -= 2048
        self.emissivityEE /= 512

    def _ExtractResolutionParameters(self):
        # extract resolution
        self.resolutionEE = (eeData[51] & 0x0600) >> 9

    def _ExtractKsTaParameters(self):
        # extract KsTa
        self.KsTa = eeData[34]
        if self.KsTa > 1023:
            self.KsTa = self.KsTa - 2048
        self.KsTa /= 32768

    def _ExtractKsToParameters(self):
        # extract ksTo
        self.ct[0] = -40
        self.ct[1] = -20
        self.ct[2] = 0
        self.ct[3] = 80
        self.ct[4] = 120
        self.ct[5] = eeData[58]
        self.ct[6] = eeData[60]
        self.ct[7] = eeData[62]

        KsToScale = eeData[52]
        KsToScale = 1 << KsToScale

        self.ksTo[0] = eeData[53]
        self.ksTo[1] = eeData[54]
        self.ksTo[2] = eeData[55]
        self.ksTo[3] = eeData[56]
        self.ksTo[4] = eeData[57]
        self.ksTo[5] = eeData[59]
        self.ksTo[6] = eeData[61]
        self.ksTo[7] = eeData[63]
        for i in range(8):
            if self.ksTo[i] > 1023:
                self.ksTo[i] -= 2048
            self.ksTo[i] /= KsToScale

    def _ExtractCPParameters(self):
        # extract CP
        self.cpOffset = 32 * eeData[47] + eeData[48]
        if self.cpOffset > 32767:
            self.cpOffset -= 65536

        self.cpAlpha = eeData[45]
        if self.cpAlpha > 1023 :
            self.cpAlpha = self.cpAlpha - 2048
        self.cpAlpha //= math.pow(2,self.cpAlpha)

        self.cpKta = eeData[49] & 0x001F
        if self.cpKta > 31:
            self.cpKta -= 64
        ktaScale1 = eeData[49] >> 6
        self.cpKta /= math.pow(2,ktaScale1)

        self.cpKv = eeData[50] & 0x001F
        if self.cpKv > 31:
            self.cpKv = self.cpKv - 64
        kvScale = eeData[50] >> 6
        self.cpKv /= math.pow(2,kvScale)

    def _ExtractAlphaParameters(self):
        # extract alpha
        scaleRowAlpha = [0] * 6
        rowMaxAlphaNorm = [0] * 6
        alphaTemp = [0] * 192
        scaleRowAlpha[0] = (eeData[25] >> 5) + 20
        scaleRowAlpha[1] = (eeData[25] & 0x001F) + 20
        scaleRowAlpha[2] = (eeData[26] >> 5) + 20
        scaleRowAlpha[3] = (eeData[26] & 0x001F) + 20
        scaleRowAlpha[4] = (eeData[27] >> 5) + 20
        scaleRowAlpha[5] = (eeData[27] & 0x001F) + 20

        for i in range(6):
            rowMaxAlphaNorm[i] = eeData[28 + i] / math.pow(2,scaleRowAlpha[i])
            rowMaxAlphaNorm[i] /= 2047

        for i in range(6):
            for j in range(32):
                p = 32 * i +j
                alphaTemp[p] = eeData[256 + p] * rowMaxAlphaNorm[i]
                alphaTemp[p] = alphaTemp[p] - self.tgc * self.cpAlpha
                alphaTemp[p] = SCALEALPHA / alphaTemp[p]

        temp = max(alphaTemp)

        self.alphaScale = 0
        while temp < 32768 :
            temp = temp * 2
            self.alphaScale = self.alphaScale + 1

        for i in range(192):
            temp = alphaTemp[i] * math.pow(2,self.alphaScale)
            self.alpha[i] = int(temp + 0.5)


    def _ExtractOffsetParameters(self):
        # extract offset
        scaleOffset = eeData[16] >> 5
        scaleOffset = 1 << scaleOffset
        offsetRef = 32 * eeData[17] + eeData[18]
        if offsetRef > 32767:
            offsetRef -= 65536
        for Index in range(192):
            tempOffset = eeData[64 + Index]
            if tempOffset > 1023:
                tempOffset = eeData[64 + Index] - 2048
            self.offset[0][Index] = tempOffset * scaleOffset + offsetRef
            tempOffset = eeData[640 + Index]
            if tempOffset > 1023:
                tempOffset = eeData[640 + Index] - 2048
            self.offset[1][Index] = tempOffset * scaleOffset + offsetRef

    def _ExtractKtaPixelParameters(self): # pylint: disable=too-many-locals
        # extract KtaPixel
        ktaTemp = [0] * 192
        ktaAvg = eeData[21]
        if ktaAvg > 1023:
            ktaAvg = ktaAvg - 2048
        ktaScale1 = eeData[22] >> 5
        ktaScale2 = eeData[22] & 0x001F
        for i in range(192):
            tempKta = (eeData[448 + i] >> 5)
            if tempKta > 31:
                tempKta = tempKta - 64
            ktaTemp[i] = tempKta * math.pow(2,ktaScale2)
            ktaTemp[i] += ktaAvg
            ktaTemp[i] /= math.pow(2,ktaScale1)
        temp = max(map(abs,ktaTemp))
        ktaScale1 = 0
        while temp < 64 :
            temp = temp * 2
            ktaScale1 += 1
        for i in range(192):
            temp = ktaTemp[i] * math.pow(2,ktaScale1)
            if temp < 0:
                self.kta[i] = int(temp - 0.5)
            else:
                self.kta[i] = int(temp + 0.5)
        self.ktaScale = ktaScale1

    def _ExtractKvPixelParameters(self):
        kvTemp = [0] * 192
        kvAvg = eeData[23]
        if kvAvg > 1023:
            kvAvg -= 2048
        kvScale1 = eeData[24] >> 5
        kvScale2 = eeData[24] & 0x001F
        for i in range(192):
            tempKv = (eeData[448 + i] & 0x001F)
            if tempKv > 15:
                tempKv = tempKv - 32
            kvTemp[i] = tempKv * math.pow(2,kvScale2)
            kvTemp[i] += kvAvg
            kvTemp[i] /= math.pow(2,kvScale1)
        temp = abs(kvTemp[0])
        for kv in kvTemp:
            temp = max(temp, abs(kv))
        kvScale1 = 0
        while temp < 64 :
            temp = temp * 2
            kvScale1 += 1
        for i in range(192):
            temp = kvTemp[i] * math.pow(2,kvScale1)
            if temp < 0:
                self.kv[i] = int(temp - 0.5)
            else:
                self.kv[i] = int(temp + 0.5)
        self.kvScale = kvScale1

    def _ExtractDeviatingPixels(self):
        self.brokenPixels = [0xFFFF] * 5

        pixCnt = 0
        brokenPixCnt = 0

        while (pixCnt < 192) and (brokenPixCnt < 3):
            if (eeData[pixCnt+64] == 0) and (eeData[pixCnt+256] == 0) and (eeData[pixCnt+448] == 0) and (eeData[pixCnt+640] == 0):
                self.brokenPixels[brokenPixCnt] = pixCnt
                brokenPixCnt += 1
            pixCnt += 1
        if brokenPixCnt > 2:
            raise RuntimeError("More than 3 broken pixels")
        # print("Found %d broken pixels" % (brokenPixCnt))
