#!/usr/bin/python
import spidev
import RPi.GPIO as GPIO
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import json
import time
import datetime

matplotlib.use('Agg')

BigJS0N = {}
BigJS0N["firmware_md5"]="49ed003fae3a312590fe99a2fa90392c"
BigJS0N["firmware_version"]="p"
BigJS0N["data"]=[]
BigJS0N["registers"]={}
BigJS0N["parameters"]={}
BigJS0N["time"] = unicode(datetime.datetime.now())
BigJS0N["experiment"] = {}
BigJS0N["timings"] = {}
BigJS0N["experiment"]["id"] = str(datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
    
def CreateDACCurve(Deb,Fin,CurveType):
    n = 200/5
    DACValues = []
    for k in range(n+1):
        if CurveType:
            val = int(Deb+1.0*k*(Fin-Deb)/n)
        else:
            val = int((Fin-Deb)*k**3/n**3+Deb)
        DACValues.append(val) 
    DACValues[-1] = 0
    DACValues[-2] = 0
    return DACValues,len(DACValues)
    
    
def SetDACCurve(spi,DACValues):
    if len(DACValues) < 43: # to correct
        for i in range(len(DACValues)):
            if (DACValues[i] >= 0) and (DACValues[i] < 1020):
                WriteFPGA(spi,16+i,DACValues[i]/4)
            else:
                 WriteFPGA(spi,16+i,0)
            #print 16+i,len(DACValues)
    return 0

def WriteFPGA(spi,adress,value):
    spi.xfer([0xAA] )
    spi.xfer([adress] )
    spi.xfer([value] )
    BigJS0N["registers"][int(adress)]=value
    
def StartUp():
    GPIO.setmode(GPIO.BCM)
    PRESET = 25
    IO4 = 26
    GPIO.setup(PRESET,GPIO.OUT)
    GPIO.setup(IO4,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    print "Reset 25 - Low 1s"
    GPIO.output(PRESET,GPIO.LOW)
    time.sleep(3)
    print "Reset 25 - High 0.2s"
    GPIO.output(PRESET,GPIO.HIGH)
    time.sleep(0.2)

    spi = spidev.SpiDev()
    spi.open(0,1) # CS2 - FPGA, on CE1 = IO4
    spi.mode = 0b01
    print "spi.cshigh is " + str(spi.cshigh)
    print "spi mode is " + str(spi.mode)
    spi.max_speed_hz = 2000000
    print "spi maxspeed is "+str(spi.max_speed_hz)+"hz"
    return spi

def TestSPI(spi,ncycles):
    i = 0
    while i < ncycles:
        WriteFPGA(spi,0xEB,0x01) # 0: single mode 1 continious mode
        time.sleep(0.5)
        WriteFPGA(spi,0xEB,0x00) # 0: single mode 1 continious mode
        time.sleep(0.5)  
        i = i+1

def LoopSPI(spi):
    while 1:
        WriteFPGA(spi,0xEB,0x01) # 0: single mode 1 continious mode
        WriteFPGA(spi,0xEB,0x00) # 0: single mode 1 continious mode

def LoopAcq(spi):
    while 1:
        WriteFPGA(spi,0xEB,0x00) # Doing 1 shot 
        WriteFPGA(spi,0xEF,0x01) # Cleaning memory pointer
        WriteFPGA(spi,0xEA,0x01) # Software Trig : As to be clear by software
        time.sleep(0.001) # sleep 1ms
    
def ClearMem(spi):
    WriteFPGA(spi,0xEF,0x01) # To access memory

def ConfigSPI(spi):
    # Setup FPGA values by default
    setPon(200,spi)              # Set PulseOn
    setPulsesDelay(100,spi)      # Set Lengh between Pon and Poff: 100ns
    setPoff(2000,spi)            # Setting Poff 2us
    #setDACConstant(20,spi)   # gain at 20mV (2%)
    WriteFPGA(spi,0xEC,0x33)
    setDeltaAcq(7000,spi)    # 7us
    #WriteFPGA(spi,0xEA,0x00) # Software Trig : As to be clear by software
    WriteFPGA(spi,0xEB,0x00) # 0: single mode 1 continious mode
    WriteFPGA(spi,0xED,0x03) # Frequency of ADC acquisition / sEEADC_freq (3 = 16Msps, 1 = 32, 0 = 64, 2 = 21Msps)
    WriteFPGA(spi,0xEE,0xA0) # How many cycles in countinious mode
    print "Config FPGA done!"

def setDACConstant(mV,spi):
    if mV > 1000:
        mV = 1000
    elif mV < 0:
        mV = 0   
    hmV = mV/4
    print "Gain:", mV," mV -- ",hex(hmV)
    WriteFPGA(spi,0xEC,hmV) # Voltage gain control: 0V to 1V
    
def setPon(POn,spi):
    if POn > 2500:
        POn = 2500
    elif POn < 0:
        POn = 0
    HPon = POn* 128 / 1000
    BigJS0N["parameters"]["Pon"] = int(POn)
    print "Pulse width:", POn," ns -- ",hex(HPon)
    WriteFPGA(spi,0xE0,HPon) # set sEEPon
    
def setPulsesDelay(DeltaPP,spi):
# Set Lengh between Pon and Poff
    if DeltaPP > 2500:
        DeltaPP = 2500
    elif DeltaPP < 0:
        DeltaPP = 0
    HPP =DeltaPP * 128 / 1000
    #print  hex(HPP)
    BigJS0N["parameters"]["PulsesDelay"] = int(DeltaPP)
    print "Pulses delay:", DeltaPP," ns -- ",hex(HPP)
    WriteFPGA(spi,0xD0,HPP) # set sEEPon

def setPoff(sEEPoff,spi):
    # Sets the damping length.
    POff = sEEPoff * 128 / 1000
    #print sEEPoff,POff
    POffMSB, POffLSB = 0x00FF&POff/256,0x00FF&POff 
    print "Poff:", sEEPoff," ns -- ",hex(POffMSB),hex(POffLSB)
    BigJS0N["parameters"]["Poff"] = int(sEEPoff)
    WriteFPGA(spi,0xE1,POffMSB) # set sEEPon MSB
    WriteFPGA(spi,0xE2,POffLSB) # set sEEPon LSB

    # Setting Poff to Acq delay sEEDelayACQ
def setDeltaAcq(DeltaAcq,spi):
    if DeltaAcq > 255*255:
        DeltaAcq = 254*254
    elif DeltaAcq < 0:
        DeltaAcq = 0
    hDA = DeltaAcq * 128 / 1000
    hDAMSB, hDALSB = hDA/255 , 0x00FF&hDA 
    print "Delay between:",DeltaAcq,"ns -- ", hex(hDAMSB),hex(hDALSB)
    BigJS0N["parameters"]["DeltaAcq"] = int(DeltaAcq)
    WriteFPGA(spi,0xE3,hDAMSB) # set sEEPon MSB
    WriteFPGA(spi,0xE4,hDALSB) # set sEEPon LSB
    
def SetLengthAcq(LAcqI,spi):
    LAcq = LAcqI * 128 / 1000
    #print LAcq,hex(LAcq),hex(LAcqI)
    BigJS0N["parameters"]["LengthAcq"] = int(LAcqI)
    LAcqMSB, LAcqLSB = 0x00FF&LAcq/256 , 0x00FF&LAcq
    print "Acquisition length: ", LAcq, " ns -- ",hex(LAcqMSB),hex(LAcqLSB)
    WriteFPGA(spi,0xE5,LAcqMSB) # set sEEPon MSB
    WriteFPGA(spi,0xE6,LAcqLSB) # set sEEPon LSB

def setPeriodAcq(lEPeriod,spi):
    lEPNs = lEPeriod*128/1000 #ns
    EPNsMSB, EPNs, EPNsLSB = 0x00FF&lEPNs/(256*256),0x00FF&lEPNs/256,0x0000FF&lEPNs 
    print "Period between two acquisitions:", lEPeriod,"us --", hex(EPNsMSB),hex(EPNs),hex(EPNsLSB) 
    BigJS0N["parameters"]["PeriodAcq"] = int(lEPeriod)
    WriteFPGA(spi,0xE7,EPNsMSB) # Period of one cycle MSB
    WriteFPGA(spi,0xE8,EPNs) # Period of one cycle 15 to 8
    WriteFPGA(spi,0xE9,EPNsLSB) # Period of one cycle LSB

def setPulseTrain(Pon,Pdelay,Poff,DelayAcq,Acq,spi):
    setPon(Pon,spi)
    setPulsesDelay(Pdelay+Pon,spi)
    setPoff(Poff+Pdelay+Pon,spi)
    setDeltaAcq(DelayAcq+Poff+Pdelay+Pon,spi)
    SetLengthAcq(Acq+DelayAcq+Poff+Pdelay+Pon,spi)
    #SetLengthAcq(Acq,spi)
    
# Now we start the script

spi = StartUp()
print("------")
ConfigSPI(spi)
print("------")

N = 0

# Sampling at 64Msps
f = 0x00
# Pulse train
t1 = 200
t2 = 100
t3 = 2000
t4 = 300-t1-t2+10
t5 = 200000
setPulseTrain(t1,t2,t3,t4,t5,spi)
# Now is the gain
Curve = CreateDACCurve(100,400,True)[0]
SetDACCurve(spi,Curve)
LAcq = t5/1000 #ns to us
WriteFPGA(spi,0xED,f) # Frequency of ADC acquisition / sEEADC_freq (3 = 16Msps, 1 = 32, 0 = 64, 2 = 21Msps)
WriteFPGA(spi,0xEB,0x00) # Doing one line
WriteFPGA(spi,0xEF,0x01) # Cleaning memory pointer
WriteFPGA(spi,0xEA,0x01) # Software Trigger for the acquisition

Fech = int(64/((1+f)))
Nacq = LAcq * Fech
print Fech, "-> "+str(Nacq) + ' samples' 

A = []
# We get 2x the number of samples as each sample is split over 2 bytes
for i in range(2*Nacq+1):
    A.append ( spi.xfer([0x00] )[0] )
    a = np.asarray(A).astype(int)
    BigJS0N["data"] = A
    with open(BigJS0N["experiment"]["id"]+".json", 'w') as outfile:
        json.dump(BigJS0N, outfile)

print "Done"
