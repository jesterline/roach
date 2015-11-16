#!/usr/bin/env python
"""
authors: apullin

This script will run an experiment with one or several Velociroach robots.

The main function will send all the setup parameters to the robots, execute defined manoeuvres, and record telemetry.

"""
from lib import command
import time,sys,os,traceback
import serial

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/")) 
import shared_multi as shared
import numpy as np

from velociroach import *

####### Wait at exit? #######
EXIT_WAIT   = False

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
    R1 = Velociroach('\x21\x04', xb)
    R1.SAVE_DATA = True
                            
    #R1.RESET = False       #current roach code does not support software reset
    
    shared.ROBOTS.append(R1) #This is necessary so callbackfunc can reference robots
    shared.xb = xb           #This is necessary so callbackfunc can halt before exit

    # Send resets
    for r in shared.ROBOTS:
        if r.RESET:
            r.reset()
            time.sleep(0.35)
    # Repeat this for other robots
    # TODO: move reset / telem flags inside robot class? (pullin)
    
    # Send robot a WHO_AM_I command, verify communications
    for r in shared.ROBOTS:
        r.query(retries = 3)
    
    #Verify all robots can be queried
    verifyAllQueried()  # exits on failure
    
    # Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ----------RIGHT----------
    motorgains = [1800,0,100,0,0, 1000,0,50,0,0] #removed for multigain
    #motorgains = [0,0,0,0,0 , 0,0,0,0,0]

    #Alternating tripod gait
    simpleAltTripod = GaitConfig(motorgains, rightFreq=15, leftFreq=22) # Parameters can be passed into object upon construction, as done here. removed for multigain

    simpleAltTripod.phase = PHASE_180_DEG                             # Or set individually, as here
    simpleAltTripod.deltasLeft = [0.25, 0.25, 0.25]
    simpleAltTripod.deltasRight = [0.25, 0.25, 0.25]
    #simpleAltTripod.deltasTime  = [0.25, 0.25, 0.25] # Not current supported by firmware; time deltas are always exactly [0.25, 0.25, 0.25, 0.25]
    
    #Hold constant position for 
    constSetPoint = GaitConfig(motorgains, rightFreq=5, leftFreq=.05) #freq=.05 gives 10s at set point # Parameters can be passed into object upon construction, as done here.
    constSetPoint.phase = PHASE_180_DEG                             # Or set individually, as here 
    constSetPoint.deltasLeft = [0.5, 0, 0]
    constSetPoint.deltasRight = [0.25, 0.25, 0.25]

    # Configure intra-stride control
    R1.setGait(simpleAltTripod) #removed for multigain

    # example , 0.1s lead in + 2s run + 0.1s lead out
    EXPERIMENT_RUN_TIME_MS     = 26000 #ms
    EXPERIMENT_LEADIN_TIME_MS  = 100  #ms
    EXPERIMENT_LEADOUT_TIME_MS = 100  #ms
    
    # Some preparation is needed to cleanly save telemetry data
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            #This needs to be done to prepare the .telemtryData variables in each robot object
            r.setupTelemetryDataTime(EXPERIMENT_LEADIN_TIME_MS + EXPERIMENT_RUN_TIME_MS + EXPERIMENT_LEADOUT_TIME_MS)
            r.eraseFlashMem()
        
    # Pause and wait to start run, including lead-in time
    print ""
    print "  ***************************"
    print "  *******    READY    *******"
    print "  ***************************"
    raw_input("  Press ENTER to start run ...")
    print ""

    # Initiate telemetry recording; the robot will begin recording immediately when cmd is received.
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            r.startTelemetrySave()
    
    # Sleep for a lead-in time before any motion commands
    time.sleep(EXPERIMENT_LEADIN_TIME_MS / 1000.0)
    
    # Create arrays for freqs to be used
    # Various
    #frArray = [1,3,5,7,9]
    #frArray = [3,3.5,4,4.5,5]
    #numFreqs = len(frArray)
    #125 to 150
    #phaseArray=[0x5556, 0x5778, 0x6000, 0x6222, 0x6444, 0x6667]
    #155 to 180
    #phaseArray=[0x6889, 0x7111, 0x7333, 0x7556, 0x7778, 0x8000]
    #180 to 205
    #phaseArray=[0x8000, 0x8222, 0x8444, 0x8667, 0x8889, 0x9111]
    #210 to 235
    phaseArray=[0x9333, 0x9556, 0x9778, 0x9FFF, 0xA38E, 0xA71C]
    numPhases = len(phaseArray)

    # Wave of freqs
    #numFreqs = 21
    #frArray = np.concatenate((np.linspace(2600,3600,11),np.linspace(3500,2600,10)),axis=1)

    ######## Motion is initiated here! ########
    #start run with initial freq
    simpleAltTripod = GaitConfig(motorgains, rightFreq=1, leftFreq=1) 
    simpleAltTripod.phase = phaseArray[0]
    simpleAltTripod.deltasLeft = [0.25, 0.25, 0.25]
    simpleAltTripod.deltasRight = [0.25, 0.25, 0.25]
    R1.setGait(simpleAltTripod)
    R1.startTimedRun( EXPERIMENT_RUN_TIME_MS) #Faked for now, since pullin doesn't have a working VR+AMS to test with
    time.sleep(EXPERIMENT_RUN_TIME_MS / 1000.0 / numPhases)
    #time.sleep(EXPERIMENT_RUN_TIME_MS / 1000.0 / 4) #for const set point

    for i in range(1,numPhases):
        simpleAltTripod = GaitConfig(motorgains, rightFreq=1, leftFreq=1) 
        simpleAltTripod.phase = phaseArray[i] #R1.currentGait.phase
        simpleAltTripod.deltasLeft = [0.25, 0.25, 0.25]
        simpleAltTripod.deltasRight = [0.25, 0.25, 0.25]
        R1.setVelProfile(simpleAltTripod)
        R1.setPhase(simpleAltTripod.phase)
        
        #R1.startTimedRun( EXPERIMENT_RUN_TIME_MS / numGains) #Faked for now, since pullin doesn't have a working VR+AMS to test with
        time.sleep(EXPERIMENT_RUN_TIME_MS / 1000.0 / numPhases - 0.3)  #argument to time.sleep is in SECONDS #-0.4 to account for lag
        #time.sleep(EXPERIMENT_RUN_TIME_MS / 1000.0 / numGains / 2 - 0.4) #for const set point
        ######## End of motion commands   ########
    
    #time.sleep(EXPERIMENT_RUN_TIME_MS / 1000.0 / 4) #for const set point

    # Sleep for a lead-out time after any motion
    time.sleep(EXPERIMENT_LEADOUT_TIME_MS / 1000.0) 
    
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            raw_input("Press Enter to start telemetry read-back ...")
            r.downloadTelemetry()
    
    if EXIT_WAIT:  #Pause for a Ctrl + C , if desired
        while True:
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                break

    print "Done"
    xb_safe_exit(xb)
    
#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
#TODO: provide a more informative exit here; stack trace, exception type, etc
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\nRecieved Ctrl+C, exiting."
        shared.xb.halt()
        shared.ser.close()
    except Exception as args:
        print "\nGeneral exception:",args
        print "\n    ******    TRACEBACK    ******    "
        traceback.print_exc()
        print "    *****************************    \n"
        print "Attempting to exit cleanly..."
        shared.xb.halt()
        shared.ser.close()
        sys.exit()
    except serial.serialutil.SerialException:
        shared.xb.halt()
        shared.ser.close()
