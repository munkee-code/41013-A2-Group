import time
from pyfirmata2 import Arduino, ArduinoMega, Board

#USB port number - must be configured
PORT = 'COM7'

#Define Arduino
Mega = ArduinoMega(PORT)
Mega.samplingOn()
time.sleep(0.3)

#Estop powered pin
epower = Mega.get_pin('d:22:o')
epower.write(1) 
time.sleep(0.2)

#Estop read pin
eread = Mega.get_pin('d:7:i') 
eread.enable_reporting()
time.sleep(0.2)

#Button check pin
bcheck = Mega.get_pin('d:32:i')
bcheck.enable_reporting()
time.sleep(0.2)

#LightCurtainpins
LightIN = Mega.get_pin('a:15:i')
LightOUT = Mega.get_pin('d:26:o')
LightIN.enable_reporting()
time.sleep(0.2)

#State store
LastState = None

def CheckCURTAIN():
    CurtainState = LightIN.value
    time.sleep(0.01)
    CurtainState2 = LightIN.value
    time.sleep(0.01)
    diff = CurtainState2-CurtainState
    if diff < 0.001:
        print(CurtainState, "Nothing")
        
    elif CurtainState >= 0.001:
        print("DETECTED")

try:
    while True:
        #Check States
        StopState = eread.value
        ButtonState = bcheck.value
        CurtainState = LightIN.value

        CheckCURTAIN()

        #if CurtainState <= 0.064:
            #print(CurtainState, "Nothing")
        #elif CurtainState >= 0.064:
            #print("DETECTED")

        #if ButtonState == 0:
            #print("not set")
        #elif ButtonState == 1:
             #print("reset")
        
        #Wait to debounce button
        time.sleep(0.14)
        
        # Check state again
        NewState = eread.value

        #If both checks match and are different to current state, change the state
        if NewState == StopState and NewState != LastState:
            LastState = NewState
            
            if NewState == 0:
                print("🚨 EMERGENCY STOP PRESSED! (Circuit Open)")
            elif NewState == 1:
                print("✅ E-Stop Released (System Safe)")
        #if ButtonState == 0:
            #print("not set")
        #elif ButtonState == 1:
            #print("reset")
        
        #Wait further
        time.sleep(0.18)
        
except KeyboardInterrupt:
    print("\nExiting and shutting down power loop...")
    epower.write(0)
    Mega.exit()