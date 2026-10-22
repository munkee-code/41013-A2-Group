import time
from pyfirmata2 import Arduino, ArduinoMega, Board
import swift

#USB port number - must be configured
PORT = 'COM3'

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

#State store
LastState = None
LastBState = None

try:
    while True:
        #Check States
        StopState = eread.value
        
        #Wait to debounce button
        time.sleep(0.14)
        
        # Check state again
        NewState = eread.value

        #If both checks match and are different to current state, change the state
        if NewState == StopState and NewState != LastState:
            LastState = NewState
            
            if NewState == 0:
                print("EMERGENCY STOP PRESSED")
                #env.hold()
            elif NewState == 1:
                print("E-Stop Released")

        ButtonState = bcheck.value
        #Wait to debounce button
        time.sleep(0.14)
        
        # Check state again
        NewBState = bcheck.value

        #If both checks match and are different to current state, change the state
        if NewBState == ButtonState and NewBState != LastBState:
            LastBState = NewBState

        if NewState ==1 and LastBState == 0:
            print(bcheck.value)
        
        #Wait further
        time.sleep(0.18)
        
except KeyboardInterrupt:
    print("\nExiting and shutting down power loop...")
    epower.write(0)
    Mega.exit()