import time
from pyfirmata2 import Arduino, ArduinoMega, Board

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

state_system = 0
running = True
    
def checkRESET():
    print("System waiting for restart")
    while True:
        #Print reset message
        print("System waiting for restart")
        #Check States
        ButtonState = bcheck.value
            
        if ButtonState == 0:
            print("not set")
        elif ButtonState == 1:
            print("System has been resumed")
            state_system = 0
        #Wait further
        time.sleep(0.25)


def checkESTOP():
    #State store
    LastEState = False
    Pressed=0
    def loop():
        while True:
            LastEState = False
            #Check States
            StopState = eread.value

            #Wait to debounce buttons
            time.sleep(0.2)
                    
            #Check state again
            NewState = eread.value
            
            #If both checks match and are different to current state, change the state
            if NewState == StopState and NewState != LastEState:
                LastEState = NewState
                        
            if NewState == 0:
                print("HARDWARE E STOP ACTIVATED")
                Pressed = 1
                loop()
            elif NewState == 1 and Pressed == 1:
                print("Hardware E stop reset")
                #checkRESET()

            #Wait further
            time.sleep(0.25)
    loop()
        

Pressed = 0
LastEState = True
checkESTOP()

#try:
        
#except KeyboardInterrupt:
    #print("\nExiting and shutting down power loop...")
    #epower.write(0)
    #Mega.exit()