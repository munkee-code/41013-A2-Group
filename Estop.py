import time
from pyfirmata2 import Arduino, ArduinoMega, Board
import swift


class Estop:
    def __init__(self,Port,env,gui):
        self.env = env

        Mega = ArduinoMega(Port)
        Mega.samplingOn()
        time.sleep(0.02)

        epower = Mega.get_pin('d:22:o')
        epower.write(1) 
        time.sleep(0.02)

        #Estop read pin
        eread = Mega.get_pin('d:7:i') 
        eread.enable_reporting()
        time.sleep(0.02)

        #State store
        LastState = None

        try:
            while True:
                #Check States
                StopState = eread.value
        
                #Wait to debounce button
                time.sleep(0.05)
        
                # Check state again
                NewState = eread.value

                #If both checks match and are different to current state, change the state
                if NewState == StopState and NewState != LastState:
                    LastState = NewState
            
                    if NewState == 0:
                        print("EMERGENCY STOP PRESSED")
                        #env.hold()
                        gui.estopActive = True
                    elif NewState == 1:
                        print("E-Stop Released")

                #Wait to debounce button
                time.sleep(0.13)
        
                #Wait further
                time.sleep(0.13)
        
        except KeyboardInterrupt:
            print("\nExiting and shutting down power loop...")
            epower.write(0)
            Mega.exit()