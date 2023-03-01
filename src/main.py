from ev3dev2.motor import LargeMotor, OUTPUT_C, OUTPUT_B, SpeedPercent 
from ev3dev2.sensor import INPUT_1 
from ev3dev2.sensor.lego import InfraredSensor 

# Connect infrared and motor sensors to the EV3 brick 
ir = InfraredSensor(INPUT_1) 
motorLeft = LargeMotor(OUTPUT_B) 
motorRight = LargeMotor(OUTPUT_C) 

 # Start moving forward at a speed of 25%  
motorLeft.run_forever(speed_sp=25)  
motorRight.run_forever(speed_sp=25)  

 # Check if the wall is detected by the infrared sensor  
while not ir.proximity < 30:  

    # If the wall is detected, turn right and continue moving forward  
    motorLeft.run_forever(speed_sp=-50)  
    motorRight.run_forever(speed_sp=50)
