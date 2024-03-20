# Code for one of the projects

Compose:
	 1 Vibration sensor, 3 DC motors, 2 ultrasonic sensor, 1 servo motor 

Loop: 
	
	Program continuously read vibration sensor pin 10, and if its value is low. Then static bool value is set to true.
	Every one second program check that boolean value and if true, it adds by one to static couter and sets that bool value back to false. 
		If value of counter overflows the desired number of counts, then "cleaning process" is started. 
			Starting one DC motor it waits in while loop with ultrasonic sensor readings per loop. If the distance measured from ultrasonic sensor is greater then
			predefined distance, the motor starts and same procedure is goin with second motor but it with difference that if the measured distance from start is lesser then predefined,
			 cleaning procces is reseted and all running motors are shutted down.
			Starting last DC motor the proccedure is same with periodic checking from ultrasonic sensors, but this motor will spin for predefined certain amount of time. If time is up, then motors are shut down and counter is reseted to 0.
		If something goes in the way of USS, the motors are shut down, but counter is not reseted, therefore the cleaning procces will start again. 
		If interruption occurs when third motor is spinning and time is < 3/4 desired time, then it would restart again, but if time > 3/4 desired time, the cleaning would be considered 			done. 

		If cleaning the servo is at position 45 degree if not 0 degree. 
			
	
