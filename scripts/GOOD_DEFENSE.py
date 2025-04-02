import random
import numpy as np

# each player covers a region of 120 pixels. It takes approximately 550 steps for each player to reach the end of its zone (this number must be callibrated again)
# the computer vision must detect the position of the ball and the player region it exists in
# it is 550 pulses until the red player closest to the brain hits the wall starting from its position closest to the human player
# remember to account for the bounds of the board!

# check out GRID OVERLAY openCV function to automatically calibrate the distance between walls
# openCV grid calibration (account for image distortion in coordinate systems)


def defense(position = 0):

    #pulse_per_pixel = 550 / 120 #The stepper motor is not moving across the ENTIRE range of the field. It is moving a fractional distance across the field based on the distance of the player nearest the wall. Tha fractional distance is approximately 120 pixels across the image.
    #Pulse_per_pixel may need to be made a custom value per the field range of each player
    #position = random.random()*360
    #zone_distance = position
    #zone = 0

    #rod_position = int(random.random()*550)
    #rod_position = 0
	pulses = 0
	player1 = np.arange(20, 129) #player1's lane ... really starts at ~20 because of the table ... player 1's range is from ~20 to ~ 110
	player2 = np.arange(130, 229) #player2's lane
	player3 = np.arange(230, 350) #player3's lane ... really ends ~350 because of table
		
	if int(position) in player1:
		pulse_per_pixel = 450 / 110
		zone_distance = position - player1[0]
        	#zone_ratio = zone_distance / ( player1[-1] - player1[0] ) * 550
        	#zone = 1
		pulses = int(pulse_per_pixel*zone_distance)
	if int(position) in player2:
		pulse_per_pixel = 550 / 100
		zone_distance = position - player2[0]
        	#zone_ratio = zone_distance / ( player2[-1] - player2[0] ) * 550
        	#zone = 2
		pulses = int(pulse_per_pixel*zone_distance)
	if int(position) in player3:
		pulse_per_pixel = 450 / 120
		zone_distance = position - player3[0]
        	#zone_ratio = zone_distance / ( player3[-1] - player3[0] ) * 550
        	#zone = 3
		pulses = int(pulse_per_pixel*zone_distance) + 100
	

	return pulses




#print("Ball is at: ", position, "pixels.")
#print("Rod is At:", rod_position, "pulses. ")
#print("Ball Position in Zone: ", zone_distance, "into Zone ", zone)
#print("Number of Pulses from 0 to Ball: ", pulses ) #this value is a number of pulses from the rightmost wall required to mirror the position of the ball
#print("Required Number of Pulses from current location to Ball:", pulses - rod_position)

# we would need to compare this measurement ^^^ to the live position of the rod to make sure we aren't feeding the motor too many steps.




# given that when the rod is moved entirely across the field each player 'runs' across their respective zone, the zone each player covers is approximately 550 steps across
# however, not every zone is equal in size and this must be accounted for in the final calculations
# for now, the steps can be calculated based on the ration of the number of pixels to the size of the zone

# it might be possible to set each players zone to a unique range of pixels and to find the number of steps it takes for that specific player to cross its zone
# then you could relate each players position to a fractional distance across their zone and therby a number of motor steps

# example
# player two's zone might not start from a flush position against the wall (due to the uneven positions of the players)
# player twos zone could exist within a range of pixels/steps
# player two could have calculations that only get called when the ball is detected within its pixel range
# when player two is set to GAURD it could automatically fluctauate between a number of motor steps

# each players GUARDING RANGE could be associated with a range of MOTOR STEPS that could relate to pixel values.

# THIS PROGRAM "AS IS" IS A ROUGH APPROXIMATION OF THE FINAL CALCULATIONS. THE GUARDING RANGE OF EACH PLAYER AND THE ASSOCIATED NUMBER OF MOTOR STEPS MUST BE FINE TUNED
