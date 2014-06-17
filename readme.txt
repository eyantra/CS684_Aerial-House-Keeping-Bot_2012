Group 13 (Aerial House Keeping Bot)
Raj Agrawal 11305R004
Prashanth Kazipeta 09007033
Pranay Kumar Myana 09007034
Hricha Kabir 113050079

*********************************************************************************************************************************
Setup the Firebird V Bot with chords and the black box over the triangular arena where the black box is to be moved.
Tie the chords with the wheels of the bot.
Note that chords should be tied such that, the forward rotation of the wheel releases the chords and backward rotation winds the
chords back to wheel.
The initial position of black box should be kept to coordinates (0, 0) in the arena.
Setup the Zigbee communication interface between the bot and a computer.

**********************************************************************************************************************************
Moving the Black Box along the troughs:
•	Give the input to the bot using the Zigbee module configured in the computer.
•	The input should be in the format (u<x, y>) followed by a ‘.’ where (x, y) is the trough no.
•	As soon as u press ‘.’ the black box will start moving towards the destination.
•	Once the BB has moved to destination, bot will ask the actual position of the BB, from the feedback system.
•	Since feedback system has not been integrated, this step can be done manually. Bot will send a request the position of 	the 
	black box via a Zigbee module, for this you will see ‘r’ on the Zigbee interface.
•	At this time give the actual coordinates of the BB in the arena.
•	If there is any error bot will move again to the actual destination.
•	Once the black box has moved to the destination it will halt and the bot will wait for the next input from user.
