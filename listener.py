#!/usr/bin/env python

import rospy

import dynamixel_driver
from dynamixel_driver import dynamixel_io
from dynamixel_driver import dynamixel_const
from dynamixel_driver import dynamixel_serial_proxy

from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from beginner_tutorials.msg import angle_list


#change these values to configure the motors
motorID = [1,2,3,4,5]
motorPort = "/dev/ttyUSB0"
motorBaudRate = 1000000
motorID_limit_min = [10,60,150,150,150]
motorID_limit_max = [290,190,240,240,240]
motorID_angleOwner = [0,1,2,3,4]

#global variable to store previous values and check
prevAngles = []

#special motor/angle index to parse into bool operation
#Note: SET WITH CAUTION, DUE TO LACK OF LIMITER, ANGLE WILL FORCE INTO FIXED ANGLE SETTINGS
angleIndex_list = [0,]
angleIndex_cutOffAngles = [[180,],]
angleIndex_fixAngles = [[100,300],]
angleIndex_lastState = [0,]
angleIndex_counter = [0,]
angleIndex1 = 0;
angleIndex1_fixAngle1 = 100
angleIndex1_fixAngle2 = 300
#for suppressing fringe responses
angleIndex1_counter = 0
angleIndex1_lastState = False

# Angle converter will convert a float angle number into a tuple of (a,b,c) which will be used to write into motor
# a is just the motor id
# (b,c), b is a decominal number asssociated with the 2 Least Significant Hexadecimal, while c is a decimal number involved with the 3rd Hexadecimal
def angleConverter(ID, angle, angleMin = 0, angleMax = 300):
	if angle < angleMin or angle > angleMax:
		rospy.loginfo("Angle: " + str(angle) + " --->Angle not within acceptable range! Angle converter will return NULL(None)")
		return None
	else:
		angleFloat = float(angle)
		#angle div is the angle for each bit which is about 0.29 degree
		angleDiv = 300.0/1023.0
		#get the angle in approx number of bits
		bitAngle = int(angleFloat/angleDiv)
		
		if bitAngle > 1023:
			bitAngle = 1023
		elif bitAngle < 0:
			bitAngle = 0
			
		bitAngle_thirdBit = (bitAngle & 3840)    #3840 is 1111 0000 0000
		bitAngle_thirdBit = (bitAngle_thirdBit >> 8)    #get the int number involved with third hexadecimal
		
		bitAngle_firstSecondBit = bitAngle & 255    #255 is 0000 1111 1111, will also get int number involved with first and second hexademimnal
		
		rospy.loginfo("Angle: " + str(angle))
		rospy.loginfo("Angle (Bit): " + str(bitAngle))
		#rospy.loginfo(bitAngle_firstSecondBit)
		#rospy.loginfo(bitAngle_thirdBit)
		return (ID,bitAngle_firstSecondBit, bitAngle_thirdBit)


def initialize_motor(chosenPort, chosenBaudRate = 1000000):
	rospy.loginfo("Initialzing motor at port: " + chosenPort + " and Baud rate: " + str(chosenBaudRate))
	motor = dynamixel_io.DynamixelIO( port=chosenPort, baudrate = chosenBaudRate)
	return motor
		
	
def set_motor_group_pos(data):
	#data should be a tuple of floats (angles), assumed to be in order of index numbers
	motors = initialize_motor(motorPort,motorBaudRate)
	inputDataList = []
	dataLength = 0
	
	isSegmented = True
	anglePart = 5.0
	isIndexed = True
	indexCounterLimit = 5;
	
	prevAngleTemp = []
	
	#check which data set is shorter, loop cycle has to adhere to shorter data set to not overflow
	if len(motorID) >= len(data.angle_list):
		dataLength = len(data.angle_list)
	elif len(motorID) < len(data.angle_list):
		dataLength = len(motorID)
	
	rospy.loginfo("")
	rospy.loginfo("")
	
	#loop through each angle and motor set and check for validity and add into array of data (to be written into motorss at end).
	for i in range(dataLength):

		motors.set_speed(motorID[i], 100)
		angleTarget = data.angle_list[motorID_angleOwner[i]]
		isAngleIndex = False
		isAngleIndex_tag = 0;
		
		#Special angles indexed will be parsed accordingly
		for j in angleIndex_list:
			if i==j:
				isAngleIndex = True
				isAngleIndex_tag = i
				break
			
		if isAngleIndex == True:
			rospy.loginfo("Special indexed angle detected! Processing...")
			global angleIndex_lastState[i]
			global angleIndex_counter[i]
			numOfCutOff = len(angleIndex_cutOffAngles[i])
			
			for j in range(numOfCutOff):
				if angleTarget < angleIndex_cutOffAngles[j]:
					#it is this state
					rospy.loginfo("Angle (" + str(angleTarget) + ") results in " + str(j) + " state")
					angleResult_spec = angleConverter(ID = motorID[i], angle = angleIndex1_fixAngle1)
					
					if angleIndex1_lastState != j:
						if angleIndex_counter[] < indexCounterLimit:
							ang
			
			if angleTarget <= 180:
				rospy.loginfo("Angle (" + str(angleTarget) + ") results in 'false' state")
				angleResult_spec = angleConverter(ID = motorID[i], angle = angleIndex1_fixAngle1)
				
				if angleIndex1_lastState == True:
					if angleIndex1_counter < indexCounterLimit:
						angleIndex1_counter+=1
						prevAngleTemp.append(angleIndex1_fixAngle2)
					else:
						angleIndex1_lastState = False
						angleIndex1_counter = 0;
						inputDataList.append(angleResult_spec)
						rospy.loginfo("Special angle appended: " + str(angleResult_spec))
						prevAngleTemp.append(angleIndex1_fixAngle1)
				else:
					angleIndex1_counter = 0;
					inputDataList.append(angleResult_spec)
					rospy.loginfo("Special angle appended: " + str(angleResult_spec))
					prevAngleTemp.append(angleIndex1_fixAngle1)
				
			elif angleTarget > 180:
				rospy.loginfo("Angle: " + str(angleTarget) + "--> results in 'true' state")
				angleResult_spec = angleConverter(ID = motorID[i], angle = angleIndex1_fixAngle2)
				
				if angleIndex1_lastState == False:
					if angleIndex1_counter < indexCounterLimit:
						angleIndex1_counter+=1
						prevAngleTemp.append(angleIndex1_fixAngle1)
					else:
						angleIndex1_lastState = True
						angleIndex1_counter = 0;
						inputDataList.append(angleResult_spec)
						rospy.loginfo("Special angle appended: " + str(angleResult_spec))
						prevAngleTemp.append(angleIndex1_fixAngle2)
				else:
					angleIndex1_counter = 0;
					inputDataList.append(angleResult_spec)
					rospy.loginfo("Special angle appended: " + str(angleResult_spec))
					prevAngleTemp.append(angleIndex1_fixAngle2)
			
			rospy.loginfo("Special State: " + str(angleIndex1_lastState) + "    Special Count: " + str(angleIndex1_counter))
			continue
		
		angleResult = angleConverter(ID = motorID[i], angle = angleTarget, angleMin = motorID_limit_min[i], angleMax = motorID_limit_max[i])
		
		if prevAngles:
			angleCurr = prevAngles[len(prevAngles)-1][i]
		else:
			angleCurr = None
		
		if angleResult == None:
			rospy.loginfo("Angle with " + str(angleTarget) + " degrees is out of range/not acceptable, ignoring this angle.")
			prevAngleTemp.append(angleCurr)
			continue
		
		if isSegmented == False:
			inputDataList.append(angleResult)
			rospy.loginfo("Angle result appended: " + str(angleResult))
			prevAngleTemp.append(angleTarget)
		else:
			if angleCurr != None:
				angleDiff = angleTarget - angleCurr
				rospy.loginfo("For motor number " + str(motorID[i]) + " ---> Current Angle: " + str(angleCurr) + "  Angle Diff: " + str(angleDiff))
				
				if(angleDiff) > anglePart or (angleDiff) < -anglePart:
					inputDataList.append(angleResult)
					rospy.loginfo("Angle result appended: " + str(angleResult))
					prevAngleTemp.append(angleTarget)
				else:
					rospy.loginfo("Angle not distant enough, rejecting angle.")
					prevAngleTemp.append(angleCurr)
			else:
				inputDataList.append(angleResult)
				rospy.loginfo("Angle result appended: " + str(angleResult))
				prevAngleTemp.append(angleTarget)
	
		
	tuple(inputDataList)
	rospy.loginfo("Inputing angle/motor list")
	rospy.loginfo(inputDataList)
	
	if inputDataList:
		motors.sync_write(dynamixel_const.DXL_GOAL_POSITION_L, inputDataList)
	
	if len(prevAngles) >= 5:
		prevAngles.pop(0)
		
	prevAngles.append(prevAngleTemp)
	rospy.loginfo("Previous Angles:")
	rospy.loginfo(prevAngles)
	
	
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
	
	rospy.init_node('listener', anonymous=True)
	
	rospy.Subscriber('chatter', angle_list, set_motor_group_pos)
	
    # spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
    listener()
