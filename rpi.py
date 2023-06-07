import serial
import time

#for light
RED_CIRCLE = 'A'
BLUE_CIRCLE = "B"
GREEN_CIRCLE = "C"
BLUE_CROSS_1 = "D"
BLUE_CROSS_2 = "E"
RED_TRI_1 = "F"
RED_TRI_2 = "G"
RED_TRI_3 = "H"
LIGHT_OFF= "I"

#for control
FIRE_ON = "1"
WATER_ON = "2"
CAR_FRONT = "3"
CAR_BACK = "4"
CAR_LEFT = "5"
CAR_RIHGT = "6"
CAR_OFF = "7"

#return state
RED_CIRCLE_D = b'a\r\n'
BLUE_CIRCLE_D = b'b\r\n'
GREEN_CIRCLE_D = b'c\r\n'
BLUE_CROSS_1_D = b'd\r\n'
BLUE_CROSS_2_D = b'e\r\n'
RED_TRI_1_D = b'f\r\n'
RED_TRI_2_D = b'g\r\n'
RED_TRI_3_D = b'h\r\n'
LIGHT_OFF_D = b'i\r\n'
FIRE_ON_D = b'j\r\n'
WATER_ON_D = b'k\r\n'
CAR_OFF_D = b'l\r\n'

Mega = serial.Serial(port='/dev/ttyACM0',baudrate=9600)
IMU = serial.Serial(port='/dev/ttyACM1',baudrate=9600)
BE = serial.Serial(port='/dev/ttyACM2',baudrate=115200)
state = "green_light_1"
return_data = ""
right = False
left = False
while True:
	if BE.inWaiting()>0:
		BE_data = BE.read()
		if state == 'wait_FIRE_on':
			print("FIRE_on")
			write_data = FIRE_ON
			Mega.write(bytes(write_data,'utf-8'))
			state = "Fire_ON"
		if state == 'wait_WATER_on':
			print("WATER_on")
			write_data = WATER_ON
			Mega.write(bytes(write_data,'utf-8'))
			state = 'WATER_on'
		if BE_data == b'success\r\n' and state == 'car':
			print("finish")
			state = 'finish'
			write_data = LIGHT_OFF
			Mega.write(bytes(write_data,'utf-8'))
			
	if Mega.inWaiting() >0:
		return_data = Mega.readline()
		print("return data :",return_data)

		#For Fire
		if state == "red_light_wait_1" and return_data == RED_CIRCLE_D:
			state = "red_light_2"
		if state == "red_light_wait_2" and return_data == RED_TRI_1_D:
			state = "red_light_3"
		if state == "red_light_wait_3" and return_data == RED_TRI_2_D:
			state = "red_light_4"
		if state == "red_light_wait_4" and return_data == RED_TRI_3_D:
			state = "wait_FIRE_on"
		if state == "Fire_ON" and return_data == FIRE_ON_D:
			state = "blue_light_1"
			write_data = LIGHT_OFF
			Mega.write(bytes(write_data,'utf-8'))
			print("fire_finish")
				

		#For Water
		if state == "blue_light_wait_1" and return_data == BLUE_CROSS_1_D:
			state = "blue_light_2"
		if state == "blue_light_wait_2" and return_data == BLUE_CROSS_2_D:
			state = "blue_light_3"
		if state == "blue_light_wait_3" and return_data == BLUE_CIRCLE_D:
			state = "wait_WATER_on"
		if state == "WATER_on" and return_data == WATER_ON_D:
			state = "green_light_1"
			write_data = LIGHT_OFF
			Mega.write(bytes(write_data,'utf-8'))
			print("water_finish")
			write_data = CAR_OFF
			Mega.write(bytes(write_data,'utf-8'))
		

		#For Car
		if state == "green_light_wait" and return_data == GREEN_CIRCLE_D:
			state = "car"

		print(state)

	if IMU.inWaiting() >0:
		data = IMU.readline()
		
		#For Fire
		if state == "red_light_1":
			if data == b'3\r\n':
				print("red_circle_on")
				write_data = RED_CIRCLE
				Mega.write(bytes(write_data,'utf-8'))
				state = 'red_light_wait_1'
				write_data = CAR_OFF
				Mega.write(bytes(write_data,'utf-8'))
		if state == 'red_light_2':
			if data == b'down\r\n':
				print("red_tri_1_on")
				write_data = RED_TRI_1
				Mega.write(bytes(write_data,'utf-8'))
				state = 'red_light_wait_2'
		if state == 'red_light_3':
			if data == b'right\r\n':
				print("red_tri_2_on")
				write_data = RED_TRI_2
				Mega.write(bytes(write_data,'utf-8'))
				state = 'red_light_wait_3'
		if state == 'red_light_4':
			if data == b'up\r\n':
				print("red_tri_3_on")
				write_data = RED_TRI_3
				Mega.write(bytes(write_data,'utf-8'))
				state = 'red_light_wait_4'

		#For water
		if state == "blue_light_1":
			if data == b'down\r\n':
				print("blue_cross_1_on")
				write_data = BLUE_CROSS_1
				Mega.write(bytes(write_data,'utf-8'))
				state = 'blue_light_wait_1'
				write_data = CAR_OFF
				Mega.write(bytes(write_data,'utf-8'))
		if state == 'blue_light_2':
			if data == b'right\r\n':
				print("blue_cross_2_on")
				write_data = BLUE_CROSS_2
				Mega.write(bytes(write_data,'utf-8'))
				state = 'blue_light_wait_2'
		if state == 'blue_light_3':
			if data == b'3\r\n':
				print("blue_circle_on")
				write_data = BLUE_CIRCLE
				Mega.write(bytes(write_data,'utf-8'))
				state = 'blue_light_wait_3'

		#For Car
		if state == "green_light_1":
			if data == b'2\r\n':
				print("green_circle_on")
				write_data = GREEN_CIRCLE
				Mega.write(bytes(write_data,'utf-8'))
				state = 'green_light_wait'
		if state == "car":
			if data == b'up\r\n':
				print("up")
				write_data = CAR_FRONT
				Mega.write(bytes(write_data,'utf-8'))
				left = False
				right = False
			if data == b'down\r\n':
				print("down")
				write_data = CAR_BACK
				Mega.write(bytes(write_data,'utf-8'))
				left = False
				right = False
			if data == b'right\r\n':
				print("Stop")
				write_data = CAR_OFF
				Mega.write(bytes(write_data,'utf-8'))
				state = 'finsih'
			if data == b'2\r\n' and left == False:
				print("left")
				write_data = CAR_LEFT
				Mega.write(bytes(write_data,'utf-8'))
				left = True
				right = False
			if data == b'1\r\n'and right == False:
				print("right")
				write_data = CAR_RIHGT
				Mega.write(bytes(write_data,'utf-8'))
				right = True
				left = False

		
