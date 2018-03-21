#!/usr/bin/env python

# Using object-oriented, tested code using the language that you feel most proficient in, 
# please write a "pico y placa" predictor. The inputs should be a license plate number 
# (the full number, not the last digit), a date (as a String), and a time, and the program 
# will return whether or not that car can be on the road.

import datetime

class LicencePlate:
	def __init__(self):
		self.Number = 0
		self.LastNumber = 0
		self.Day = 0
		self.Hour = 0
	
	def __del__(self):
		self.Number = 0
		self.LastNumber = 0
		self.Day = 0
		self.Hour = 0

	def inputNumber(self):
		while True:
			number = raw_input("Licence Plate Number: ")
			if (self.validateNumber(number) == False):
				print("\nError: Format is incorrect. Please enter a 3 digit number.\n")
			elif (len(str(number))>3):
				print("\nError: Format is incorrect. Please enter a 3 digit number.\n")
			else:
				self.Number = int(number)
				self.LastNumber = (int(number)%100)%10
				break

	def validateNumber(self,number):
		try:
			return int(number)
		except ValueError:
			return False

	def inputDate(self):
		while True:
			date = raw_input("Date (MM/DD/YYYY): ")
			if (self.validateDate(date) == False):
				print("\nError: Format is incorrect. Please enter it as MM/DD/YYYY.\n")
			else:
				self.Day = datetime.datetime.strptime(date,'%d/%m/%Y').weekday()
				break

	def validateDate(self, date):
		try:
			return datetime.datetime.strptime(date,'%d/%m/%Y')
		except ValueError:
			return False

	def inputTime(self):
		while True:
			time = raw_input("Time (HH:MM): ")
			if (self.validateTime(time) == False):
				print("\nError: Format is incorrect. Please enter it as HH:MM.\n")
			else:
				self.Hour = datetime.datetime.strptime(time,'%H:%M').hour
				break
				
	def validateTime(self, time):
		try:
			return datetime.datetime.strptime(time,'%H:%M')
		except ValueError:
			return False

	def checkRestrictions(self):
		if (self.Hour>=7 and self.Hour<10 or self.Hour>=16 and self.Hour<19):
			if (self.LastNumber==0 or self.LastNumber==1 and self.Day==0):
				return False
			if(self.LastNumber==2 or self.LastNumber==3 and self.Day==1):
				return False
			elif (self.LastNumber==4 or self.LastNumber==5 and self.Day==2):
				return False
			elif (self.LastNumber==6 or self.LastNumber==7 and self.Day==3):
				return False
			elif (self.LastNumber==8 or self.LastNumber==9 and self.Day==4):
				return False
			else:
				return True
		else:
			return True

	def printInfo(self):
		print "\nThe information you entered was: \n"
		print "Plate Number: ", self.Number
		if self.Day == 0: print "Day:           Monday"
		if self.Day == 1: print "Day:           Tuesday"
		if self.Day == 2: print "Day:           Wednesday"
		if self.Day == 3: print "Day:           Thurday"
		if self.Day == 4: print "Day:           Friday"
		if self.Day == 5: print "Day:           Saturday"
		if self.Day == 6: print "Day:           Sunday"
 		print "Hour:         ", self.Hour

	def evaluatePlate(self):
		print("Please enter the following information: \n")
		self.inputNumber()
		self.inputDate()
		self.inputTime()
		self.printInfo()
		if (self.checkRestrictions() == True): 
			print "\n>>Road circulation permited<<\n"
		else:
			print "\n>>Road circulation NOT permited<<\n"
		print "\n|-----------------------------------------|\n\n"

if __name__ == '__main__':
	print "Program started.\n"
	test = LicencePlate()
	while True:
		test.evaluatePlate()
		ans = raw_input("Do you wish to enter different information? [Y/N]")
		if (ans == 'N' or ans == 'n' or ans == 'no' or ans == 'No') : 
			print "Program stopped.\n"
			break
