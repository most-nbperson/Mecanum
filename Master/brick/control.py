__author__ = 'Brick'
import serial
import serial.tools.list_ports
import string
import time

		
		
class Car:
	def __init__(self):
		self.COM="/dev/ttyAMA0"
		self.open = False
		self.buf=''
		self.data=''
		res=self.create_com()
		self.LastError=0
		self.LastErrorR=0
	def create_com(self,COM='auto', baud_rate=115200):
		if self.open == True:
			self.ser.close()
		self.baud_rate = baud_rate		
		try:
			self.ser=serial.Serial(self.COM, self.baud_rate)
			self.open=True
		except:
			print('Error:Couldn\'t open serial port!')
			self.open = False
			time.sleep(1)
		return self.open
			

		
		
	def receive(self,count):
		if self.open==True:
			self.buf=self.ser.read(count)
			#self.data= str(binascii.b2a_hex(self.buf))[2:-1]
			print(self.buf)	
			
	def Forward(self):
		self.ser.write('-a-0-000'.encode())
		print('-F-')

	def Backward(self):
		self.ser.write('-b-0-000'.encode())
		print('-B-')
	def Left(self):
		self.ser.write('-e-0-000'.encode())
		print('-L-')

	def Right(self):
		self.ser.write('-f-0-000'.encode())
		print('-R-')
	def LL(self):
		self.ser.write('-c-0-000'.encode())
		print('-LL-')
	def RR(self):
		self.ser.write('-d-0-000'.encode())
		print('-RR-')

	def Stop(self):
		self.ser.write('-s-0-000'.encode())
		print('-S-')

	def auto_control(self,Dangle,dist,angleTH=70.0):
		x=1 if Dangle>=0  else 0
		self.ser.write('-x-{}-{:0>3d}'.format(x,int(abs(Dangle))).encode())
		print('Dangle:',Dangle)
		time.sleep(0.5)
		# if abs(Dangle)>angleTH:
		# 	self.rotate_track(Dangle)
		# else:
		# 	self.line_track(Dangle)


	def rotate_track(self,Dangle,P=2.3,D=1):
		dV = P*Dangle 
		if abs(dV)<310:
			dV=310*(dV/abs(dV))
		#print('rotate',Dangle,dV)
		# self.ser.write('-RR-0-'.encode())

	def line_track(self,Dangle,BaseSpeed=380,BP=2.0,PP=0.00015,D=7):
		P=BP+PP*(Dangle**2)
		dV=P*Dangle + D*(Dangle - self.LastError)
		self.LastError=Dangle
		BaseSpeed=500-1000.0*abs(Dangle)/180.0


	def Lidar_process(self):
		i=0
		j=0
		while True:
			
			if self.open == False:
				res=self.create_com()
			
			self.Lidar_Get()
			count = self.ser.inWaiting()  
			if count != 0: 
				self.receive(count)
				j=j+1

				self.ser.write(s)
				
				

			
if __name__ == "__main__":
	ldr=Car()
	#ldr.Lidar_process()
			
			
			
		
		
		
		
		
		
		
		
		
		
		
		
