import serial
import os
import glob
import time
import datetime
import numpy as np
from time import strftime
from daqhats import mcc118, OptionFlags, HatIDs, HatError
from daqhats_utils import select_hat_device, enum_mask_to_string
import gpiod
import time

time.sleep(5)


def blink():
    RED_LED = 16
    chip = gpiod.Chip('gpiochip4')
    RED_line = chip.get_line(RED_LED) 
    RED_line.request(consumer="LED", type=gpiod.LINE_REQ_DIR_OUT)

    RED_line.set_value(1)
    time.sleep(0.04)
    RED_line.set_value(0)
    time.sleep(0.04)
    RED_line.release()
    #lcd_string("CO2: " + str(CO2) + "ppm", LCD_LINE_1)
    # print(CO2)
    

# now=datetime.now()
StartTime=strftime("%Y%m%d%H%M%S")
Filename=str(StartTime)+".csv"
FileLocation="/home/field/DataFolder/"+Filename
Header="Timestamp, CO2 (ppm), CH4 (ppm), CH4 Voltage (V), Pressure (bar), Temperature (C)"
Export=[]
np.savetxt(FileLocation, Export, delimiter=",",header=Header)


#Configure Thermal Probe - (1 Wire)
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28-00000f8beceb')[0]
device_file = device_folder + '/w1_slave'

def read_temp_raw():
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

#CELSIUS CALCULATION
def read_temp_c():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        lines = read_temp_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = int(temp_string) / 1000.0 # TEMP_STRING IS THE SENSOR OUTPUT, MAKE SURE IT'S AN INTEGER TO DO THE MATH
        temp_c = str(round(temp_c, 1)) # ROUND THE RESULT TO 1 PLACE AFTER THE DECIMAL, THEN CONVERT IT TO A STRING
        return temp_c
        

#----------------------------------------------------------------------------------------

#Configure Serial Connection - RS232 @3.3V - CO2 Probe
ser = serial.Serial('/dev/ttyAMA0', 9600,timeout=1)  # open serial port

commandstring="z\r\n"
calibratestring ="G\r\n"

ser.write(calibratestring.encode('utf-8'))
s = ser.readline()  
print (str(s))
time.sleep(2)

ScalingFactor=".\r\n"

ser.write(ScalingFactor.encode('utf-8'))
s = ser.readline()  
print (str(s))

time.sleep(2)
#------------------------------------------------------------------------------------------

#Configure Analog DAQ Hat

options = OptionFlags.DEFAULT
low_chan = 0
high_chan = 3
mcc_118_num_channels = mcc118.info().NUM_AI_CHANNELS
address = select_hat_device(HatIDs.MCC_118)
hat = mcc118(address)


# LCD Display

chip = gpiod.Chip('gpiochip0')

lines = {
	'LCD_RS': chip.get_line(17),
	'LCD_E': chip.get_line(2),
	'LCD_D4': chip.get_line(24),
	'LCD_D5': chip.get_line(19),
	'LCD_D6': chip.get_line(27),
	'LCD_D7': chip.get_line(23),
}

for line in lines.values():
    print("Check")
    line.request(consumer='lcd', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
    x = "Good"
    print(x)

LCD_WIDTH = 16 # Maximum characters per line
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

# Timing constants
E_PULSE = 0.0001
E_DELAY = 0.0001

def set_line(line_name, value):
	lines[line_name].set_value(1 if value else 0)


def lcd_init():
	# Initialize display
	""" GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LCD_E, GPIO.OUT)
	GPIO.setup(LCD_RS, GPIO.OUT)
	GPIO.setup(LCD_D4, GPIO.OUT)
	GPIO.setup(LCD_D5, GPIO.OUT)
	GPIO.setup(LCD_D6, GPIO.OUT)
	GPIO.setup(LCD_D7, GPIO.OUT)
	GPIO.setup(LED_ON, GPIO.OUT) """
	
	lcd_byte(0x33, LCD_CMD) # 110011 Initialise
	lcd_byte(0x32, LCD_CMD)
	lcd_byte(0x28, LCD_CMD)
	lcd_byte(0x0C, LCD_CMD)
	lcd_byte(0x06, LCD_CMD)
	lcd_byte(0x01, LCD_CMD)
	time.sleep(0.05)
	
def lcd_byte(bits, mode):
	# Send byte data to pins
	# bits = data 
	# mode = True for character
	#		 False for command
	
	#LCD_RS.value = mode # RS
	
	set_line('LCD_RS', mode)
	
	# High bits
	
	set_line('LCD_D4', bits & 0x10)
	set_line('LCD_D5', bits & 0x20)
	set_line('LCD_D6', bits & 0x40)
	set_line('LCD_D7', bits & 0x80)
    
    # Toggle 'Enable' Pin
	lcd_toggle_enable()
	
	# Low bits
	
	set_line('LCD_D4', bits & 0x01)
	set_line('LCD_D5', bits & 0x02)
	set_line('LCD_D6', bits & 0x04)
	set_line('LCD_D7', bits & 0x08)
    
    # Toggle 'Enable' Pin
	lcd_toggle_enable()
	
def lcd_toggle_enable():
	# Toggle enable
	time.sleep(E_DELAY)
	set_line('LCD_E', 1)
	time.sleep(E_PULSE)
	set_line('LCD_E', 0)
	time.sleep(E_DELAY)
	
def lcd_string(message, line):
	# Send string to display
	
	
	message = message.ljust(LCD_WIDTH, " ")
	lcd_byte(line, LCD_CMD)
	
	for i in range(LCD_WIDTH):
		lcd_byte(ord(message[i]), LCD_CHR)
        
def cleanup():
	lcd_byte(0x01, LCD_CMD) 

#Read Values

count = 0
#def cut_power(LEL_Check):
    #if 0.3 < LEL_Check < 0.6:
        #count += 1
        #if count > 10:
            #print("Initiating shutdown of Raspberry Pi")
            #os.system("sudo shutdown -h now")
            #print("Shutdown command issued. The Raspberry Pi is powering off shortly")
        
        #else:
            #print("Methane Below LEL")
    

x=True
while x==True:
    lcd_init()
    ser.write(commandstring.encode('utf-8'))
    s = ser.readline()       # read up to one hundred bytes
    s=str(s)
    #CO2=int(s[5:10])*100
    CO2=int(s[5:10])*100 * 1.0535 + 567.79
    lcd_string("CO2: " + str(CO2) + "ppm", LCD_LINE_1)
    Temp=float(read_temp_c())
    CH4Volt=hat.a_in_read(0)
    CH4Con = round(hat.a_in_read(0) *6962.8 - 8980.6, 4)
    lcd_string("CH4: " + str(CH4Con) + "ppm", LCD_LINE_2)
    
    
    AirPressV=hat.a_in_read(1)
    AirPressureMbar= (AirPressV-0.33)*0.606 #In Bar
    
    now=int(strftime("%Y%m%d%H%M%S"))

    Header="Timestamp,CO2 (ppm), CH4 (ppm), CH4 Voltage (V), Pressure (bar), Temperature (C)"
    Export=[]
   
    Export.append([now,CO2,CH4Con,CH4Volt,AirPressureMbar,Temp])
    
    #print("Temp:"+str(read_temp_c()))
    #print("Pres:"+str(AirPressureMbar))
    #print("CH4:"+str(CH4Con))
    #print("CO2:"+str(CO2))


    #cut_power(CH4Volt)
    blink()
    
    
    with open(FileLocation, "ab") as q:
        np.savetxt(q, Export, fmt='%1.3f', delimiter=",")
    q.close

    del Export
        
    
