# SETUP:
import hub, utime, math
hub.display.clear()

#Motors
leftMotor = hub.port.F.motor
rightMotor = hub.port.E.motor
pair = leftMotor.pair(rightMotor)

#Sensors:
sonicSensor = hub.port.D.device
lightSensor = hub.port.C.device
force = hub.port.A.device # only used for training AI

#Initialize Raspberry Pi:
pi = hub.port.B
pi.mode(hub.port.MODE_FULL_DUPLEX)
pi.baud(115200)
music = [] # cycling array of length 10 to be populated with music detection signals
partner_pos = []
count = 0


#Driving Physics:
radIn = 83 # turn radius for inner wheel (in mm)
radOut = radIn + 100 # 100 mm between wheels
speedIn = 28
speedInDist = float(speedIn*5.3772) - 24.637
speedOutDist = (float(speedInDist/radIn) * radOut)
speedOut = int(float((speedOutDist + 24.637) / 5.3772) + 0.5)
boost = -5 #added to pwm input to increase or decrease speed, depending on dancer's position

# Distance control:
Kp = 1
setpoint = 20 # brake if we are closer than 20cm to another dancer

#FLOOR TAPE RECOGNITION TRAINING:
utime.sleep(5)
force.mode(0)
training = []
#train for case A then B
for case in range(2): #case 0 is carpet, case 1 is tape
    for counter in range(5):
        hub.display.show(str(counter + 1))
        while not force.get()[0]:
            utime.sleep(.1)
        brightness = lightSensor.get()[0]
        print("Case " + str(case) + ": " + str(brightness))
        while force.get()[0]:
            utime.sleep(.1)
        training.append((case,brightness))

hub.display.show('H')
while not force.get()[0]:
    utime.sleep(.1)
hub.display.show('G')

#FUNCTION DEFINITIONS:
def isTape():
    brightness = lightSensor.get()[0] # read reflected light intensity from sensor

    min = 1000
    tape = False
    for (c,l) in training:
        dist = abs(brightness - l)
        if dist < min:
            min = dist
            tape = c #if case == 1, tape is detected

    return tape

def drive():
    dist = sonicSensor.get()[0]
    if dist is None: # if error getting distance
        pair.pwm(-speedIn - boost, speedOut + boost) # continue driving
    else:
        error = setpoint - dist
        brake = max(0, int(Kp * error)) # if dist > setpoint, brake is 0
        pair.pwm(-speedIn - boost + brake, speedOut + boost - brake)

def getData():
    text = "(" # holds full string to return
    c = '' # holds one char from serial

    while c != '(': # pull values off of serial until the start of an ordered pair
        try:
            c = pi.read(1).decode() # read one byte off of serial
        except:
            c = ''

    while c != ')': # pull values off of serial until the end of an ordered pair
        try:
            c = pi.read(1).decode() # read one byte off of serial
            if c is '(': # bug fix
                return "(0 300)"
            text += c
        except:
            c = ''

    return text

def parse(msg):
    data = msg.strip("()").split() # break ordered pair into array
    music = int(data[0])
    x_coord = int(data[1])

    return (music, x_coord)

def steer(x):
    setpoint = 300 # center of frame
    P = 0.025 # proportional gain

    error = setpoint - x # negative if x coord is greater than 300
    return int(P * error)


#MAIN LOOP:
while True:
    if isTape():
        boost *= -1 # when you cross the tape, speed up or slow down
        print("Tape! Boost = " + str(boost))
        while isTape(): # wait to get off tape
            utime.sleep(0.05)

    message = getData() #read data from serial into ordered pair: (m x)
    print(message)
    current_music, x_coord = parse(message) # parse values into variables

    # Add current readings into queues:
    music.append(current_music)
    partner_pos.append(x_coord)

    # Maintain max length of queues:
    if len(music) > 10:
        music.pop(0)
    if len(partner_pos) > 5:
        partner_pos.pop(0)

    avg_pos = sum(partner_pos)/len(partner_pos) # get average of last 5 positions
    bias = steer(avg_pos)

    if (1 in music):
        pair.pwm(-speedIn - boost - bias, speedOut + int(1.4*boost) - bias)
    else:
        pair.pwm(0,0)

    utime.sleep(0.1)

pair.pwm(0,0)