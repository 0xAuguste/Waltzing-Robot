# import the necessary packages
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
import pyaudio
from array import array
import serial

# SETUP VIDEO:-------------------------------------

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (23, 26, 109)
greenUpper = (82, 255, 240)

#greenLower = (29, 86, 6)
#greenUpper = (64, 255, 255)

# grab the reference to the webcam
print("Booting camera...")
vs = VideoStream(src=0).start()
time.sleep(2.0) # allow the camera or video file to warm up

# SETUP AUDIO:-------------------------------------

ser = serial.Serial(port='/dev/serial0', 
					baudrate=115200, 
					parity=serial.PARITY_NONE,
					stopbits=serial.STOPBITS_ONE,
					bytesize=serial.EIGHTBITS,
					timeout=1,
					write_timeout=1
)

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100 # 44100 Hz sampling rate

p = pyaudio.PyAudio()

stream = p.open(
			format=FORMAT,
			channels=CHANNELS,
			rate=RATE,
			input=True,
			output=True,
			frames_per_buffer=CHUNK,
			input_device_index = 1
)

rawI = stream.read(CHUNK, exception_on_overflow = False) # read in as bytes, has length 2*CHUNK
# 'h' is for signed short since 2 bytes are given for each number
data_intI = array('h', rawI).tolist() # has length CHUNK

timeVec = np.arange(0, 2*CHUNK, 2)

data = data_intI*np.hanning(len(data_intI)) # hanning window
N_FFT = len(data)
freqVec = (float(RATE)*np.arange(0,int(N_FFT/2)))/N_FFT
FFTDataRaw = np.abs(np.fft.fft(data)) # calculate FFT
FFTData = FFTDataRaw[0:int(N_FFT/2)]/float(N_FFT)
freqVec = freqVec[50:]
FFTData = FFTData[50:]

# FUNCTION DEFINITIONS:-------------------------------------

# isMusic()
# Reads chunk of sound pressure levels from microphone, computes
# fast Fourier Transform. Returns true if amplitude spikes in
# frequency range not correlated to motor noise
def isMusic(stream):
	uniqueFreqs = []
	musicDetected = False
	raw = stream.read(CHUNK, exception_on_overflow = False) # read in as bytes, has length 2*CHUNK
	# 'h' is for signed short since 2 bytes are given for each number
	data_int = array('h', raw).tolist() # has length CHUNK
	soundRange = np.std(data_int)
	
	data = data_int*np.hanning(len(data_int)) # hanning window
	N_FFT = len(data)
	freqVec = (float(RATE)*np.arange(0,int(N_FFT/2)))/N_FFT
	FFTDataRaw = np.abs(np.fft.fft(data)) # calculate FFT
	FFTData = FFTDataRaw[0:int(N_FFT/2)]/float(N_FFT)
	FFTData = FFTData[50:]
	for i in range(len(FFTData)):
		if (FFTData[i] > 3):
			if (not(freqVec[i] > 700 and freqVec[i] < 750)
				and not(freqVec[i] > 1000 and freqVec[i] < 1030)
				and not(freqVec[i] > 880 and freqVec[i] < 920)
				and not(freqVec[i] > 150 and freqVec[i] < 210)):
					musicDetected = True

	return musicDetected

def xCoord(video):
	frame = video.read()
	if frame is None:
		return 300 # center of frame

	# resize the frame, blur it, and convert it to the HSV color space
	frame = imutils.resize(frame, height=600)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		if radius > 20:
			return int(y)
	
	
	return 300 #center


# MAIN LOOP:-------------------------------------
while True:
	music = isMusic(stream)
	green_pos = xCoord(vs)

	message = "(%d %d)" % (music, green_pos)

	print(message)
	ser.write(message.encode())

	time.sleep(0.1)

vs.stop() # stop the camera video stream
cv2.destroyAllWindows() # close all windows