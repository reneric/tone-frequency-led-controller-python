#!/usr/bin/env python
import pyaudio
from multiprocessing import Process
from numpy import zeros,linspace,short,fromstring,hstack,transpose,log,frombuffer
from scipy import fft
from time import sleep
import RPi.GPIO as GPIO
import board
import busio
import adafruit_pca9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

# Set the PWM frequency
pca.frequency = 500

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


# Margin of error
BANDWIDTH = 10
# How many 46ms segments before we determine it is a legitimate tone
triggerlength=8
# How many false 46ms blips before we declare the alarm is not ringing
resetlength=10
# Enable debug output
debug=
# The frequency in which the channels begin (Hz)
CHANNEL_START=1100
# The number of channels connected
CHANNEL_COUNT=16
# The size of each channel (Hz)
CHANNEL_SIZE=100
# LED Dim speed
LED_DIM_SPEED=100


# Set up sampler
NUM_SAMPLES = 2048
SAMPLING_RATE = 44100
pa = pyaudio.PyAudio()
_stream = pa.open(format=pyaudio.paInt16,
                  channels=1, rate=SAMPLING_RATE,
                  input=True,
                  frames_per_buffer=NUM_SAMPLES)

print("Frequency detector LED controller working. Press CTRL-C to quit.")

ALL_ON_FREQ=700
ALL_OFF_FREQ=800

channelhitcounts=[]
resetcounts=[]
clearcounts=[]

max_freq=0
channel_start_freqs=[]
channel_end_freqs=[]
status=[]
laststatus=[]


for i in range(0,CHANNEL_COUNT):
    led = i + 1
    channel_start_freq = CHANNEL_START + (i * CHANNEL_SIZE)
    channel_end_freq = channel_start_freq + CHANNEL_SIZE - 1
    
    channel_start_freqs.append(channel_start_freq)
    channel_end_freqs.append(channel_end_freq)
    status.append(False)
    laststatus.append(False)
    channelhitcounts.append(0)
    resetcounts.append(0)
    clearcounts.append(0)
    max_freq=channel_end_freq
    print('Channel %s: Range %sHz - %sHz' % (led, channel_start_freq, channel_end_freq))

def check_statuses(st, flag=True):  
    chk = True
    for item in st: 
        if item != flag: 
            chk = False
            break; 
              
    return chk

def determine_channel_num(fq):
    channel=0
    for i in range(0,CHANNEL_COUNT):
        channel_start_freq = channel_start_freqs[i]
        channel_end_freq = channel_end_freqs[i]
        if fq >= channel_start_freq and fq <= channel_end_freq:
            channel=i+1
    return channel

def get_freq():
    while _stream.get_read_available()< NUM_SAMPLES: sleep(0.01)
    audio_data  = frombuffer(_stream.read(_stream.get_read_available()), dtype=short)[-NUM_SAMPLES:]
    # Each data point is a signed 16 bit number, so we can normalize by dividing 32*1024
    normalized_data = audio_data / 32768.0
    intensity = abs(fft.fft(normalized_data))[:NUM_SAMPLES//2]
    
    which = intensity[1:].argmax()+1    # use quadratic interpolation around the max
    if which != len(intensity)-1:
        y0,y1,y2 = log(intensity[which-1:which+2:])
        x1 = (y2 - y0) * .5 / (2 * y1 - y2 - y0)
        # find the frequency and output it
        return (which+x1)*SAMPLING_RATE/NUM_SAMPLES
    else:
        return which*SAMPLING_RATE/NUM_SAMPLES


def turn_on_led(led):
    if debug: print('turn_on_led %s' % led)
    for i in range(0, 0xffff, LED_DIM_SPEED):
        pca.channels[led].duty_cycle = i

def turn_off_led(led):
    if debug: print('turn_off_led %s' % led)
    for i in range(0xffff, 0, -LED_DIM_SPEED * 10):
        pca.channels[led].duty_cycle = i
onprocesses=[]
offprocesses=[]
def all_on():
    for channel in range(0, CHANNEL_COUNT):
        p = Process(target=turn_on_led, args=(channel,))
        onprocesses.append(p)
        p.start()
    for t in onprocesses:
        t.join()

def all_off():
    for channel in range(0, CHANNEL_COUNT):
        p = Process(target=turn_off_led, args=(channel,))
        offprocesses.append(p)
        p.start()
    for t in offprocesses:
        t.join()

while True:
    frequency = get_freq()
    
    if frequency > CHANNEL_START and frequency < max_freq:
        channel=determine_channel_num(frequency)
        channel_index=channel-1

        if channel:
            if debug: print(frequency)
            channelhitcounts[channel_index]+=1
            resetcounts[channel_index]=0
            if debug: print(channelhitcounts[channel_index])
            if (channelhitcounts[channel_index]>=triggerlength):
                channelhitcounts[channel_index]=0
                resetcounts[channel_index]=0
                if debug: print('LED %s' % (channel))
                status[channel_index]=frequency < channel_start_freqs[channel_index] + 50
        else:
            for i in range(0, CHANNEL_COUNT):
                channelhitcounts[i]=0
                resetcounts[i]+=1
                if debug: print('reset' % resetcounts[i])
                if (resetcounts[i]>=resetlength): resetcounts[i]=0
              
              
    
    else:
        # print('Channel: %s' % (channel))
        for i in range(0, CHANNEL_COUNT):
            channelhitcounts[i]=0
            resetcounts[i]+=1
            if debug: print('reset' % resetcounts[i])
            if (resetcounts[i]>=resetlength): resetcounts[i]=0
    if frequency >= ALL_ON_FREQ and frequency < ALL_ON_FREQ + CHANNEL_SIZE:
        already_on = check_statuses(status)
        if not already_on:
            for i in range(0, CHANNEL_COUNT):
                print('Channel %s: %s' % (i + 1, 'ON'))
                laststatus[i] = True
                status[i] = True
            print('---------------------')
            all_on()
        
    elif frequency >= ALL_OFF_FREQ and frequency < ALL_OFF_FREQ + CHANNEL_SIZE:
        already_off = check_statuses(status, False)
        if not already_off:
            for i in range(0, CHANNEL_COUNT):
                print('Channel %s: %s' % (i + 1, 'OFF'))
                laststatus[i] = False
                status[i] = False
            print('---------------------')
            all_off()
    elif frequency > ALL_OFF_FREQ + CHANNEL_SIZE:
        # Update the LED statuses if they have changed
        if laststatus != status:
            for i in range(0, CHANNEL_COUNT):
                print('Channel %s: %s' % (i + 1, 'ON' if status[i] else 'OFF'))
                if laststatus[i] != status[i]:
                    if status[i]:
                        turn_on_led(i)
                    else:
                        turn_off_led(i)
                    laststatus[i] = status[i]
            print('---------------------')
    else:
        None
            
      
