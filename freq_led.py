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
import sys
import argparse

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

# Set the PWM frequency
pca.frequency = 500

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Arguments
parser = argparse.ArgumentParser()
parser.add_argument('-v', dest='verbose', action="store_true", default=False)
parser.add_argument('-d', dest='debug', action="store_true", default=False)
parser.add_argument('-ds', dest='dimmer', action="store", default=0.4, type=float)
args = parser.parse_args()


# Margin of error
BANDWIDTH = 10
# How many 46ms segments before we determine it is a legitimate tone
triggerlength=8
# How many false 46ms blips before we declare the alarm is not ringing
resetlength=10
# Enable debug output
debug=args.debug
# Enable verbose output
verbose=args.verbose
# The frequency in which the channels begin (Hz)
CHANNEL_START=1100
# The number of channels connected
CHANNEL_COUNT=16
# The size of each channel (Hz)
CHANNEL_SIZE=100
# LED Dim speed
LED_DIM_SPEED_SECONDS=args.dimmer
LED_DIM_SPEED=round((0.000671397/LED_DIM_SPEED_SECONDS)*65535)
# Set up sampler
NUM_SAMPLES = 2048
SAMPLING_RATE = 44100
pa = pyaudio.PyAudio()
_stream = pa.open(format=pyaudio.paInt16,
                  channels=1, rate=SAMPLING_RATE,
                  input=True,
                  frames_per_buffer=NUM_SAMPLES)

print("Frequency detector LED controller working. Press CTRL-C to quit.")
print("LED dim speed: %s seconds" % args.dimmer)
if verbose: print("LED DIMMER: %s" % LED_DIM_SPEED)
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
    if verbose: print('Channel %s: Range %sHz - %sHz' % (led, channel_start_freq, channel_end_freq))

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
    for i in reversed(range(0, 0xffff, LED_DIM_SPEED)):
        pca.channels[led].duty_cycle = i


def all_on(affected_channels=[]):
    processes=[]
    for channel in affected_channels:
        p = Process(target=turn_on_led, args=(channel,))
        processes.append(p)
        p.start()
    for t in processes:
        t.join()

def all_off(affected_channels=[]):
    processes=[]
    for channel in affected_channels:
        p = Process(target=turn_off_led, args=(channel,))
        processes.append(p)
        p.start()
    for t in processes:
        t.join()

while True:
    frequency = round(get_freq())
    
    # If frequency is within the LED single channel range
    if frequency > CHANNEL_START and frequency < max_freq:
        channel=determine_channel_num(frequency)
        channel_index=channel-1

        if channel:
            if debug: print('%sHz' % frequency)
            channelhitcounts[channel_index]+=1
            resetcounts[channel_index]=0
            if (channelhitcounts[channel_index]>=triggerlength):
                channelhitcounts[channel_index]=0
                resetcounts[channel_index]=0
                if debug: print('\nLED %s\n' % (channel))
                status[channel_index]=frequency < channel_start_freqs[channel_index] + 50
        else:
            for i in range(0, CHANNEL_COUNT):
                channelhitcounts[i]=0
                resetcounts[i]+=1
                # if debug: print('resetcount: %s' % resetcounts[i])
                if (resetcounts[i]>=resetlength): resetcounts[i]=0
              
              
    
    else:
        for i in range(0, CHANNEL_COUNT):
            channelhitcounts[i]=0
            resetcounts[i]+=1
            # if debug: print('resetcount: %s' % resetcounts[i])
            if (resetcounts[i]>=resetlength): resetcounts[i]=0
    
    
    # All On
    if frequency >= ALL_ON_FREQ and frequency < ALL_ON_FREQ + CHANNEL_SIZE:
        already_on = check_statuses(status)
        affected_channels = []
        if not already_on:
            for i in range(0, CHANNEL_COUNT):
                if verbose: print('Channel %s: %s' % (i + 1, 'ON'))
                # Only turn ON lights if they are currently OFF
                if not status[i]: affected_channels.append(i)
                laststatus[i] = True
                status[i] = True
            if verbose: print('---------------------')
            all_on(affected_channels)
    
    # All Off
    if frequency >= ALL_OFF_FREQ and frequency < ALL_OFF_FREQ + CHANNEL_SIZE:
        already_off = check_statuses(status, False)
        affected_channels = []
        if not already_off:
            for i in range(0, CHANNEL_COUNT):
                if verbose: print('Channel %s: %s' % (i + 1, 'OFF'))
                # Only turn OFF lights if they are currently ON
                if status[i]: affected_channels.append(i)
                laststatus[i] = False
                status[i] = False
            if verbose: print('---------------------')
            all_off(affected_channels)
    
    # If the frequency is greater than the two all on/off channels
    if frequency > ALL_OFF_FREQ + CHANNEL_SIZE:
        # Update the LED statuses if they have changed
        if laststatus != status:
            for i in range(0, CHANNEL_COUNT):
                if verbose: print('Channel %s: %s' % (i + 1, 'ON' if status[i] else 'OFF'))
                if laststatus[i] != status[i]:
                    if status[i]:
                        turn_on_led(i)
                    else:
                        turn_off_led(i)
                    laststatus[i] = status[i]
            if verbose: print('---------------------')

            
      
