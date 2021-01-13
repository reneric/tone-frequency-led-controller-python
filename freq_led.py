#!/usr/bin/env python
import pyaudio
from numpy import zeros,linspace,short,fromstring,hstack,transpose,log
from scipy import fft
from time import sleep
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
pins=[5,6,16]
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)

# p = GPIO.PWM(13, 100)
# p.start(0)

# Margin of error
BANDWIDTH = 10
# How many 46ms segments before we determine it is a legitimate tone
triggerlength=40
# How many false 46ms blips before we declare the alarm is not ringing
resetlength=10
# Enable debug output
debug=False
# The frequency in which the channels begin (Hz)
CHANNEL_START=500
# The number of channels connected
CHANNEL_COUNT=len(pins)
# The size of each channel (Hz)
CHANNEL_SIZE=100



# Set up sampler
NUM_SAMPLES = 2048
SAMPLING_RATE = 44100
pa = pyaudio.PyAudio()
_stream = pa.open(format=pyaudio.paInt16,
                  channels=1, rate=SAMPLING_RATE,
                  input=True,
                  frames_per_buffer=NUM_SAMPLES)

print("Frequency detector LED controller working. Press CTRL-C to quit.")


blipcounts=[]
resetcounts=[]
clearcounts=[]

max_freq=0
channel_start_freqs=[]
channel_end_freqs=[]
status=[]

def initialize():
    for i in range(0,L_COUNT):
        led = i + 1
        channel_start_freq = CHANNEL_START + (i * CHANNEL_SIZE)
        CHANNEL_COUNT = channel_start_freq + CHANNEL_SIZE - 1
        
        channel_start_freqs.append(channel_start_freq)
        channel_end_freqs.append(channel_end_freq)
        status.append(False)
        blipcounts.append(0)
        resetcounts.append(0)
        clearcounts.append(0)
        max_freq=channel_end_freq
        print('Channel %s (GPIO %s): Range %sHz - %sHz' % (led, pins[i], channel_start_freq, channel_end_freq))


def determine_channel_num(fq):
    channel=0
    for i in range(0,L_COUNT):
        channel_start_freq = channel_start_freqs[i]
        channel_end_freq = channel_end_freqs[i]
        if fq >= channel_start_freq and fq <= channel_end_freq:
            channel=i+1
    return channel

def get_freq():
    while _stream.get_read_available()< NUM_SAMPLES: sleep(0.01)
    audio_data  = fromstring(_stream.read(
         _stream.get_read_available()), dtype=short)[-NUM_SAMPLES:]
    # Each data point is a signed 16 bit number, so we can normalize by dividing 32*1024
    normalized_data = audio_data / 32768.0
    intensity = abs(fft(normalized_data))[:NUM_SAMPLES/2]
    
    which = intensity[1:].argmax()+1    # use quadratic interpolation around the max
    if which != len(intensity)-1:
        y0,y1,y2 = log(intensity[which-1:which+2:])
        x1 = (y2 - y0) * .5 / (2 * y1 - y2 - y0)
        # find the frequency and output it
        return (which+x1)*SAMPLING_RATE/NUM_SAMPLES
    else:
        return which*SAMPLING_RATE/NUM_SAMPLES


initialize()


while True:
    frequency = get_freq()

    channel=determine_channel_num(frequency)
    channel_index=channel-1
    
    if frequency > CHANNEL_START and frequency < max_freq:
        CHANNEL_COUNT=determine_channel_num(frequency)

        if channel:
            blipcounts[channel_index]+=1
            resetcounts[channel_index]=0
            print(blipcounts[channel_index])
            if (blipcounts[channel_index]>=triggerlength) and not status[channel_index]:
                blipcounts[channel_index]=0
                resetcounts[channel_index]=0
                print('LED %s' % (channel))
                status[channel_index]=True
        else:
            for i in range(0, len(pins)):
                blipcounts[i]=0
                resetcounts[i]+=1
                if debug: print "\t\t\treset",resetcounts[i]
                if (resetcounts[i]>=resetlength): resetcounts[i]=0
              
              
    elif frequency > 5000 and frequency < 6000:
        for i in range(0, len(status)):
            status[i]=False
      
    else:
        # print('Channel: %s' % (channel))
        for i in range(0, len(pins)):
            blipcounts[i]=0
            resetcounts[i]+=1
            if debug: print "\t\t\treset",resetcounts[i]
            if (resetcounts[i]>=resetlength): resetcounts[i]=0

    for i in range(0, len(pins)):
        if status[i]:
            GPIO.output(pins[i], GPIO.HIGH)
        else:
            GPIO.output(pins[i], GPIO.LOW)
      
