#!/usr/bin/env python
import pyaudio
from threading import Timer
from numpy import zeros,linspace,short,fromstring,hstack,log10,log,frombuffer
from scipy import fft
from time import sleep
import math
import RPi.GPIO as GPIO
import board
import busio
import os
import adafruit_pca9685
import sys
from datetime import datetime, date
import argparse
import struct
import time
i2c = busio.I2C(board.SCL, board.SDA)


pca1 = adafruit_pca9685.PCA9685(address=0x40, i2c_bus=i2c)
pca2 = adafruit_pca9685.PCA9685(address=0x41, i2c_bus=i2c)
# Set the PWM frequency
pca1.frequency = 1000
pca2.frequency = 1000
pcas = [pca1, pca2]

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Arguments
parser = argparse.ArgumentParser()
parser.add_argument('-v', '--verbose', dest='verbose', action="store_true", default=False, help="Enable verbose logging")
parser.add_argument('-d', '--debug', dest='debug', action="store_true", default=False, help="Enable debug logs")
parser.add_argument('-ds', '--dimmer-speed', dest='dimmer', action="store", default=0.4, type=float, help="LED Dimmer speed")
parser.add_argument('-ldc', '--low-duty-cycle', dest='low_duty_cycle', action="store", default=0, type=int, help="Low Duty Cycle")
parser.add_argument('-hdc', '--high-duty-cycle', dest='high_duty_cycle', action="store", default=10000, type=int, help="Low Duty Cycle")
parser.add_argument('-fs', '--failover-seconds', dest='failover_seconds', action="store", default=300, type=int, help="Time to trigger failover state")
parser.add_argument('-db', '--decibel-threshold', dest='decibel_threshold', action="store", default=-10, type=int, help="The decibel threshold to gate the audio")
parser.add_argument('-tl', '--trigger-length', dest='trigger_length', action="store", default=5, type=int, help="How many segments before we determine it is a legitimate tone")
parser.add_argument('-ss', '--startup-sequence', dest='startup_sequence', action="store_true", default=False, help="Enable/disable startup sequence")

args = parser.parse_args()

# Enable/Disable startup sequence
STARTUP_SEQUENCE=args.startup_sequence
# Decibel gate threshold
DECIBEL_THRESHOLD=args.decibel_threshold
# Time to trigger failover state
FAILOVER_SECONDS=args.failover_seconds
# How many 46ms segments before we determine it is a legitimate tone
triggerlength=args.trigger_length
# How many false 46ms blips before we declare the alarm is not ringing
resetlength=10
# Enable debug output
debug=args.debug
# Enable verbose output
verbose=args.verbose
# Low Duty Cycle
LOW_DUTY_CYCLE=args.low_duty_cycle
# High Duty Cycle
HIGH_DUTY_CYCLE=args.high_duty_cycle
# The frequency in which the channels begin (Hz)
CHANNEL_START=1100
# The number of channels connected
CHANNEL_COUNT=32
# The size of each channel frequency block (Hz)
CHANNEL_SIZE=100
HALF_CHANNEL=CHANNEL_SIZE/2
# The frequency block for the "All On" command
ALL_ON_FREQ=800
# The "All Off"( frequency/2) range will be the next block of frequencies after the "All On" block
ALL_OFF_FREQ=ALL_ON_FREQ+HALF_CHANNEL
# The frequency blocks for the "Special groups"
SPECIAL_GROUP_FREQ=5000
# Left Group
LEFT_ON_FREQ=600
LEFT_OFF_FREQ=600+HALF_CHANNEL
# Right Group
RIGHT_ON_FREQ=700
RIGHT_OFF_FREQ=700+HALF_CHANNEL
# (LED/2) Dim speed
LED_DIM_SPEED_SECONDS=args.dimmer
LED_DIM_SPEED=round((0.01133/LED_DIM_SPEED_SECONDS)*15000)
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


channelhitcounts=[]
resetcounts=[]
# ALL ON, ALL OFF, LEFT ON, LEFT OFF, RIGHT ON, RIGHT OFF, SPECIAL GROUP ON
allhitcounts=[0,0,0,0,0,0,0]
allresetcounts=[0,0,0,0,0,0,0]
groupmodecount=0
groupmodereset=0

max_freq=0
channel_start_freqs=[]
channel_end_freqs=[]
status=[]
laststatus=[]

# Special Groups
ALL_CHANNELS=[i for i in range(0, CHANNEL_COUNT)]
LEFT_CHANNELS=[i for i in range(0,13)]
RIGHT_CHANNELS=[i for i in range(13,26)]
BLUE_ANGELS_CHANNELS=[i for i in range(0,6)]
SKYHAWK_CHANNELS=[i for i in range(9,11)]
NAVIHAWK_CHANNELS=[i for i in range(12,13)]
AQUALAND_CHANNELS=[i for i in range(13,16)]
SAILHAWK_CHANNELS=[i for i in range(24,26)]
PROMASTER_CHANNELS=[i for i in range(22,24)]

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
    max_freq=channel_end_freq
    if verbose: print('Channel %s: Range %sHz - %sHz' % (led, channel_start_freq, channel_end_freq))


def cie1931(L):
    L = L*100.0
    if L <= 8:
        return (L/903.3)
    else:
        return ((L+16.0)/119.0)**3

INPUT_SIZE=50*LED_DIM_SPEED_SECONDS
OUTPUT_SIZE=19999
m16 = lambda x: struct.unpack('H', struct.pack('H', x))[0]
x = range(LOW_DUTY_CYCLE,int(INPUT_SIZE+1))
dim_range = [m16(round(cie1931(float(L)/INPUT_SIZE)*OUTPUT_SIZE)) for L in x]

def get_failover_seconds(ft):
    failover_diff = datetime.combine(datetime.now(), ft) - datetime.combine(datetime.now(), datetime.now().time())
    return abs(failover_diff.total_seconds())

def can_trigger(hit_count):
    return hit_count >= triggerlength

def check_statuses(st, flag=True):  
    chk = True
    for item in st: 
        if item != flag: 
            chk = False
            break; 
              
    return chk

def is_in_range(freq, start, end):
    return freq >= start and freq < end

def determine_channel_num(fq):
    channel=0
    for i in range(0,CHANNEL_COUNT):
        channel_start_freq = channel_start_freqs[i]
        channel_end_freq = channel_end_freqs[i]
        if fq >= channel_start_freq and fq <= channel_end_freq:
            channel=i+1
    return channel

def get_pca(channel):
    return pcas[math.trunc(channel/16)]

def get_channel(channel):
    pca = get_pca(channel)
    pca_channel = channel if channel < 16 else channel - 16
    return pca.channels[pca_channel]

def rms(data):
    SHORT_NORMALIZE = (1.0/32768.0)
    swidth = 2
    frame = data
    count = len(frame) / swidth
    format = "%dh" % (count)
    shorts = struct.unpack(format, frame)

    sum_squares = 0.0
    for sample in shorts:
        n = sample * SHORT_NORMALIZE
        sum_squares += n * n
    rms = math.pow(sum_squares / count, 0.5)

    return rms * 1000

def get_freq():
    while _stream.get_read_available()< NUM_SAMPLES: sleep(0.01)
    data = _stream.read(_stream.get_read_available())
    try:
        r = rms(data)
        decibel = 20 * log10(r/20)
        # if debug and verbose: print('%sdB' % str(round(decibel, 1)))
        if (decibel < DECIBEL_THRESHOLD):
            return 0

        audio_data  = frombuffer(data, dtype=short)[-NUM_SAMPLES:]
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
    except:
        print('Error', sys.exc_info()[0])

def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

sleep_time = 0.0005
def set_all(dc, channels):
    
    if (0 <= CHANNEL_COUNT and 0 in channels):
        get_channel(0).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (1 <= CHANNEL_COUNT and 1 in channels):
        get_channel(1).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (2 <= CHANNEL_COUNT and 2 in channels):
        get_channel(2).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (3 <= CHANNEL_COUNT and 3 in channels):
        get_channel(3).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (4 <= CHANNEL_COUNT and 4 in channels):
        get_channel(4).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (5 <= CHANNEL_COUNT and 5 in channels):
        get_channel(5).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (6 <= CHANNEL_COUNT and 6 in channels):
        get_channel(6).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (7 <= CHANNEL_COUNT and 7 in channels):
        get_channel(7).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (8 <= CHANNEL_COUNT and 8 in channels):
        get_channel(8).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (9 <= CHANNEL_COUNT and 9 in channels):
        get_channel(9).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (10 <= CHANNEL_COUNT and 10 in channels):
        get_channel(10).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (11 <= CHANNEL_COUNT and 11 in channels):
        get_channel(11).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (12 <= CHANNEL_COUNT and 12 in channels):
        get_channel(12).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (13 <= CHANNEL_COUNT and 13 in channels):
        get_channel(13).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (14 <= CHANNEL_COUNT and 14 in channels):
        get_channel(14).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (15 <= CHANNEL_COUNT and 15 in channels):
        get_channel(15).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (16 <= CHANNEL_COUNT and 16 in channels):
        get_channel(0+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (17 <= CHANNEL_COUNT and 17 in channels):
        get_channel(1+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (18 <= CHANNEL_COUNT and 18 in channels):
        get_channel(2+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (19 <= CHANNEL_COUNT and 19 in channels):
        get_channel(3+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (20 <= CHANNEL_COUNT and 20 in channels):
        get_channel(4+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (21 <= CHANNEL_COUNT and 21 in channels):
        get_channel(5+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (22 <= CHANNEL_COUNT and 22 in channels):
        get_channel(6+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (23 <= CHANNEL_COUNT and 23 in channels):
        get_channel(7+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (24 <= CHANNEL_COUNT and 24 in channels):
        get_channel(8+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (25 <= CHANNEL_COUNT and 25 in channels):
        get_channel(9+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (26 <= CHANNEL_COUNT and 26 in channels):
        get_channel(10+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (27 <= CHANNEL_COUNT and 27 in channels):
        get_channel(11+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (28 <= CHANNEL_COUNT and 28 in channels):
        get_channel(12+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (29 <= CHANNEL_COUNT and 29 in channels):
        get_channel(13+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (30 <= CHANNEL_COUNT and 30 in channels):
        get_channel(14+16).duty_cycle = dc
    else:
        sleep(sleep_time)
    if (31 <= CHANNEL_COUNT and 31 in channels):
        get_channel(15+16).duty_cycle = dc
    else:
        sleep(sleep_time)

def set_all_command(i, on_channels, off_channels):
    dim_len = len(dim_range) - 1
    dc = dim_range[i]
    reversed_dc = dim_range[dim_len - i]
    if (0 <= CHANNEL_COUNT and 0 in on_channels):
        get_channel(0).duty_cycle = dc
    elif (0 <= CHANNEL_COUNT and 0 in off_channels):
        get_channel(0).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (1 <= CHANNEL_COUNT and 1 in on_channels):
        get_channel(1).duty_cycle = dc
    elif (1 <= CHANNEL_COUNT and 1 in off_channels):
        get_channel(1).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (2 <= CHANNEL_COUNT and 2 in on_channels):
        get_channel(2).duty_cycle = dc
    elif (2 <= CHANNEL_COUNT and 2 in off_channels):
        get_channel(2).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (3 <= CHANNEL_COUNT and 3 in on_channels):
        get_channel(3).duty_cycle = dc
    elif (3 <= CHANNEL_COUNT and 3 in off_channels):
        get_channel(3).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (4 <= CHANNEL_COUNT and 4 in on_channels):
        get_channel(4).duty_cycle = dc
    elif (4 <= CHANNEL_COUNT and 4 in off_channels):
        get_channel(4).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (5 <= CHANNEL_COUNT and 5 in on_channels):
        get_channel(5).duty_cycle = dc
    elif (5 <= CHANNEL_COUNT and 5 in off_channels):
        get_channel(5).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (6 <= CHANNEL_COUNT and 6 in on_channels):
        get_channel(6).duty_cycle = dc
    elif (6 <= CHANNEL_COUNT and 6 in off_channels):
        get_channel(6).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (7 <= CHANNEL_COUNT and 7 in on_channels):
        get_channel(7).duty_cycle = dc
    elif (7 <= CHANNEL_COUNT and 7 in off_channels):
        get_channel(7).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (8 <= CHANNEL_COUNT and 8 in on_channels):
        get_channel(8).duty_cycle = dc
    elif (8 <= CHANNEL_COUNT and 8 in off_channels):
        get_channel(8).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (9 <= CHANNEL_COUNT and 9 in on_channels):
        get_channel(9).duty_cycle = dc
    elif (9 <= CHANNEL_COUNT and 9 in off_channels):
        get_channel(9).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (10 <= CHANNEL_COUNT and 10 in on_channels):
        get_channel(10).duty_cycle = dc
    elif (10 <= CHANNEL_COUNT and 10 in off_channels):
        get_channel(10).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (11 <= CHANNEL_COUNT and 11 in on_channels):
        get_channel(11).duty_cycle = dc
    elif (11 <= CHANNEL_COUNT and 11 in off_channels):
        get_channel(11).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (12 <= CHANNEL_COUNT and 12 in on_channels):
        get_channel(12).duty_cycle = dc
    elif (12 <= CHANNEL_COUNT and 12 in off_channels):
        get_channel(12).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (13 <= CHANNEL_COUNT and 13 in on_channels):
        get_channel(13).duty_cycle = dc
    elif (13 <= CHANNEL_COUNT and 13 in off_channels):
        get_channel(13).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (14 <= CHANNEL_COUNT and 14 in on_channels):
        get_channel(14).duty_cycle = dc
    elif (14 <= CHANNEL_COUNT and 14 in off_channels):
        get_channel(14).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (15 <= CHANNEL_COUNT and 15 in on_channels):
        get_channel(15).duty_cycle = dc
    elif (15 <= CHANNEL_COUNT and 15 in off_channels):
        get_channel(15).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (16 <= CHANNEL_COUNT and 16 in on_channels):
        get_channel(0+16).duty_cycle = dc
    elif (16 <= CHANNEL_COUNT and 16 in off_channels):
        get_channel(0+16).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (17 <= CHANNEL_COUNT and 17 in on_channels):
        get_channel(1+16).duty_cycle = dc
    elif (17 <= CHANNEL_COUNT and 17 in off_channels):
        get_channel(1+16).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (18 <= CHANNEL_COUNT and 18 in on_channels):
        get_channel(2+16).duty_cycle = dc
    elif (18 <= CHANNEL_COUNT and 18 in off_channels):
        get_channel(2+16).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (19 <= CHANNEL_COUNT and 19 in on_channels):
        get_channel(3+16).duty_cycle = dc
    elif (19 <= CHANNEL_COUNT and 19 in off_channels):
        get_channel(3+16).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (20 <= CHANNEL_COUNT and 20 in on_channels):
        get_channel(4+16).duty_cycle = dc
    elif (20 <= CHANNEL_COUNT and 20 in off_channels):
        get_channel(4+16).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (21 <= CHANNEL_COUNT and 21 in on_channels):
        get_channel(5+16).duty_cycle = dc
    elif (21 <= CHANNEL_COUNT and 21 in off_channels):
        get_channel(5+16).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (22 <= CHANNEL_COUNT and 22 in on_channels):
        get_channel(6+16).duty_cycle = dc
    elif (22 <= CHANNEL_COUNT and 22 in off_channels):
        get_channel(6+16).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (23 <= CHANNEL_COUNT and 23 in on_channels):
        get_channel(7+16).duty_cycle = dc
    elif (23 <= CHANNEL_COUNT and 23 in off_channels):
        get_channel(7+16).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (24 <= CHANNEL_COUNT and 24 in on_channels):
        get_channel(8+16).duty_cycle = dc
    elif (24 <= CHANNEL_COUNT and 24 in off_channels):
        get_channel(8+16).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (25 <= CHANNEL_COUNT and 25 in on_channels):
        get_channel(9+16).duty_cycle = dc
    elif (25 <= CHANNEL_COUNT and 25 in off_channels):
        get_channel(9+16).duty_cycle = reversed_dc
    else:
        sleep(sleep_time)
    
    if (26 <= CHANNEL_COUNT and 26 in on_channels):
        get_channel(10+16).duty_cycle = dc
    elif (26 <= CHANNEL_COUNT and 26 in off_channels):
        get_channel(10+16).duty_cycle =dim_range[len(dim_range) -  dc]
    else:
        sleep(sleep_time)
    
    if (27 <= CHANNEL_COUNT and 27 in on_channels):
        get_channel(11+16).duty_cycle = dc
    elif (27 <= CHANNEL_COUNT and 27 in off_channels):
        get_channel(11+16).duty_cycle =dim_range[len(dim_range) -  dc]
    else:
        sleep(sleep_time)
    
    if (28 <= CHANNEL_COUNT and 28 in on_channels):
        get_channel(12+16).duty_cycle = dc
    elif (28 <= CHANNEL_COUNT and 28 in off_channels):
        get_channel(12+16).duty_cycle =dim_range[len(dim_range) -  dc]
    else:
        sleep(sleep_time)
    
    if (29 <= CHANNEL_COUNT and 29 in on_channels):
        get_channel(13+16).duty_cycle = dc
    elif (29 <= CHANNEL_COUNT and 29 in off_channels):
        get_channel(13+16).duty_cycle =dim_range[len(dim_range) -  dc]
    else:
        sleep(sleep_time)
    
    if (30 <= CHANNEL_COUNT and 30 in on_channels):
        get_channel(14+16).duty_cycle = dc
    elif (30 <= CHANNEL_COUNT and 30 in off_channels):
        get_channel(14+16).duty_cycle =dim_range[len(dim_range) -  dc]
    else:
        sleep(sleep_time)
    
    if (31 <= CHANNEL_COUNT and 31 in on_channels):
        get_channel(15+16).duty_cycle = dc
    elif (31 <= CHANNEL_COUNT and 31 in off_channels):
        get_channel(15+16).duty_cycle =dim_range[len(dim_range) -  dc]
    else:
        sleep(sleep_time)
    
def turn_on_led(led):
    if debug: print('turn_on_led %s' % (led+1))
    start = time.time()
    channel = get_channel(led)
    for i in dim_range:
        channel.duty_cycle = i
        sleep(0.02*LED_DIM_SPEED_SECONDS)
    if verbose and debug: print('LED %s: ON - %s seconds' % (led, str(round(time.time() - start, 2))))

def turn_off_led(led):
    if debug: print('turn_off_led %s' % (led+1))
    start = time.time()
    channel = get_channel(led)
    for i in reversed(dim_range):
        channel.duty_cycle = i
        sleep(0.02*LED_DIM_SPEED_SECONDS)
    # channel.duty_cycle = 0
    if verbose and debug: print('LED %s: OFF - %s seconds' % (led, str(round(time.time() - start, 2))))

def all_on(affected_channels=[]):
    start = time.time()

    for i in dim_range:
        set_all(i, affected_channels)
    if verbose and debug: print('total: ', str(round(time.time() - start, 2)))

def all_off(affected_channels=[]):
    start = time.time()
    for i in reversed(dim_range):
        set_all(i, affected_channels)
    if verbose and debug: print('total: ', str(round(time.time() - start, 2)))

def command_all(on_channels=[], off_channels=[]):
    print(on_channels)
    print(off_channels)
    for i in range(len(dim_range)):
        set_all_command(i, on_channels, off_channels)

def choose_special_group(freq):
    special_dict = {
       0: BLUE_ANGELS_CHANNELS,
       1: SKYHAWK_CHANNELS,
       2: NAVIHAWK_CHANNELS,
       3: AQUALAND_CHANNELS,
       4: SAILHAWK_CHANNELS,
       5: PROMASTER_CHANNELS, 
    }
    s = math.trunc(math.floor((freq - 5000) / 100))
    return special_dict[s]

def startup_sequence():
    print('Running startup sequence...')
    print('All On')
    all_on(ALL_CHANNELS)
    print('All Off')
    all_off(ALL_CHANNELS)
    print('Left On')
    all_on(LEFT_CHANNELS)
    print('Right On')
    all_on(RIGHT_CHANNELS)
    print('All Off')
    all_off(ALL_CHANNELS)
    print('Startup sequence complete!')

if STARTUP_SEQUENCE: startup_sequence()


failover_time = datetime.now().time()
group_mode=False
while True:
    try:
        frequency = round(get_freq())

        if debug and frequency > 10: print('%s Hz' % frequency)
            
        # If frequency is within the LED single channel range
        if is_in_range(frequency, CHANNEL_START, max_freq):
            channel=determine_channel_num(frequency)
            channel_index=channel-1
            
            if channel:
                channelhitcounts[channel_index]+=1
                resetcounts[channel_index]=0
                if (can_trigger(channelhitcounts[channel_index])):
                    failover_time = datetime.now().time()
                    channelhitcounts[channel_index]=0
                    resetcounts[channel_index]=0
                    if verbose: print('\nLED %s\n' % (channel))
                    
                    # Set the status to ON if the frequency is in the first have of the channel range. Otherwise, turn it off
                    status[channel_index]=frequency < channel_start_freqs[channel_index] + HALF_CHANNEL
            else:
                for i in ALL_CHANNELS:
                    channelhitcounts[i]=0
                    resetcounts[i]+=1
                    if (resetcounts[i]>=resetlength): resetcounts[i]=0
                
        else:
            for i in ALL_CHANNELS:
                channelhitcounts[i]=0
                resetcounts[i]+=1
                if (resetcounts[i]>=resetlength): resetcounts[i]=0
        
        
        # All On
        if is_in_range(frequency, ALL_ON_FREQ, ALL_ON_FREQ + HALF_CHANNEL) and not group_mode:
            allhitcounts[0]+=1
            allresetcounts[0]=0
            if (can_trigger(allhitcounts[0])):
                failover_time = datetime.now().time()
                allhitcounts[0]=0
                allresetcounts[0]=0

                already_on = check_statuses(status)
                affected_channels = []
                if not already_on:
                    for i in ALL_CHANNELS:
                        if verbose: print('Channel %s: %s' % (i + 1, 'ON'))
                        # Only turn ON lights if they are currently OFF
                        if not status[i]: affected_channels.append(i)
                        laststatus[i] = True
                        status[i] = True
                    if verbose: print('---------------------')
                    all_on(affected_channels)
        
        # All Off
        elif is_in_range(frequency, ALL_OFF_FREQ, ALL_OFF_FREQ + HALF_CHANNEL) and not group_mode:
            allhitcounts[1]+=1
            allresetcounts[1]=0
            if (can_trigger(allhitcounts[1])):
                failover_time = datetime.now().time()
                allhitcounts[1]=0
                allresetcounts[1]=0

                already_off = check_statuses(status, False)
                affected_channels = []
                if not already_off:
                    for i in ALL_CHANNELS:
                        if verbose: print('Channel %s: %s' % (i + 1, 'OFF'))
                        # Only turn OFF lights if they are currently ON
                        if status[i]: affected_channels.append(i)
                        laststatus[i] = False
                        status[i] = False
                    if verbose: print('---------------------')
                    all_off(affected_channels)

        # Special Group On
        elif is_in_range(frequency, SPECIAL_GROUP_FREQ, SPECIAL_GROUP_FREQ + 6000):
            allhitcounts[6]+=1
            allresetcounts[6]=0
            if (can_trigger(allhitcounts[6])):
                failover_time = datetime.now().time()
                allhitcounts[6]=0
                allresetcounts[6]=0
                sg = choose_special_group(frequency)
                sg_status = [status[i] for i in sg]
                already_on = check_statuses(sg_status)
                affected_channels = []
                if not already_on:
                    for i in sg:
                        if verbose: print('Special Channel %s: %s' % (i + 1, 'ON'))
                        # Only turn ON lights if they are currently OFF
                        if not status[i]: affected_channels.append(i)
                        status[i] = True
                        if not group_mode: laststatus[i] = status[i]
                    if verbose: print('---------------------')
                    if not group_mode: all_on(affected_channels)
        
        # Left On
        elif is_in_range(frequency, LEFT_ON_FREQ, LEFT_ON_FREQ + HALF_CHANNEL):
            allhitcounts[2]+=1
            allresetcounts[2]=0
            if (can_trigger(allhitcounts[2])):
                failover_time = datetime.now().time()
                allhitcounts[2]=0
                allresetcounts[2]=0
                sg_status = [status[i] for i in LEFT_CHANNELS]
                already_on = check_statuses(sg_status)
                affected_channels = []
                if not already_on:
                    for i in LEFT_CHANNELS:
                        if verbose: print('Channel %s: %s' % (i + 1, 'ON'))
                        # Only turn ON lights if they are currently OFF
                        if not status[i]: affected_channels.append(i)
                        status[i] = True
                        if not group_mode: laststatus[i] = status[i]
                    if verbose: print('---------------------')
                    if not group_mode: all_on(affected_channels)
        # Left Off
        elif is_in_range(frequency, LEFT_OFF_FREQ, LEFT_OFF_FREQ + HALF_CHANNEL):
            allhitcounts[3]+=1
            allresetcounts[3]=0
            if (can_trigger(allhitcounts[3])):
                failover_time = datetime.now().time()
                allhitcounts[3]=0
                allresetcounts[3]=0
                sg_status = [status[i] for i in LEFT_CHANNELS]
                already_off = check_statuses(sg_status, False)
                affected_channels = []
                if not already_off:
                    for i in LEFT_CHANNELS:
                        if verbose: print('Channel %s: %s' % (i + 1, 'ON'))
                        # Only turn ON lights if they are currently OFF
                        if not status[i]: affected_channels.append(i)
                        status[i] = False
                        if not group_mode: laststatus[i] = status[i]
                    if verbose: print('---------------------')
                    if not group_mode: all_off(affected_channels)

        # Right On
        elif is_in_range(frequency, RIGHT_ON_FREQ, RIGHT_ON_FREQ + HALF_CHANNEL):
            allhitcounts[4]+=1
            allresetcounts[4]=0
            if (can_trigger(allhitcounts[4])):
                failover_time = datetime.now().time()
                allhitcounts[4]=0
                allresetcounts[4]=0
                sg_status = [status[i] for i in RIGHT_CHANNELS]
                already_on = check_statuses(sg_status)
                affected_channels = []
                if not already_on:
                    for i in RIGHT_CHANNELS:
                        if verbose: print('Channel %s: %s' % (i + 1, 'ON'))
                        # Only turn ON lights if they are currently OFF
                        if not status[i]: affected_channels.append(i)
                        status[i] = True
                        if not group_mode: laststatus[i] = status[i]
                    if verbose: print('---------------------')
                    if not group_mode: all_on(affected_channels)
        # Right Off
        elif is_in_range(frequency, RIGHT_OFF_FREQ, RIGHT_OFF_FREQ + HALF_CHANNEL):
            allhitcounts[5]+=1
            allresetcounts[5]=0
            if (can_trigger(allhitcounts[5])):
                failover_time = datetime.now().time()
                allhitcounts[5]=0
                allresetcounts[5]=0
                sg_status = [status[i] for i in RIGHT_CHANNELS]
                already_off = check_statuses(sg_status, False)
                affected_channels = []
                if not already_off:
                    for i in RIGHT_CHANNELS:
                        if verbose: print('Channel %s: %s' % (i + 1, 'ON'))
                        # Only turn ON lights if they are currently OFF
                        if not status[i]: affected_channels.append(i)
                        status[i] = False
                        if not group_mode: laststatus[i] = status[i]
                    if verbose: print('---------------------')
                    if not group_mode: all_off(affected_channels)
        
        else:
            for i in range(len(allhitcounts)):
                allhitcounts[i]=0
                allresetcounts[i]+=1
                if (allresetcounts[i]>=resetlength): allresetcounts[i]=0

        # If the frequency is greater than the two all on/off channels
        if frequency > ALL_OFF_FREQ + HALF_CHANNEL and not group_mode:
            # Update the LED statuses if they have changed
            if laststatus != status:
                for i in ALL_CHANNELS:
                    if verbose: print('Channel %s: %s' % (i + 1, 'ON' if status[i] else 'OFF'))
                    if laststatus[i] != status[i]:
                        if status[i]:
                            turn_on_led(i)
                        else:
                            turn_off_led(i)
                        laststatus[i] = status[i]
                if verbose: print('---------------------')
        
        # If frequency is in GROUP MODE
        if is_in_range(frequency, 500, 600):
            groupmodecount+=1
            if (can_trigger(groupmodecount)):
                if group_mode and verbose and groupmodecount == triggerlength + 1: print('GROUP MODE ON')
                # If in the "End Group Mode" range and the system is already in group mode
                if frequency > 550 and group_mode:
                    on_channels = []
                    off_channels = []
                    print(laststatus)
                    print(status)

                    for i in ALL_CHANNELS:
                        if verbose: print('Group Channel %s: %s' % (i + 1, 'ON' if status[i] else 'OFF'))
                        if laststatus[i] != status[i]:
                            if status[i]:
                                on_channels.append(i)
                            else:
                                off_channels.append(i)
                            laststatus[i] = status[i]
                    group_mode = False
                    command_all(on_channels, off_channels)
                    if verbose: print('GROUP MODE OFF')
                group_mode = frequency < 550
        # If not in group mode range
        else:
            groupmodecount=0
            groupmodereset+=1
            if (groupmodereset>=resetlength): groupmodereset=0

        # If failover count is in the failover threshold
        failover_diff = get_failover_seconds(failover_time)
        if (failover_diff > FAILOVER_SECONDS):
            print('FAILOVER')
            already_on = check_statuses(status)
            affected_channels = []
            if not already_on:
                for i in ALL_CHANNELS:
                    if verbose: print('Channel %s: %s' % (i + 1, 'ON'))
                    # Only turn ON lights if they are currently OFF
                    if not status[i]: affected_channels.append(i)
                    laststatus[i] = True
                    status[i] = True
                all_on(affected_channels)


    except KeyboardInterrupt:
        print('Interrupted')
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)
    except:
        print('Error', sys.exc_info()[0])            
      
