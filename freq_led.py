#!/usr/bin/env python
import pyaudio
from multiprocessing import Process, Pool
from numpy import zeros,linspace,short,fromstring,hstack,transpose,log,frombuffer
from scipy import fft
from time import sleep
import RPi.GPIO as GPIO
import board
import busio
import os
import adafruit_pca9685
import sys
import argparse
import time
i2c = busio.I2C(board.SCL, board.SDA)


pca = adafruit_pca9685.PCA9685(address=0x40, i2c_bus=i2c)
pca2 = adafruit_pca9685.PCA9685(address=0x41, i2c_bus=i2c)
# Set the PWM frequency
pca.frequency = 1000
pca2.frequency = 1000

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Arguments
parser = argparse.ArgumentParser()
parser.add_argument('-v', '--verbose', dest='verbose', action="store_true", default=False, help="Enable verbose logging")
parser.add_argument('-d', '--debug', dest='debug', action="store_true", default=False, help="Enable debug logs")
parser.add_argument('-ds', '--dimmer-speed', dest='dimmer', action="store", default=0.4, type=float, help="LED Dimmer speed")
parser.add_argument('-ldc', '--low-duty-cycle', dest='low_duty_cycle', action="store", default=0, type=int, help="Low Duty Cycle")
parser.add_argument('-hdc', '--high-duty-cycle', dest='high_duty_cycle', action="store", default=10000, type=int, help="Low Duty Cycle")

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
# Low Duty Cycle
LOW_DUTY_CYCLE=args.low_duty_cycle
print('LOW_DUTY_CYCLE', LOW_DUTY_CYCLE)
# High Duty Cycle
HIGH_DUTY_CYCLE=args.high_duty_cycle
print('HIGH_DUTY_CYCLE', HIGH_DUTY_CYCLE)
# The frequency in which the channels begin (Hz)
CHANNEL_START=1100
# The number of channels connected
CHANNEL_COUNT=32
# The size of each channel frequency block (Hz)
CHANNEL_SIZE=100
# The frequency block for the "All On" command
ALL_ON_FREQ=700
# The "All Off" frequency range will be the next block of frequencies after the "All On" block
ALL_OFF_FREQ=ALL_ON_FREQ+CHANNEL_SIZE
# LED Dim speed
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

def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

def set_all(dc, channels):
    sleep_time = 0.00005
    if (0 <= CHANNEL_COUNT and 0 in channels):
        pca.channels[0].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (1 <= CHANNEL_COUNT and 1 in channels):
        pca.channels[1].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (2 <= CHANNEL_COUNT and 2 in channels):
        pca.channels[2].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (3 <= CHANNEL_COUNT and 3 in channels):
        pca.channels[3].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (4 <= CHANNEL_COUNT and 4 in channels):
        pca.channels[4].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (5 <= CHANNEL_COUNT and 5 in channels):
        pca.channels[5].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (6 <= CHANNEL_COUNT and 6 in channels):
        pca.channels[6].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (7 <= CHANNEL_COUNT and 7 in channels):
        pca.channels[7].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (8 <= CHANNEL_COUNT and 8 in channels):
        pca.channels[8].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (9 <= CHANNEL_COUNT and 9 in channels):
        pca.channels[9].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (10 <= CHANNEL_COUNT and 10 in channels):
        pca.channels[10].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (11 <= CHANNEL_COUNT and 11 in channels):
        pca.channels[11].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (12 <= CHANNEL_COUNT and 12 in channels):
        pca.channels[12].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (13 <= CHANNEL_COUNT and 13 in channels):
        pca.channels[13].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (14 <= CHANNEL_COUNT and 14 in channels):
        pca.channels[14].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (15 <= CHANNEL_COUNT and 15 in channels):
        pca.channels[15].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (16 <= CHANNEL_COUNT and 16 in channels):
        pca2.channels[0].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (17 <= CHANNEL_COUNT and 17 in channels):
        pca2.channels[1].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (18 <= CHANNEL_COUNT and 18 in channels):
        pca2.channels[2].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (19 <= CHANNEL_COUNT and 19 in channels):
        pca2.channels[3].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (20 <= CHANNEL_COUNT and 20 in channels):
        pca2.channels[4].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (21 <= CHANNEL_COUNT and 21 in channels):
        pca2.channels[5].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (22 <= CHANNEL_COUNT and 22 in channels):
        pca2.channels[6].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (23 <= CHANNEL_COUNT and 23 in channels):
        pca2.channels[7].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (24 <= CHANNEL_COUNT and 24 in channels):
        pca2.channels[8].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (25 <= CHANNEL_COUNT and 25 in channels):
        pca2.channels[9].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (26 <= CHANNEL_COUNT and 26 in channels):
        pca2.channels[10].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (27 <= CHANNEL_COUNT and 27 in channels):
        pca2.channels[11].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (28 <= CHANNEL_COUNT and 28 in channels):
        pca2.channels[12].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (29 <= CHANNEL_COUNT and 29 in channels):
        pca2.channels[13].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (30 <= CHANNEL_COUNT and 30 in channels):
        pca2.channels[14].duty_cycle = dc
    else:
        sleep(sleep_time)
    if (31 <= CHANNEL_COUNT and 31 in channels):
        pca2.channels[15].duty_cycle = dc
    else:
        sleep(sleep_time)

def turn_on_led(led, speed=LED_DIM_SPEED):
    if debug: print('turn_on_led %s: %s' % (led, speed))
    start = time.time()
    for i in range(LOW_DUTY_CYCLE, HIGH_DUTY_CYCLE, speed):
        if led <= 15: pca.channels[led].duty_cycle = i
        if led > 15: pca2.channels[led-16].duty_cycle = i
    if debug: print('LED %s: ON - %s seconds' % (led, str(round(time.time() - start, 2))))

def turn_off_led(led, speed=LED_DIM_SPEED):
    if debug: print('turn_off_led %s: %s' % (led, speed))
    start = time.time()
    for i in reversed(range(LOW_DUTY_CYCLE, HIGH_DUTY_CYCLE, speed)):
        if led <= 15: pca.channels[led].duty_cycle = i
        if led > 15: pca2.channels[led-16].duty_cycle = i
    if led <= 15: pca.channels[led].duty_cycle = 0
    if led > 15: pca2.channels[led-16].duty_cycle = 0
    print('LED %s: OFF - %s seconds' % (led, str(round(time.time() - start, 2))))

def turn_on_led_chunks(led, chunk, speed=LED_DIM_SPEED):
    if debug: print('turn_on_led %s: %s' % (led, speed))
    start = time.time()
    for i in chunk:
        pca.channels[led].duty_cycle = i
    if debug: print(time.time() - start)

def all_on(affected_channels=[]):
    processes=[]
    start = time.time()
    if debug: print('LED_DIM_SPEED:',LED_DIM_SPEED)
    speed = LED_DIM_SPEED-CHANNEL_COUNT
    for i in range(LOW_DUTY_CYCLE, HIGH_DUTY_CYCLE, speed):
        set_all(i, affected_channels)
    if debug: print('total: ', str(round(time.time() - start, 2)))

def all_off(affected_channels=[]):
    processes=[]
    start = time.time()
    if debug: print('LED_DIM_SPEED:',LED_DIM_SPEED)
    speed = LED_DIM_SPEED-CHANNEL_COUNT
    for i in reversed(range(LOW_DUTY_CYCLE, HIGH_DUTY_CYCLE, speed)):
        set_all(i, affected_channels)
    for channel in affected_channels:
        if channel <= 15: pca.channels[channel].duty_cycle = 0
        if channel > 15: pca2.channels[channel-16].duty_cycle = 0
    if debug: print('total: ', str(round(time.time() - start, 2)))

while True:
    try:
        frequency = round(get_freq())
        if debug: print('%sHz' % frequency)

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
                    
                    # Set the status to ON if the frequency is in the first have of the channel range. Otherwise, turn it off
                    status[channel_index]=frequency < channel_start_freqs[channel_index] + (CHANNEL_SIZE/2)
            else:
                for i in range(0, CHANNEL_COUNT):
                    channelhitcounts[i]=0
                    resetcounts[i]+=1
                    if (resetcounts[i]>=resetlength): resetcounts[i]=0
                
        else:
            for i in range(0, CHANNEL_COUNT):
                channelhitcounts[i]=0
                resetcounts[i]+=1
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
                            turn_on_led(i, round(LED_DIM_SPEED/CHANNEL_COUNT))
                        else:
                            turn_off_led(i, round(LED_DIM_SPEED/CHANNEL_COUNT))
                        laststatus[i] = status[i]
                if verbose: print('---------------------')
    except KeyboardInterrupt:
        print('Interrupted')
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)
    except:
        print('Error', sys.exc_info()[0])            
      
