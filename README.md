# Tone Frequency LED Controller
## Setup
### Enable I2C Interface
Run the following command to launch the raspi-config utility. Select “Interfacing Options”
```
  $ sudo raspi-config
```
1. Highlight the “I2C” option and activate “<Select>”.
2. Select and activate “<Yes>” :
3. Highlight and activate “<Ok>” :
4. When prompted to reboot highlight and activate “<Yes>” :
5. The Raspberry Pi will reboot and the interface will be enabled.

#### Configure I2C
```
  $ sudo apt-get install -y python-smbus
  $ sudo apt-get install -y i2c-tools
```
#### Verify I2C is enabled
```
  $ ls /dev/i2c* /dev/spi*
```

### Libraries
Listed below are all of the libraries and configurations needed to get the RPI setup to run the program.
```
  $ sudo apt-get update
  $ sudo apt-get upgrade
  $ sudo pip3 install --upgrade setuptools
  $ sudo apt-get install -y python3 git python3-pip
  $ sudo update-alternatives --install /usr/bin/python python $(which python2) 1
  $ sudo update-alternatives --install /usr/bin/python python $(which python3) 2
  $ sudo update-alternatives --config python
  
  $ pip3 install adafruit-blinka
  $ sudo apt-get install python-scipy
  $ sudo apt-get install python-opencv
  $ sudo apt-get install python-scipy
  $ sudo apt-get install python-pyaudio python3-pyaudio

  $ sudo pip3 install scipy
  $ sudo apt-get install libatlas-base-dev

  $ git clone https://github.com/reneric/tone-frequency-led-controller-python.git
```

## Run at startup
In order for the program to run automatically at boot, we'll need modify the .bashrc file.
Put your command at the bottom of ‘/home/pi/.bashrc’:
```
  $ sudo nano /home/pi/.bashrc
```
Go to the last line of the script and add:
```
  python /home/pi/tone-frequency-led-controller-python/freq_led.py &
```