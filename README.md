# Tone Frequency LED Controller

```
  $ sudo apt-get update
  $ sudo apt-get upgrade
  $ sudo pip3 install --upgrade setuptools
  $ sudo apt-get install -y python3 git python3-pip
  $ sudo update-alternatives --install /usr/bin/python python $(which python2) 1
  $ sudo update-alternatives --install /usr/bin/python python $(which python3) 2
  $ sudo update-alternatives --config python

  # Verify I2C
  $ ls /dev/i2c* /dev/spi*
  
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
In order for the program to run automatically at boot, we'll need to edit the file /etc/rc.local
### Run the code below to edit rc.local
```
  
  $ sudo nano /etc/rc.local
```
### Add the following line above the exit
```
  ...
  sudo python /home/pi/tone-frequency-led-controller-python/freq_led.py &
  exit 0
```