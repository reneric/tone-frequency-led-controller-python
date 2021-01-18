# Tone Frequency LED Controller


## Usage
```bash
Usage: python freq_led.py [-v] [-d] [-ds=1]

Options:
  --verbose, -v          Enable verbose logging               [boolean]
  --debug, -d            Enable debug logs                    [boolean]
  --dimmer-speed, -ds    Enable debug logs                    [integer]
  --help                 Show help    
```

## Raspberry Pi OS
Download and install [Raspberry Pi Imager](https://www.raspberrypi.org/software/) to a computer with an SD card reader.

### Initial Configuration
Insert the SD card back into your computer. There are three text files we will create/edit in boot. [Adafruit instructions](https://learn.adafruit.com/raspberry-pi-zero-creation/text-file-editing)

1. wpa_supplicant.conf - wifi settings
2. config.txt - global system settings
3. ssh - an empty text file to enable ssh
#### Wifi Configuration
If using WiFi, edit wpa_supplicant.conf and add the network
```bash
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=US
 
network={
    ssid="YOURSSID"
    psk="YOURPASSWORD"
    scan_ssid=1
}
```
### Enable UART
Enable UART on the GPIO header pins. If `config.txt` already exists, add the following to the end. If it does not exist, you will have to create the file
```bash
# Enable UART
enable_uart=1
```
### Enable SSH
Create an empty file named `ssh`
```bash
  $ touch ssh
```


# Setup
### Enable I2C Interface
Run the following command to launch the raspi-config utility. Select “Interfacing Options”:
```bash
  $ sudo raspi-config
```
1. Highlight the “I2C” option and activate “\<Select\>”
2. Select and activate “\<Yes\>” 
3. Highlight and activate “\<Ok\>” 
4. When prompted to reboot highlight and activate “\<Yes\>” 
5. The Raspberry Pi will reboot and the interface will be enabled.

#### Configure I2C
```bash
  $ sudo apt-get install -y python-smbus
  $ sudo apt-get install -y i2c-tools
```
#### Verify I2C is enabled
```bash
  $ ls /dev/i2c* /dev/spi*
```

### Libraries
Listed below are all of the libraries and configurations needed to get the RPI setup to run the program.
```bash
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
```
## Project code
Clone the project into the `/home/pi` directory
```bash
  $ git clone https://github.com/reneric/tone-frequency-led-controller-python.git
```
## Run at startup
In order for the program to run automatically at boot, we'll need modify the .bashrc file.
Put your command at the bottom of ‘/home/pi/.bashrc’:
```bash
  $ sudo nano /home/pi/.bashrc
```
Go to the last line of the script and add:
```bash
  $ python /home/pi/tone-frequency-led-controller-python/freq_led.py &
```
