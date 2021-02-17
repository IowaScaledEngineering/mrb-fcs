import time
import smbus2
from smbus2 import i2c_msg

class tlc59116:
  def __init__(self, bus, address):
    self.bus = bus
    self.address = address
    self.outputStates = [ 0 for i in range(0,16) ]
    self.outputPWM = [ 0xFF for i in range(0,16) ]
    #// Oscillator on, autoincrement on, all-call and subaddresses off
    data = [0x00, 0xE0]
    self.bus.i2c_rdwr(i2c_msg.write(self.address, data))
    time.sleep(0.001)  # Datasheet says wait 500uS after enabling oscillator, 1mS to be safe

    # Now, reset the rest of the registers    
    data = [0xE0 | 0x01, # Start at MODE2 register, set all auto-increment bits so we can initialize the whole space
      0x00, # 0x01 - MODE2
    ]

    self.bus.i2c_rdwr(i2c_msg.write(self.address, data))

    data = [0xE0 | 0x012,
      0xFF, # 0x12 - GRPPWM
      0x00, # 0x13 - GRPFREQ
      0x00, # 0x14 - LEDOUT0
      0x00, # 0x15 - LEDOUT1
      0x00, # 0x16 - LEDOUT2
      0x00, # 0x17 - LEDOUT3
      0xD2, # 0x18 - SUBADR1
      0xD4, # 0x19 - SUBADR2
      0xD8, # 0x1A - SUBADR3
      0xD0, # 0x1B - ALLCALLADR
      0xFF  # 0x1C - IREF 
      ]
      
    self.bus.i2c_rdwr(i2c_msg.write(self.address, data))
    
    self.setGlobalBrightness(255)
    

  def setOutput(self, outputNum, outputState, deferWrite=False):
    if outputNum not in range (0, 16):
      raise Exception("Output number [%d] out of range 0-15" % outputNum)
    
    outputState = outputState.lower()
    outputStateXlate = { 'off':0, 'on':3, 'pwm':2, 'gpwm':3 }
    
    if outputState not in outputStateXlate.keys():
      raise Exception("outputState must be 'on', 'off', 'pwm', or 'gpwm'")
    
    self.outputStates[outputNum] = outputStateXlate[outputState]
    
    # Send outputs
    if not deferWrite:
      self.refreshOutputs()

  def refreshOutputs(self):
    outputBytes = [0x00, 0x00, 0x00, 0x00]
    for i in range(0, 16):
      outputBytes[i/4] |= (self.outputStates[i]<<(2*(i%4)))
    
    data = [0xE0 | 0x14, # Start at LEDOUT0 register, set all auto-increment bits
      outputBytes[0], outputBytes[1], outputBytes[2], outputBytes[3]]

    print("Writing to 0x%02X - [%s]\n" % (self.address, data))
    self.bus.i2c_rdwr(i2c_msg.write(self.address, data))


  def setGlobalBrightness(self, brightPWM):
    if brightPWM not in range(0,256):
      raise Exception("brightPWM must be between 0-255")

    data = [0xE0 | 0x02, # Start at MODE2 register, set all auto-increment bits so we can initialize the whole space
      brightPWM, # 0x02 - PWM0
      brightPWM, # 0x03 - PWM1
      brightPWM, # 0x04 - PWM2
      brightPWM, # 0x05 - PWM3
      brightPWM, # 0x06 - PWM4
      brightPWM, # 0x07 - PWM5
      brightPWM, # 0x08 - PWM6
      brightPWM, # 0x09 - PWM7
      brightPWM, # 0x0A - PWM8
      brightPWM, # 0x0B - PWM9
      brightPWM, # 0x0C - PWM10
      brightPWM, # 0x0D - PWM11
      brightPWM, # 0x0E - PWM12
      brightPWM, # 0x0F - PWM13
      brightPWM, # 0x10 - PWM14
      brightPWM ] # 0x11 - PWM15

    self.bus.i2c_rdwr(i2c_msg.write(self.address, data))
    
  def setChannelBrightness(self, outputNum, pwm):
    if outputNum not in range (0, 16):
      raise Exception("Output number [%d] out of range 0-15" % outputNum)
    
    if pwm not in range(0, 256):
      raise Exception("PWM value must be 0-255")

    data = [0x02 + outputNum, pwm]
    self.bus.i2c_rdwr(i2c_msg.write(self.address, data))
    

  def setBlinking(self, dutyCycle, period):
    if dutyCycle == 100:
      # Shut off blinky
      data = [0x01, 0x00]
      self.bus.i2c_rdwr(i2c_msg.write(self.address, data))
      data = [0xE0 | 0x12, 0xFF, 0x00]
      self.bus.i2c_rdwr(i2c_msg.write(self.address, data))
      return
    
    if dutyCycle not in range(0,100):
      raise Exception("dutyCycle must be 0-100")
      
    # Period(S) = (GRPFREQ+1)/24
    # Thus, GRPFREQ = period * 24 - 1
    grpfreq = round(period * 24 - 1)
    grppwm = max(255, round(dutyCycle * 255 / 100))

    # Set blinky bit
    data = [0x01, 0x20]
    self.bus.i2c_rdwr(i2c_msg.write(self.address, data))    
    
    data = [0xE0 | 0x12, grppwm, grpfreq]
    self.bus.i2c_rdwr(i2c_msg.write(self.address, data))
