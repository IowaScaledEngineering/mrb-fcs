#!/usr/bin/python
import tlc59116
import re

class fcs:
  allSegments = set(['a','b','c','d','e','f','g'])
  characterDict = {
    ' ': set([]),
    '0': set(['a', 'b', 'c', 'd', 'e', 'f']),
    '1': set(['b','c']),
    '2': set(['a','b','g','e','d']),  
    '3': set(['a','b','c','g','d']),
    '4': set(['f','b','g','c']),  
    '5': set(['a','f','g','c','d']),
    '6': set(['a','f','g','c','d','e']),
    '7': set(['a','b','c']),
    '8': set(['a','b','c','d','e','f','g']),
    '9': set(['a','b','c','d','f','g']),
    'a': set(['a','b','c','f','g','e']), 
    'b': set(['f','g','c','e','d']),    
    'c': set(['g','e','d']),  
    'd': set(['b','g','e','d','c']),  
    'e': set(['a','f','g','e','d']),
    'f': set(['a','f','g','e'])
  }
  
  segMap = [
    {  # Segment maps for left character (0/2)
      'a': 11,
      'b': 12,
      'c': 6,
      'd': 7,
      'e': 8,
      'f': 9,
      'g': 10
    },
    {  # Segment maps for right character (1/3)
      'a': 14,
      'b': 15,
      'c': 1,
      'd': 3,
      'e': 4,
      'f': 13,
      'g': 2
    }
    
    
  ]
  
  def __init__(self, bus):
    self.bus = bus
    self.tlc = [None, None]
    self.tlc[0] = tlc59116.tlc59116(self.bus, 0x60) # Left
    self.tlc[1] = tlc59116.tlc59116(self.bus, 0x61) # Right

  def setDecimalPositions(self, dpNum):
    self.tlc[0].setOutput(0, "off", True)
    self.tlc[1].setOutput(0, "off", True)
    self.tlc[1].setOutput(5, "off", True)
    if dpNum == 0:
      self.tlc[1].setOutput(0, "on", True)
    elif dpNum == 1:
      self.tlc[1].setOutput(5, "on", True)    
    elif dpNum == 2:
      self.tlc[1].setOutput(0, "on", True)

  def setColon(self, colonOn):
    if colonOn is True:
      self.tlc[0].setOutput(5, "on")
    else:
      self.tlc[0].setOutput(5, "off")    

  def setBrightness(self, brightness):
    self.tlc[0].setGlobalBrightness(brightness)
    self.tlc[1].setGlobalBrightness(brightness)    

  def setChar(self, charNum, char):
    if charNum not in range (0, 4):
      raise Exception("Character number must be between 0-3")
    if char not in self.characterDict.keys():
      raise Exception("No segment map for character [%c]" % (char))    
    
    segments = self.characterDict[char]
    tlc = self.tlc[charNum/2]
    
    for segment in segments:
      tlc.setOutput(self.segMap[charNum%2][segment], "on", True)

    for segment in (self.allSegments - segments):
      tlc.setOutput(self.segMap[charNum%2][segment], "off", True)

    print("Refreshing [%d]\n" % (charNum/2));
    tlc.refreshOutputs()

# Stage 1 - convert string to characters and punct
# Stage 2 - convert punct and characters to segments
# Stage 3 - convert segments to outputs
  def write(self, str):
    # Formats
    # X = any char except .
    # Y = any char including .
    # {0-2}:{0-2}X
    for i in range(0,4):
      if i > len(str):
        self.setChar(i, ' ')
      else:
        self.setChar(i, str[i:i+1])    

