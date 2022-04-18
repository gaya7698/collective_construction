import os
from posixpath import split
import socket
import sys
import subprocess
import time
import re
import numpy as np

          

'''s = socket.socket()  
port = 8080

s.bind(('', port))        
print ("socket binded to %s" %(port))
 
# put the socket into listening mode
s.listen(5)    
print ("socket is listening")'''   

## Run environment - wont continue script after initializing sim - no more input/output
#os.system('argos3 -c experiments/diffusion_1.argos') 

## test path input/outputs
path=[ [1 ,0, 1, 1, 0 ], [1, 1, 0, 1, 0] ]

## Get State Space
file1 = open(r"./experiments/diffusion_1.argos","r+") 
file1.seek(0) 
print("Output of Read function is ")
lines=file1.readlines()
worldSTR=[]
for i in range(len(lines)):
  if "wall" in lines[i]:
    print(lines[i+1])
    worldSTR.append(re.findall(r'(?<=on=").*?(?=" o)',lines[i+1]))
file1.close()
worldCOORD=[]
for i in range(len(worldSTR)):
  temp=[worldSTR[i][0].split(",")][0]
  print(temp)
  worldCOORD.append(list(map(float, temp)))
xcoord=[]
ycoord=[]
print(worldCOORD)
for i in worldCOORD:
  xcoord.append(i[0])
  ycoord.append(i[1])
xmin= min(xcoord)
ymin= min(ycoord)
xrange=max(xcoord)-xmin
yrange=max(ycoord)-ymin
Map=np.zeros((int(xrange), int(yrange)))

#number of robots
N=2
#for i in range(10):
while True:
  for i in range(N):
    BotName=('fb_{}' .format(i))
    file1 = open(BotName) 
    #print(file1.readlines())
    #if (file1.read()):
    #file1.seek(0)
    temp=(file1.readlines()) 
    print(temp)
    print(len(temp))
    if len(temp)>1:
      print(temp[1])
      BotPos=(list(map(float,temp[1].split(" "))))
      Botx=int(np.floor(BotPos[0]-xmin))
      Boty=int(np.floor(BotPos[1]-ymin))
      Map[Botx,Boty]=1
    file1.close()

  print(Map)
  Map=np.zeros((int(xrange), int(yrange)))
  time.sleep(0.1)

# Establish connection with client.
'''c, addr = s.accept()    
print ('Got connection from', addr )

# send a thank you message to the client. encoding to send byte type.
c.send('Thank you for connecting'.encode())
c.close()'''






