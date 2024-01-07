import sys
import time
import socket
import base64
import datetime

verbose = True
V2 = True

def calcultateCheckSum(stringToCheck):
   xsum_calc = 0
   for char in stringToCheck:
      xsum_calc = xsum_calc ^ ord(char)
   return "%02X" % xsum_calc

def getGGABytes():
   now = datetime.datetime.utcnow()
   ggaString= "GPGGA,%02d%02d%04.2f,3734.087,N,12702.603,E,1,12,1.0,0.0,M,0.0,M,," % \
      (now.hour,now.minute,now.second)

   checksum = calcultateCheckSum(ggaString)
   if verbose:
      print("$%s*%s\r\n" % (ggaString, checksum))
   return bytes("$%s*%s\r\n" % (ggaString, checksum),'ascii')

def getMountPointBytes(mountpoint, useragent, user):
   mountPointString = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (mountpoint, useragent, user)
   
   # if self.host or self.V2:
   #    hostString = "Host: %s:%i\r\n" % (self.caster,self.port)
   #    mountPointString += hostString
   
   if V2:
      mountPointString+="Ntrip-Version: Ntrip/2.0\r\n"
   mountPointString+="\r\n"
   
   if verbose:
      print(mountPointString)
   return mountPointString.encode()
   # return bytes(mountPointString,'ascii')

target_host = "www.gnssdata.or.kr" 
target_port = 2101  # create a socket object 
mountpoint = "SOUL-RTCM31"
useragent = "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/95.0.4638.69 Safari/537.36"
user = "tge1375@naver.com:gnss"

target_host = "fkp.ngii.go.kr" 
target_port = 2201  # create a socket object 
mountpoint = "VRS_V32"
useragent = "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/95.0.4638.69 Safari/537.36"
user = "tge1375@naver.com:gnss"

# user = base64.b64encode(bytes(user,'utf-8')).decode("utf-8")

# 전역
factor=2 # How much the sleep time increases with each failed attempt
maxReconnect=1
maxReconnectTime=1200
sleepTime=1 # So the first one is 1 second

client = None

# 클래스 변수
ssl = False
maxConnectTime = 0
found_header = False

reconnectTry=1
sleepTime=1
if maxConnectTime > 0 :
   EndConnect=datetime.timedelta(seconds=maxConnectTime)
try:
   while reconnectTry<=maxReconnect:
      found_header=False
      if verbose:
         print('Connection {0} of {1}\n'.format(reconnectTry,maxReconnect))

      client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
      if ssl:
         client = ssl.wrap_socket(client)
      
      error_indicator = client.connect_ex((target_host,target_port))
      if error_indicator == 0:
         sleepTime = 1
         connectTime=datetime.datetime.now()

         client.settimeout(10)
         # client.send(getMountPointBytes(mountpoint, useragent, user).encode())
         client.send(getMountPointBytes(mountpoint, useragent, user))

         while not found_header:
            casterResponse = client.recv(4096) #All the data
            header_lines = casterResponse.decode('utf-8').split("\r\n")
            print(header_lines)

            for line in header_lines:
               if line=="":
                  if not found_header:
                     found_header=True
                     if verbose:
                        print("End Of Header"+"\n")
               else:
                  if verbose:
                     print("Header: " + line+"\n")

            for line in header_lines:
               if line.find("SOURCETABLE")>=0:
                  sys.stderr.write("Mount point does not exist")
                  sys.exit(1)
               elif line.find("401 Unauthorized")>=0:
                  sys.stderr.write("Unauthorized request\n")
                  sys.exit(1)
               elif line.find("404 Not Found")>=0:
                  sys.stderr.write("Mount Point does not exist\n")
                  sys.exit(2)
               elif line.find("ICY 200 OK")>=0:
                  #Request was valid
                  client.sendall(getGGABytes())
               elif line.find("HTTP/1.0 200 OK")>=0:
                  #Request was valid
                  client.sendall(getGGABytes())
               elif line.find("HTTP/1.1 200 OK")>=0:
                  #Request was valid
                  client.sendall(getGGABytes())

         data = "Initial data"
         RTCM_ARR = []
         while data:
            try:
               data = client.recv(1)
               if ord(data) == 211:
                  RTCM_ARR.append(ord(data))

                  new_data = client.recv(2)
                  print(new_data, type(new_data), new_data[0], new_data[1])
                  
                  RTCM_ARR.append(new_data[0])
                  RTCM_ARR.append(new_data[1])
                  print(RTCM_ARR)
                  
                  cnt = new_data[0] * 256 + new_data[1]
                  cnt = cnt + 1
                  print("cnt : ", cnt)

                  if cnt > 255: # Just Abstract Value 
                     print("Over Msg Error")
                     RTCM_ARR = []
                     continue

                  for i in range(cnt):
                     new_data = client.recv(1)
                     RTCM_ARR.append(ord(new_data))

                  print("Final Array : ", RTCM_ARR)
                  ros_message = bytes(RTCM_ARR)
                  RTCM_ARR = []
               
               if maxConnectTime :
                  if datetime.datetime.now() > connectTime+EndConnect:
                     if verbose:
                        sys.stderr.write("Connection Time exceeded\n")
                     sys.exit(0)
            # TODO : TypeError: catching classes that do not inherit from BaseException is not allowed
            except client.timeout:
               if verbose:
                  sys.stderr.write('Connection TimedOut\n')
               data = False

            except client.error:
               if verbose:
                  sys.stderr.write('Connection Error\n')
               data = False

            except IndexError:
               RTCM_ARR = []
               pass

            except Exception as e:
               print("Unknown Error : ", e)
               pass
               

         if verbose:
            sys.stderr.write('Closing Connection\n')
         client.close()
         client = None

         if reconnectTry < maxReconnect :
            sys.stderr.write( "%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (datetime.datetime.now(), sleepTime))
            time.sleep(sleepTime)
            sleepTime *= factor

            if sleepTime > maxReconnectTime:
                  sleepTime = maxReconnectTime
         else:
            sys.exit(1)

         reconnectTry += 1
      else:
         client = None
         if verbose:
            print("Error indicator: ", error_indicator)

         if reconnectTry < maxReconnect :
            sys.stderr.write( "%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (datetime.datetime.now(), sleepTime))
            time.sleep(sleepTime)
            sleepTime *= factor
            if sleepTime > maxReconnectTime:
                  sleepTime = maxReconnectTime
         reconnectTry += 1

except KeyboardInterrupt:
   if client:
      client.close()
   sys.exit()
