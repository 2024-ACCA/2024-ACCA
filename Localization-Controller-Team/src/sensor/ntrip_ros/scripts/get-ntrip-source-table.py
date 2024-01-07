#################################
# Get SOURCETABLE from an NTRIP serve
#
# Example:
#
# GET / HTTP/1.0
# Host: ok.smartnetna.com:10000
# User-Agent: MyAppName
# Accept: */*
#
#
# SOURCETABLE 200 OK
# Server: GNSS Spider 7.2.1.7804/1.0
# Date: Wed, 05 Dec 2018 20:55:04 GMT Standard Time
# Content-Type: text/plain
# Content-Length: 1101
#
# STR;RTCM3_IMAX;RTCM3_IMAX;RTCM 3;;2;GPS & GLO;SmartNet North America;;41.26;-96.14;1;0;Leica GNSS Spider;none;B;Y;9600;
# STR;RTCM3_ViRS;RTCM3_ViRS;RTCM 3;;2;GPS & GLO;SmartNet North America;;41.26;-96.14;1;1;Leica GNSS Spider;none;B;Y;9600;
# STR;RTCM3_NEAR;RTCM3_NEAR;RTCM 3;;2;GPS & GLO;SmartNet North America;;41.26;-96.14;1;0;Leica GNSS Spider;none;B;Y;9600;
# STR;RTCM3_MAX;RTCM3_MAX;RTCM 3;;2;GPS & GLO;SmartNet North America;;41.26;-96.14;1;1;Leica GNSS Spider;none;B;Y;9600;
# STR;RTCM2_DGPS_IMAX;RTCM2_DGPS_IMAX;RTCM 2;;2;GPS;SmartNet North America;;41.26;-96.14;1;0;Leica GNSS Spider;none;B;Y;9600;
# STR;RTCM2_DGPS_NEAR;RTCM2_DGPS_NEAR;RTCM 2;;2;GPS;SmartNet North America;;41.26;-96.14;1;0;Leica GNSS Spider;none;B;Y;9600;
# STR;MSM_NEAR;MSM_NEAR;RTCM 3;;2;GPS+GLO+GAL+BDS;SmartNet North America;;41.26;-96.14;1;0;Leica GNSS Spider;none;B;Y;9600;
# STR;MSM_VIRS;MSM_VIRS;RTCM 3;;2;GPS+GLO+GAL+BDS;SmartNet North America;;41.26;-96.14;1;1;Leica GNSS Spider;none;B;Y;9600;
# STR;MSM_IMAX;MSM_IMAX;RTCM 3;;2;GPS+GLO+GAL+BDS;SmartNet North America;;41.26;-96.14;1;0;Leica GNSS Spider;none;B;Y;9600;
# ENDSOURCETABLE
#################################
import socket

UserAgent = 'MyAppName'

# ok.smartnetna.com is commercial
# https://www.smartnetna.com/
Server = 'fkp.ngii.go.kr'
Port = 2201
Station = 'VRS_V32'

Server = 'gnssdata.or.kr'
Port = 2101
Station = 'SOUL-RTCM23'

# rtgpsout.unavco.org is free but requires registration to us
# SOURCETABLE returns hundreds of NTRIP mount points
# http://www.unavco.org/instrumentation/networks/status/all/realtime
# Server = 'rtgpsout.unavco.org'
# Port = 2101
# Station = 'WMOK_RTCM3'

print("******************")

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = (Server, Port)
client_socket.connect(server_address)

request_header = ('GET / HTTP/1.0\r\n' +
                'Host: %s:%s\r\n' +
                'User-Agent: %s\r\n' +
                'Accept: */*\r\n' +
                '\r\n') % (Server, Port, UserAgent)
print(request_header)
client_socket.send(request_header)

response = ''
while True:
    recv = client_socket.recv(1024)
    if not recv:
        break
    response += recv 

print(response)
client_socket.close()    