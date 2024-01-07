#!/usr/bin/env python3

# from nmea_msgs.msg import Sentence

from datetime import datetime
from base64 import b64encode
from threading import Thread

from http.client import HTTPConnection
from http.client import IncompleteRead

""" This is to fix the IncompleteRead error
    http://bobrochel.blogspot.com/2010/11/bad-servers-chunked-encoding-and.html"""
import http.client


usrPass = "tge1375@naver.com:gnss"
encoded_u = b64encode(usrPass.encode()).decode()

def calcultateCheckSum(stringToCheck):
    xsum_calc = 0
    for char in stringToCheck:
        xsum_calc = xsum_calc ^ ord(char)
    return "%02X" % xsum_calc

headers = {
    "Ntrip-Version": "Ntrip/2.0",
    "User-Agent": "NTRIP ntrip_ros",
    "Connection": "close",
    # 'Authorization': 'Basic ' + b64encode(self.ntc.ntrip_user + ':' + str(self.ntc.ntrip_pass))
    "Authorization": "Basic %s" % encoded_u,
}


now = datetime.utcnow()
ggaString= "GPGGA,%02d%02d%04.2f,3734.087,N,12702.603,E,1,12,1.0,0.0,M,0.0,M,," % \
    (now.hour,now.minute,now.second)
checksum = calcultateCheckSum(ggaString)
nmea_gga = "$%s*%s" % (ggaString, checksum)

connection = HTTPConnection("fkp.ngii.go.kr:2201")
connection.request(
    "GET",
    "/" + "VRS_V32",
    nmea_gga,
    headers,
)

while True:
    response = connection.getresponse()
    print(response.status)
connection.close()
