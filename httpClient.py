# simple client app that continually grabs data from http://localhost:5000/quadruped?data=1,2,3,4
# and prints the response

import requests
import time

while True:
    r = requests.get('http://localhost:5000/quadruped?data=1,2,3,4')
    print(r.text)
    time.sleep(1)
