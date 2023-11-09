import requests
import time

while True:
    r = requests.get('http://localhost:5000/quadruped?data=1')
    print(r.text)
    time.sleep(1)
