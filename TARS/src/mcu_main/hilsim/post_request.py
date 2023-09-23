#make a POST request

import requests
import os
import json

file = os.path.join(os.path.dirname(__file__), "./post_return.txt")

with open(os.path.join(os.path.dirname(__file__), "./flight_computer.csv"), "r") as csv_file:
    data = {'csv_data':csv_file.read()}
    res = requests.post('http://127.0.0.1:5000/', json=data)
    f = open(file, "w")
    f.write(json.loads(res.text)["log"])
    print('Got a response! Saved to post_return.txt')