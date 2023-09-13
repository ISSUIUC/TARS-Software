#make a POST request

import requests
import os

with open(os.path.join(os.path.dirname(__file__), "./flight_computer.csv"), "r") as csv_file:
    data = {'csv_data':csv_file.read()}
    res = requests.post('http://127.0.0.1:5000/', json=data)
    print('response from server: ' + res.text)