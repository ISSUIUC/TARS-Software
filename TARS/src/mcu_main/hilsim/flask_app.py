import flask
from flask import Flask, jsonify, request
import stream_data
import json
from flask_cors import CORS, cross_origin
import os
import traceback

app = Flask(__name__)
CORS(app)#, supports_credentials=True)

@app.route('/', methods=["GET", "POST"])
def index():
    if request.method == "POST":
        csvData = json.loads(request.get_data().decode())
        # TODO: validate data here
        result = ""
        try:
            result = stream_data.run_hilsim(csvData['csv_data'])
        except:
            print(traceback.format_exc())
            return "Internal Server Error: " + traceback.format_exc(), 500
    print("HILSIM run returned")
    return {'log': result}, 200

if __name__ == "__main__":
    v_result = stream_data.validate_ports()
    if(v_result == "OK"):
        app.run(host='0.0.0.0', debug=True)
    else:
        print("ERR: " + v_result)
