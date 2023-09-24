import flask
from flask import Flask, jsonify, request
import stream_data
import json
from flask_cors import CORS, cross_origin
import os
import traceback
import time

app = Flask(__name__)
CORS(app)#, supports_credentials=True)

run_locked = False
job_start_time = time.time()
job_runtime = 0

@app.route('/', methods=["GET", "POST"])
def index():
    global run_locked, job_queue
    if request.method == "POST":
        csvData = json.loads(request.get_data().decode())
        # TODO: validate data here
        result = ""
        try:
            if(run_locked):
                # Another job is running, deny other jobs
                remaining_sec = (time.time() - job_start_time)
                return {f'status': 'err', 'log': 'Job denied - job running, approx remaining time {remaining_sec:.1f}s'}, 400
            else:
                run_locked = True
                job_start_time = time.time()
                job_runtime = stream_data.get_runtime(csvData['csv_data'])
                result = stream_data.run_hilsim(csvData['csv_data'])
        except:
            run_locked = False
            print(traceback.format_exc())
            return "Internal Server Error: " + traceback.format_exc(), 500
    run_locked = False
    print("HILSIM run returned")
    return {'status': "ok", 'log': result}, 200

if __name__ == "__main__":
    v_result = stream_data.validate_ports()
    if(v_result == "OK"):
        app.run(host='0.0.0.0', debug=True)
    else:
        print("ERR: " + v_result)
