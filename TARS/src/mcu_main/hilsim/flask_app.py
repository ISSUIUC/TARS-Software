import flask
from flask import Flask, jsonify, request
import stream_data
import json
from flask_cors import CORS, cross_origin
import os

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
            return "Internal Server Error", 500
    return {'log': result}, 200

if __name__ == "__main__":
    app.run(host='0.0.0.0', debug=True)
