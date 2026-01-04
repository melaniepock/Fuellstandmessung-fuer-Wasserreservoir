from flask import Flask
import os
import csv
from flask_cors import CORS

FILE_PATH = "dataFiles/testData.csv"

app = Flask(__name__)
CORS(app)

@app.get("/data")
def get_data():
    rows = []
    if not os.path.exists(FILE_PATH):
        return rows

    with open(FILE_PATH, "r", encoding="utf-8", newline="") as f:
        reader = csv.reader(f, delimiter=";")
        for ts, val in reader:
            rows.append({"timestamp": int(ts), "value": float(val)})

    return rows


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5000, debug=True)
