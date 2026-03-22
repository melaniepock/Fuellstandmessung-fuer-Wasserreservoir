import serial
import csv
from dotenv import load_dotenv
import os
from supabase import create_client, Client
from datetime import datetime

SERIAL_PORT = 'COM4'
CSV_FILE = "newTestdata.csv"

load_dotenv()

SUPABASE_URL = os.getenv("SUPABASE_URL")
SUPABASE_KEY = os.getenv("SUPABASE_KEY")
EMAIL = os.getenv("EMAIL")
PASSWORD = os.getenv("PASSWORD")


def log_radar_data(date, value):
    with open(CSV_FILE, mode='a', newline='\n', encoding='utf-8') as f:
        writer = csv.writer(f, delimiter=";")
        writer.writerow([date, value])

def init_supabase():
    supabase: Client = create_client(SUPABASE_URL, SUPABASE_KEY)
    supabase.auth.sign_in_with_password(
        {
            "email": EMAIL,
            "password": PASSWORD,
        }
    )
    return supabase

def post_to_supabase(date, value):
    (supabase.table("water_levels")
        .insert({'measure_date': date, 'distance': value})
        .execute())
    
def get_time_supabase():
    return datetime.now().strftime("%Y-%m-%dT%H:%M:%S")

def get_data_from_serial_port(ser):
    value = ser.readline().decode().strip()
    return value

def simulate_data_from_serial():
    value = float(input("Testmode - please enter float: "))
    dataStr = f"{get_time_supabase()};{value:.2f}"
    print(dataStr)
    return dataStr


########### CONFIG ###########

TEST_MODE = 0
PRODUCTION_MODE = 1

MODE = TEST_MODE

CSV_LOG = True
SUPABASE = False

########### CONFIG ###########

if __name__ == "__main__":
    ser = None
    supabase = None

    if SUPABASE:
        supabase = init_supabase()

    if MODE == PRODUCTION_MODE: # weil auf laptop meh
        ser = serial.Serial(SERIAL_PORT, 115200, timeout=8)
        ser.flushInput()

    while True:
        try:
            dataStr = ""
            if MODE == TEST_MODE:
                dataStr = simulate_data_from_serial()
            elif MODE == PRODUCTION_MODE:
                dataStr = get_data_from_serial_port(ser)

            if not dataStr: # check für serial/PRODUCTION_MODE => muss noch getestet werden
                print("continue")
                continue

            date, value = dataStr.split(";") # format 2026-03-22T11:58:10;12.4 erwartet
            value = float(value)

            if CSV_LOG:
                log_radar_data(date, value)
            if SUPABASE:
                post_to_supabase(date, value)
        except:
            print("\nexiting program..")
            break


    if supabase != None:
        supabase.auth.sign_out()
    if ser != None:
        ser.close()
