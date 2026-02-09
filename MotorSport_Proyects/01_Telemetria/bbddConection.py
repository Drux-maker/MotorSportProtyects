# c:\Users\danie\Desktop\MotorSport_Proyects\01_Telemetria\bbddConexion.py

import mysql.connector

def get_db_connection():
    db_config = {
        'user': 'root',
        'password': 'loki',
        'host': 'localhost',
        'database': 'racing_simulation',
        'port': 3306
    }
    conn = mysql.connector.connect(**db_config)
    return conn