"""
Sistema de Registro de Datos de Telemetría
Guarda los datos en formato CSV (para análisis en Excel/pandas) y JSON
(para conservar la sesión completa con metadatos).

Las sesiones se nombran secuencialmente: session_1, session_2, etc.
"""

import csv
import json
from typing import Dict, List
from pathlib import Path


class DataLogger:
    """
    Registra datos de telemetría en archivos CSV y JSON.
    Cada llamada a log() escribe una fila en el CSV y añade un registro al buffer;
    al cerrar, se vuelca el buffer al JSON.
    """

    def __init__(self, output_dir: str = "telemetry_data"):
        """
        Inicializa el logger: crea la carpeta, asigna nombre de sesión
        (session_1, session_2, ...), abre el CSV con cabeceras y deja
        listo el buffer para el JSON.

        Args:
            output_dir: Carpeta donde se crearán session_X.csv y session_X.json.
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)

        # Nombre de sesión numerado (session_1, session_2, ...)
        session_number = self._get_next_session_number()
        self.session_name = f"session_{session_number}"

        self.csv_file = self.output_dir / f"{self.session_name}.csv"
        self.json_file = self.output_dir / f"{self.session_name}.json"

        # Buffer en memoria para luego escribir todo el JSON de una vez
        self.data_buffer: List[Dict] = []
        self.csv_writer = None
        self.csv_file_handle = None

        # Crear CSV y escribir la fila de cabeceras
        self._init_csv()

    def _get_next_session_number(self) -> int:
        """
        Calcula el siguiente número de sesión buscando archivos session_*.csv
        y session_*.json en output_dir. Solo se consideran nombres con número
        al final (session_1, session_2). El siguiente será max(encontrados) + 1.
        Si no hay ninguno, devuelve 1.

        Returns:
            Número a usar para esta sesión (ej: 1, 2, 3...).
        """
        if not self.output_dir.exists():
            return 1

        session_numbers = []

        # Revisar CSVs con formato session_X
        for file_path in self.output_dir.glob("session_*.csv"):
            try:
                name = file_path.stem           # "session_3" sin .csv
                number_str = name.split("_")[-1]  # "3"
                session_numbers.append(int(number_str))
            except (ValueError, IndexError):
                continue

        # Revisar también JSONs por si solo existe el JSON
        for file_path in self.output_dir.glob("session_*.json"):
            try:
                name = file_path.stem
                number_str = name.split("_")[-1]
                session_numbers.append(int(number_str))
            except (ValueError, IndexError):
                continue

        if not session_numbers:
            return 1

        return max(session_numbers) + 1

    def _init_csv(self):
        """
        Abre el archivo CSV en escritura, define la lista de columnas
        (en el mismo orden que las claves del dict de telemetría) y
        escribe la fila de cabeceras.
        """
        # Orden de columnas: debe coincidir con las claves del dict que devuelve el simulador
        self.columns = [
            'timestamp', 'session_time', 'lap_count', 'lap_time', 'distance',
            'speed_kmh', 'speed_ms', 'rpm', 'gear',
            'throttle', 'brake', 'steering_angle',
            'engine_temp', 'oil_temp', 'water_temp',
            'tire_temp_fl', 'tire_temp_fr', 'tire_temp_rl', 'tire_temp_rr',
            'brake_temp_fl', 'brake_temp_fr', 'brake_temp_rl', 'brake_temp_rr',
            'oil_pressure', 'fuel_pressure',
            'tire_pressure_fl', 'tire_pressure_fr', 'tire_pressure_rl', 'tire_pressure_rr',
            'fuel_level', 'fuel_consumption_rate',
            'latitude', 'longitude', 'altitude',
            'accel_x', 'accel_y', 'accel_z',
            'pitch', 'roll',
            'battery_voltage', 'alternator_current','grip_efectivo',
            'banking', 'width', 'pendiente_actual', 'fuerza_gravedad_pendiente', 'slip_ratio', 'track_point_index'
        ]

        self.csv_file_handle = open(self.csv_file, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.DictWriter(self.csv_file_handle, fieldnames=self.columns)
        self.csv_writer.writeheader()

    def log(self, data: Dict):
        """
        Registra un punto de telemetría: escribe una fila en el CSV y
        guarda una copia del dict en el buffer para el JSON.

        Args:
            data: Diccionario con todas las claves definidas en self.columns.
        """
        self.csv_writer.writerow(data)
        self.csv_file_handle.flush()  # Para que se vea en disco aunque el programa falle después

        self.data_buffer.append(data.copy())

    def save_json(self):
        """
        Escribe el archivo JSON de la sesión: un objeto con session_info
        (nombre, inicio, fin, total de muestras) y telemetry_data (lista
        de todos los puntos guardados en data_buffer).
        """
        session_data = {
            'session_info': {
                'session_name': self.session_name,
                'start_time': self.data_buffer[0]['timestamp'] if self.data_buffer else None,
                'end_time': self.data_buffer[-1]['timestamp'] if self.data_buffer else None,
                'total_samples': len(self.data_buffer),
            },
            'telemetry_data': self.data_buffer
        }

        with open(self.json_file, 'w', encoding='utf-8') as f:
            json.dump(session_data, f, indent=2, ensure_ascii=False)

    def close(self):
        """
        Cierra el CSV y, si hay datos en el buffer, escribe el JSON.
        Debe llamarse al terminar la simulación (main.py lo hace en finally).
        """
        if self.csv_file_handle:
            self.csv_file_handle.close()

        if self.data_buffer:
            self.save_json()

        print(f"\n[OK] Datos guardados:")
        print(f"  CSV: {self.csv_file}")
        print(f"  JSON: {self.json_file}")
        print(f"  Total de muestras: {len(self.data_buffer)}")

    def get_session_info(self) -> Dict:
        """
        Devuelve un diccionario con el nombre de sesión, rutas del CSV/JSON
        y número de muestras. Útil para que main.py sepa qué CSV usar al
        visualizar justo después de simular.

        Returns:
            Dict con session_name, csv_file, json_file, samples_logged.
        """
        return {
            'session_name': self.session_name,
            'csv_file': str(self.csv_file),
            'json_file': str(self.json_file),
            'samples_logged': len(self.data_buffer)
        }
