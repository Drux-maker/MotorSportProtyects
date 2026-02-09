"""
Sistema de Telemetría de Vehículo en Pista
Simula, registra y visualiza datos como un ingeniero de rendimiento de vehículos.

Este archivo es el punto de entrada del programa: orquesta la simulación,
el registro de datos y la generación de gráficos según los argumentos
que se pasen por línea de comandos.
"""

import time
import argparse
from telemetry_simulator import TelemetrySimulator
from data_logger import DataLogger
from visualizer import TelemetryVisualizer
from bbddConection import get_db_connection


def run_simulation(duration: float = 60.0, sample_rate: float = 10.0,
                   track_length: float = 5000.0, output_dir: str = "telemetry_data"):
    """
    Ejecuta una simulación de telemetría durante un tiempo dado.

    Crea un simulador y un logger, y en un bucle obtiene datos del simulador
    a la frecuencia indicada (sample_rate), los guarda con el logger y
    muestra un resumen por consola cada segundo. Al terminar (por tiempo
    o por Ctrl+C) cierra el logger y devuelve la info de la sesión.

    Args:
        duration: Duración de la simulación en segundos.
        sample_rate: Frecuencia de muestreo en Hz (muestras por segundo).
        output_dir: Directorio donde se guardan CSV y JSON (ej: telemetry_data).
    """
    # --- Datos de la BBDD ---
    conn = get_db_connection()
    cursor = conn.cursor(dictionary=True)

    # Obtener lista de coches
    cursor.execute("SELECT car_id, brand, model FROM cars")
    cars = cursor.fetchall()
    print("Coches disponibles:")
    for c in cars:
        print(f"{c['car_id']} - {c['brand']} {c['model']}")
    car_id = int(input("Selecciona el ID de un coche: "))

    # Obtener lista de circuitos
    cursor.execute("SELECT id, name, country FROM circuits")
    circuits = cursor.fetchall()
    print("Circuitos disponibles:")
    for c in circuits:
        print(f"{c['id']} - {c['name']} ({c['country']})")
    circuit_id = int(input("Selecciona el ID de un circuito: "))

    # Obtener el coche y el setup seleccionado
    cursor.execute("SELECT * FROM cars WHERE car_id = %s", (car_id,))
    car = cursor.fetchone()
    cursor.execute("SELECT * FROM car_setup WHERE car_id = %s", (car_id,))
    setup = cursor.fetchone()

    # Obtener el circuito y sus puntos seleccionado
    cursor.execute("SELECT * FROM circuits WHERE id = %s", (circuit_id,))
    circuit = cursor.fetchone()
    cursor.execute("SELECT * FROM track_points WHERE circuit_id = %s ORDER BY point_index", (circuit_id,))
    track_points = cursor.fetchall()

    

    # --- Configuración inicial ---
    print("=" * 60)
    print("SISTEMA DE TELEMETRÍA - SIMULACIÓN DE VEHÍCULO EN PISTA")
    print("=" * 60)
    print(f"\nConfiguración:")
    print(f"  Duración: {duration} segundos")
    print(f"  Frecuencia de muestreo: {sample_rate} Hz")
    print(f"\nUbicación:")
    print(f"  Nombre: {circuit["name"]} ")    
    print(f"  Pais: {circuit["country"]} ")
    print(f"  Longitud de pista: {circuit["length_m"]} metros")    
    print(f"  Numero de curvas: {circuit["turns"]} ")
    print(f"\nIniciando simulación...\n")

    # Crear simulador (pasamos los datos de la bbdd al simulador) y logger (guarda CSV/JSON)
    simulator = TelemetrySimulator(car, setup, track_points)
    logger = DataLogger(output_dir=output_dir)

    # Intervalo entre muestras: si sample_rate=10 Hz, sample_interval=0.1 s
    sample_interval = 1.0 / sample_rate
    start_time = time.time()
    sample_count = 0
    session_info = None

    try:
        # --- Bucle principal de simulación ---
        while True:
            elapsed = time.time() - start_time

            # Salir cuando se cumpla la duración pedida
            if elapsed >= duration:
                break

            # Pedir al simulador un nuevo “frame” de telemetría (actualiza estado interno y devuelve un dict)
            data = simulator.update(delta_time=sample_interval)

            # Guardar ese frame en CSV y en buffer para el JSON
            logger.log(data)

            sample_count += 1

            # Cada segundo (cada sample_rate muestras) imprimir resumen en consola
            if sample_count % int(sample_rate) == 0:
                print(f"Tiempo: {elapsed}s | "
                      f"Velocidad: {data['speed_kmh']} km/h | "
                      f"RPM: {data['rpm']} | "
                      f"Vuelta: {data['lap_count']} | "
                      f"Combustible: {data['fuel_level']}%")

            # Respetar la frecuencia de muestreo (evitar ir más rápido que sample_rate)
            time.sleep(sample_interval)

    except KeyboardInterrupt:
        # Si el usuario pulsa Ctrl+C, salir del bucle sin fallar
        print("\n\nSimulación interrumpida por el usuario")

    finally:
        # Siempre: cerrar archivos y escribir JSON final
        logger.close()
        session_info = logger.get_session_info()

        # Resumen final de la sesión
        print(f"\n{'=' * 60}")
        print("SIMULACIÓN COMPLETADA")
        print(f"{'=' * 60}")
        print(f"Total de muestras: {sample_count}")
        print(f"Vueltas completadas: {simulator.lap_count}")
        print(f"Distancia total: {simulator.current_distance + simulator.lap_count * track_length:.1f} m")

    # Devolver info de sesión para que, si se usa --visualize, se sepa qué CSV usar
    return session_info


def visualize_data(csv_file: str, output_dir: str = "telemetry_plots"):
    """
    Genera todos los gráficos a partir de un CSV de telemetría.

    Carga el CSV, crea el visualizador con estilo oscuro y llama a
    generate_all_plots para producir dashboard, speed trace, comparación
    de vueltas y mapa de pista. Los PNG se guardan en output_dir.

    Args:
        csv_file: Ruta al archivo CSV (ej: telemetry_data/session_1.csv).
        output_dir: Carpeta donde se guardan los PNG (ej: telemetry_plots).
    """
    print(f"\nGenerando visualizaciones desde: {csv_file}")

    visualizer = TelemetryVisualizer(style='dark_background')
    visualizer.generate_all_plots(csv_file, output_dir)

    print("\n[OK] Visualizacion completada")


def main():
    """
    Función principal: interpreta argumentos de línea de comandos y
    ejecuta simulación y/o visualización según lo que pida el usuario.
    """
    # --- Definir argumentos que acepta el programa ---
    parser = argparse.ArgumentParser(
        description='Sistema de Telemetría de Vehículo en Pista',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos de uso:
  # Ejecutar simulación de 60 segundos
  python main.py --simulate --duration 60

  # Ejecutar simulación y generar gráficos automáticamente
  python main.py --simulate --duration 120 --visualize

  # Solo visualizar datos existentes
  python main.py --visualize --csv telemetry_data/session_1.csv
        """
    )

    parser.add_argument('--simulate', action='store_true',
                        help='Ejecutar simulación de telemetría')
    parser.add_argument('--visualize', action='store_true',
                        help='Generar gráficos de visualización')
    parser.add_argument('--csv', type=str,
                        help='Archivo CSV para visualizar (si no se especifica, usa el último generado)')
    parser.add_argument('--duration', type=float, default=60.0,
                        help='Duración de la simulación en segundos (default: 60)')
    parser.add_argument('--sample-rate', type=float, default=10.0,
                        help='Frecuencia de muestreo en Hz (default: 10)')
    parser.add_argument('--track-length', type=float, default=5000.0,
                        help='Longitud de la pista en metros (default: 5000)')
    parser.add_argument('--output-dir', type=str, default='telemetry_data',
                        help='Directorio para guardar datos (default: telemetry_data)')
    parser.add_argument('--plots-dir', type=str, default='telemetry_plots',
                        help='Directorio para guardar gráficos (default: telemetry_plots)')

    args = parser.parse_args()

    # Si no se pasó --simulate ni --visualize, por defecto hacemos ambas
    if not args.simulate and not args.visualize:
        args.simulate = True
        args.visualize = True

    session_info = None

    # --- Ejecutar simulación ---
    if args.simulate:
        session_info = run_simulation(
            duration=args.duration,
            sample_rate=args.sample_rate,
            track_length=args.track_length,
            output_dir=args.output_dir
        )

    # --- Generar gráficos ---
    if args.visualize:
        # Decidir de qué CSV leer: el indicado con --csv, el recién generado, o el último del directorio
        if args.csv:
            csv_file = args.csv
        elif session_info:
            csv_file = session_info['csv_file']
        else:
            # Buscar el CSV más reciente en output_dir (por fecha de modificación)
            from pathlib import Path
            data_dir = Path(args.output_dir)
            if data_dir.exists():
                csv_files = list(data_dir.glob("*.csv"))
                if csv_files:
                    csv_file = str(max(csv_files, key=lambda p: p.stat().st_mtime))
                else:
                    print("Error: No se encontraron archivos CSV para visualizar")
                    return
            else:
                print("Error: No se encontró el directorio de datos")
                return

        visualize_data(csv_file, args.plots_dir)


# Al ejecutar "python main.py", se llama a main()
if __name__ == "__main__":
    main()
