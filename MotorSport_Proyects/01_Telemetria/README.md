# Sistema de Telemetría de Vehículo en Pista

Sistema completo para simular, registrar y visualizar datos de telemetría de un vehículo en pista, diseñado como lo usaría un ingeniero de rendimiento de vehículos.

## Características

- **Simulación Realista**: Simula datos de telemetría incluyendo velocidad, RPM, temperaturas, presiones, GPS, aceleraciones y más
- **Registro de Datos**: Guarda datos en formato CSV y JSON para análisis posterior
- **Visualización Profesional**: Genera gráficos tipo dashboard como los usados en ingeniería de vehículos
- **Análisis de Vueltas**: Compara múltiples vueltas superpuestas
- **Mapa de Pista**: Visualiza la trayectoria GPS con colores según velocidad

## Instalación

1. Instalar las dependencias:
```bash
pip install -r requirements.txt
```

## Uso

### Simulación Básica

Ejecutar una simulación de 60 segundos (por defecto):
```bash
python main.py --simulate
```

### Simulación con Parámetros Personalizados

```bash
python main.py --simulate --duration 120 --sample-rate 20 --track-length 4000
```

Parámetros disponibles:
- `--duration`: Duración en segundos (default: 60)
- `--sample-rate`: Frecuencia de muestreo en Hz (default: 10)
- `--track-length`: Longitud de pista en metros (default: 5000)

### Simulación y Visualización Automática

```bash
python main.py --simulate --duration 120 --visualize
```

### Solo Visualización

Visualizar datos de un archivo CSV existente:
```bash
python main.py --visualize --csv telemetry_data/session_20240204_120000.csv
```

## Datos Simulados

El sistema simula los siguientes parámetros:

### Velocidad y Motor
- Velocidad (km/h y m/s)
- RPM
- Marcha
- Throttle y Brake (%)

### Temperaturas
- Motor
- Aceite
- Agua
- Neumáticos (FL, FR, RL, RR)
- Frenos (FL, FR, RL, RR)

### Presiones
- Aceite
- Combustible
- Neumáticos (FL, FR, RL, RR)

### Combustible
- Nivel (%)
- Tasa de consumo (L/min)

### GPS y Posición
- Latitud/Longitud
- Altitud
- Distancia recorrida

### Aceleraciones
- Longitudinal (X)
- Lateral (Y)
- Vertical (Z)

### Otros
- Ángulo de dirección
- Pitch y Roll
- Voltaje de batería
- Corriente del alternador

## Estructura de Archivos

```
01_Telemetria/
├── main.py                  # Script principal
├── telemetry_simulator.py   # Simulador de datos
├── data_logger.py          # Sistema de registro
├── visualizer.py           # Generador de gráficos
├── requirements.txt        # Dependencias
├── README.md              # Este archivo
├── telemetry_data/        # Datos guardados (CSV/JSON)
└── telemetry_plots/       # Gráficos generados
```

## Gráficos Generados

El sistema genera 4 tipos de gráficos:

1. **Dashboard Completo**: Vista general con 8 paneles mostrando todos los parámetros principales
2. **Speed Trace**: Velocidad vs distancia en la pista
3. **Comparación de Vueltas**: Superposición de múltiples vueltas para análisis comparativo
4. **Mapa de Pista**: Trayectoria GPS coloreada según velocidad

## Ejemplo de Salida

Los datos se guardan en:
- `telemetry_data/session_YYYYMMDD_HHMMSS.csv` - Datos en formato CSV
- `telemetry_data/session_YYYYMMDD_HHMMSS.json` - Datos en formato JSON

Los gráficos se guardan en:
- `telemetry_plots/session_YYYYMMDD_HHMMSS_dashboard.png`
- `telemetry_plots/session_YYYYMMDD_HHMMSS_speed_trace.png`
- `telemetry_plots/session_YYYYMMDD_HHMMSS_lap_comparison.png`
- `telemetry_plots/session_YYYYMMDD_HHMMSS_track_map.png`

## Personalización

Puedes modificar los parámetros del simulador en `telemetry_simulator.py`:
- Velocidad máxima
- RPM máximo
- Longitud de pista
- Y muchos otros parámetros del vehículo

## Requisitos

- Python 3.8+
- numpy
- pandas
- matplotlib
