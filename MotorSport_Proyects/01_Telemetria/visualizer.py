"""
Visualizador de Datos de Telemetría
Crea gráficos profesionales a partir de un CSV de sesión: dashboard completo,
speed trace (velocidad vs distancia), comparación de vueltas y mapa de pista
con trayectoria GPS. Usa matplotlib con estilo oscuro por defecto.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from typing import Optional


class TelemetryVisualizer:
    """
    Carga un CSV de telemetría y genera varios tipos de gráficos,
    guardándolos en PNG o mostrándolos en pantalla.
    """

    def __init__(self, style: str = 'dark_background'):
        """
        Inicializa el visualizador aplicando un estilo de matplotlib
        (ej: fondo oscuro, líneas claras).

        Args:
            style: Nombre del estilo de matplotlib (dark_background, seaborn, etc.).
        """
        plt.style.use(style)
        self.fig_size = (16, 10)
        self.dpi = 100

    def load_data(self, csv_file: str) -> pd.DataFrame:
        """
        Lee el CSV y lo convierte en DataFrame. Convierte la columna
        timestamp a tipo datetime y session_time a numérico para
        poder plotear correctamente.

        Args:
            csv_file: Ruta al archivo CSV.

        Returns:
            DataFrame con los datos listos para graficar.
        """
        df = pd.read_csv(csv_file)
        df['timestamp'] = pd.to_datetime(df['timestamp'])
        df['session_time'] = pd.to_numeric(df['session_time'])
        return df

    def plot_dashboard(self, df: pd.DataFrame, output_file: Optional[str] = None):
        """
        Crea una figura con 8 subgráficos (grid 4x4): velocidad+RPM,
        throttle/brake, temperaturas motor, temperaturas neumáticos,
        presiones aceite/combustible, combustible (nivel y consumo),
        aceleraciones X/Y, temperaturas frenos. Si se pasa output_file,
        guarda la figura en PNG y cierra; si no, la muestra con plt.show().

        Args:
            df: DataFrame con columnas session_time, speed_kmh, rpm, throttle, brake, etc.
            output_file: Ruta del PNG (opcional). Si es None, se usa plt.show().
        """
        fig = plt.figure(figsize=(20, 12))
        gs = fig.add_gridspec(4, 4, hspace=0.3, wspace=0.3)

        # Panel 1: Velocidad (eje izquierdo) y RPM (eje derecho) vs tiempo
        ax1 = fig.add_subplot(gs[0, :2])
        ax1_twin = ax1.twinx()
        ax1.plot(df['session_time'], df['speed_kmh'], 'b-', linewidth=2, label='Velocidad (km/h)')
        ax1_twin.plot(df['session_time'], df['rpm'], 'r-', linewidth=2, label='RPM', alpha=0.7)
        ax1.set_xlabel('Tiempo de Sesión (s)', fontsize=10)
        ax1.set_ylabel('Velocidad (km/h)', color='b', fontsize=10)
        ax1_twin.set_ylabel('RPM', color='r', fontsize=10)
        ax1.set_title('Velocidad y RPM vs Tiempo', fontsize=12, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper left')
        ax1_twin.legend(loc='upper right')

        # Panel 2: Throttle (verde, hacia arriba) y Brake (rojo, hacia abajo) vs tiempo
        ax2 = fig.add_subplot(gs[0, 2:])
        ax2.fill_between(df['session_time'], 0, df['throttle'], alpha=0.6, color='green', label='Throttle')
        ax2.fill_between(df['session_time'], 0, -df['brake'], alpha=0.6, color='red', label='Brake')
        ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        ax2.set_xlabel('Tiempo de Sesión (s)', fontsize=10)
        ax2.set_ylabel('Throttle / Brake (%)', fontsize=10)
        ax2.set_title('Controles del Vehículo', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        # Panel 3: Temperaturas motor, aceite y agua vs tiempo
        ax3 = fig.add_subplot(gs[1, :2])
        ax3.plot(df['session_time'], df['engine_temp'], 'r-', linewidth=2, label='Motor')
        ax3.plot(df['session_time'], df['oil_temp'], 'orange', linewidth=2, label='Aceite')
        ax3.plot(df['session_time'], df['water_temp'], 'b-', linewidth=2, label='Agua')
        ax3.set_xlabel('Tiempo de Sesión (s)', fontsize=10)
        ax3.set_ylabel('Temperatura (°C)', fontsize=10)
        ax3.set_title('Temperaturas del Motor', fontsize=12, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        ax3.legend()

        # Panel 4: Temperaturas de los cuatro neumáticos (FL, FR, RL, RR) vs tiempo
        ax4 = fig.add_subplot(gs[1, 2:])
        ax4.plot(df['session_time'], df['tire_temp_fl'], 'r-', linewidth=2, label='FL')
        ax4.plot(df['session_time'], df['tire_temp_fr'], 'b-', linewidth=2, label='FR')
        ax4.plot(df['session_time'], df['tire_temp_rl'], 'g-', linewidth=2, label='RL')
        ax4.plot(df['session_time'], df['tire_temp_rr'], 'orange', linewidth=2, label='RR')
        ax4.set_xlabel('Tiempo de Sesión (s)', fontsize=10)
        ax4.set_ylabel('Temperatura (°C)', fontsize=10)
        ax4.set_title('Temperaturas de Neumáticos', fontsize=12, fontweight='bold')
        ax4.grid(True, alpha=0.3)
        ax4.legend()

        # Panel 5: Presión aceite (izq) y presión combustible (der) vs tiempo
        ax5 = fig.add_subplot(gs[2, :2])
        ax5_twin = ax5.twinx()
        ax5.plot(df['session_time'], df['oil_pressure'], 'r-', linewidth=2, label='Aceite')
        ax5_twin.plot(df['session_time'], df['fuel_pressure'], 'b-', linewidth=2, label='Combustible')
        ax5.set_xlabel('Tiempo de Sesión (s)', fontsize=10)
        ax5.set_ylabel('Presión Aceite (bar)', color='r', fontsize=10)
        ax5_twin.set_ylabel('Presión Combustible (bar)', color='b', fontsize=10)
        ax5.set_title('Presiones del Sistema', fontsize=12, fontweight='bold')
        ax5.grid(True, alpha=0.3)
        ax5.legend(loc='upper left')
        ax5_twin.legend(loc='upper right')

        # Panel 6: Nivel de combustible (izq) y consumo L/min (der) vs tiempo
        ax6 = fig.add_subplot(gs[2, 2:])
        ax6_twin = ax6.twinx()
        ax6.plot(df['session_time'], df['fuel_level'], 'g-', linewidth=2, label='Nivel (%)')
        ax6_twin.plot(df['session_time'], df['fuel_consumption_rate'], 'orange', linewidth=2, label='Consumo (L/min)', alpha=0.7)
        ax6.set_xlabel('Tiempo de Sesión (s)', fontsize=10)
        ax6.set_ylabel('Nivel Combustible (%)', color='g', fontsize=10)
        ax6_twin.set_ylabel('Consumo (L/min)', color='orange', fontsize=10)
        ax6.set_title('Sistema de Combustible', fontsize=12, fontweight='bold')
        ax6.grid(True, alpha=0.3)
        ax6.legend(loc='upper left')
        ax6_twin.legend(loc='upper right')

        # Panel 7: Aceleración longitudinal (X) y lateral (Y) vs tiempo
        ax7 = fig.add_subplot(gs[3, :2])
        ax7.plot(df['session_time'], df['accel_x'], 'r-', linewidth=2, label='Longitudinal (X)')
        ax7.plot(df['session_time'], df['accel_y'], 'b-', linewidth=2, label='Lateral (Y)')
        ax7.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
        ax7.set_xlabel('Tiempo de Sesión (s)', fontsize=10)
        ax7.set_ylabel('Aceleración (m/s²)', fontsize=10)
        ax7.set_title('Aceleraciones', fontsize=12, fontweight='bold')
        ax7.grid(True, alpha=0.3)
        ax7.legend()

        # Panel 8: Temperaturas de los cuatro frenos vs tiempo
        ax8 = fig.add_subplot(gs[3, 2:])
        ax8.plot(df['session_time'], df['brake_temp_fl'], 'r-', linewidth=2, label='FL')
        ax8.plot(df['session_time'], df['brake_temp_fr'], 'b-', linewidth=2, label='FR')
        ax8.plot(df['session_time'], df['brake_temp_rl'], 'g-', linewidth=2, label='RL')
        ax8.plot(df['session_time'], df['brake_temp_rr'], 'orange', linewidth=2, label='RR')
        ax8.set_xlabel('Tiempo de Sesión (s)', fontsize=10)
        ax8.set_ylabel('Temperatura (°C)', fontsize=10)
        ax8.set_title('Temperaturas de Frenos', fontsize=12, fontweight='bold')
        ax8.grid(True, alpha=0.3)
        ax8.legend()

        plt.suptitle('Dashboard de Telemetría - Análisis Completo', fontsize=16, fontweight='bold', y=0.995)

        if output_file:
            plt.savefig(output_file, dpi=self.dpi, bbox_inches='tight')
            print(f"[OK] Dashboard guardado en: {output_file}")
        else:
            plt.show()
        plt.close()

    def plot_speed_trace(self, df: pd.DataFrame, output_file: Optional[str] = None):
        """
        Gráfico típico de ingeniería: velocidad (km/h) en el eje Y y distancia
        en la pista (m) en el eje X. Se dibuja una línea vertical discontinua
        roja al inicio de cada nueva vuelta (lap_count > 0). Si output_file
        está definido se guarda el PNG; si no, plt.show().

        Args:
            df: DataFrame con columnas distance, speed_kmh, lap_count.
            output_file: Ruta del PNG (opcional).
        """
        fig, ax = plt.subplots(figsize=(14, 6))
        ax.plot(df['distance'], df['speed_kmh'], 'b-', linewidth=2.5, label='Velocidad')
        ax.fill_between(df['distance'], 0, df['speed_kmh'], alpha=0.3, color='blue')

        for lap in df['lap_count'].unique():
            if lap > 0:
                lap_data = df[df['lap_count'] == lap]
                if not lap_data.empty:
                    ax.axvline(x=lap_data['distance'].iloc[0], color='r', linestyle='--', alpha=0.5, linewidth=1)

        ax.set_xlabel('Distancia en Pista (m)', fontsize=12)
        ax.set_ylabel('Velocidad (km/h)', fontsize=12)
        ax.set_title('Speed Trace - Velocidad vs Distancia', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend()

        if output_file:
            plt.savefig(output_file, dpi=self.dpi, bbox_inches='tight')
            print(f"[OK] Speed trace guardado en: {output_file}")
        else:
            plt.show()
        plt.close()

    def plot_lap_comparison(self, df: pd.DataFrame, output_file: Optional[str] = None):
        """
        Comparación de vueltas: se normaliza la distancia a porcentaje (0-100 %)
        dentro del total recorrido, y se superponen todas las vueltas en 4 subgráficos
        (velocidad, RPM, throttle, temperatura media de neumáticos). Cada vuelta
        tiene un color distinto (escala viridis). Útil para ver consistencia o
        diferencias entre vueltas.

        Args:
            df: DataFrame con session_time, distance, lap_count, speed_kmh, rpm, etc.
            output_file: Ruta del PNG (opcional).
        """
        fig, axes = plt.subplots(2, 2, figsize=(16, 10))
        df_normalized = df.copy()
        df_normalized['distance_pct'] = (df_normalized['distance'] / df_normalized['distance'].max()) * 100
        colors = plt.cm.viridis(np.linspace(0, 1, len(df['lap_count'].unique())))

        # Subgráfico: velocidad vs distancia %
        ax1 = axes[0, 0]
        for i, lap in enumerate(df['lap_count'].unique()):
            lap_data = df_normalized[df_normalized['lap_count'] == lap]
            if not lap_data.empty:
                ax1.plot(lap_data['distance_pct'], lap_data['speed_kmh'],
                        color=colors[i], linewidth=2, label=f'Vuelta {int(lap)}', alpha=0.7)
        ax1.set_xlabel('Distancia (%)', fontsize=10)
        ax1.set_ylabel('Velocidad (km/h)', fontsize=10)
        ax1.set_title('Comparación de Vueltas - Velocidad', fontsize=12, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # Subgráfico: RPM vs distancia %
        ax2 = axes[0, 1]
        for i, lap in enumerate(df['lap_count'].unique()):
            lap_data = df_normalized[df_normalized['lap_count'] == lap]
            if not lap_data.empty:
                ax2.plot(lap_data['distance_pct'], lap_data['rpm'],
                        color=colors[i], linewidth=2, label=f'Vuelta {int(lap)}', alpha=0.7)
        ax2.set_xlabel('Distancia (%)', fontsize=10)
        ax2.set_ylabel('RPM', fontsize=10)
        ax2.set_title('Comparación de Vueltas - RPM', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        # Subgráfico: throttle vs distancia %
        ax3 = axes[1, 0]
        for i, lap in enumerate(df['lap_count'].unique()):
            lap_data = df_normalized[df_normalized['lap_count'] == lap]
            if not lap_data.empty:
                ax3.plot(lap_data['distance_pct'], lap_data['throttle'],
                        color=colors[i], linewidth=2, label=f'Vuelta {int(lap)}', alpha=0.7)
        ax3.set_xlabel('Distancia (%)', fontsize=10)
        ax3.set_ylabel('Throttle (%)', fontsize=10)
        ax3.set_title('Comparación de Vueltas - Throttle', fontsize=12, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        ax3.legend()

        # Subgráfico: temperatura media neumáticos vs distancia %
        ax4 = axes[1, 1]
        for i, lap in enumerate(df['lap_count'].unique()):
            lap_data = df_normalized[df_normalized['lap_count'] == lap]
            if not lap_data.empty:
                avg_tire_temp = (lap_data['tire_temp_fl'] + lap_data['tire_temp_fr'] +
                                 lap_data['tire_temp_rl'] + lap_data['tire_temp_rr']) / 4
                ax4.plot(lap_data['distance_pct'], avg_tire_temp,
                        color=colors[i], linewidth=2, label=f'Vuelta {int(lap)}', alpha=0.7)
        ax4.set_xlabel('Distancia (%)', fontsize=10)
        ax4.set_ylabel('Temp. Promedio Neumáticos (°C)', fontsize=10)
        ax4.set_title('Comparación de Vueltas - Temp. Neumáticos', fontsize=12, fontweight='bold')
        ax4.grid(True, alpha=0.3)
        ax4.legend()

        plt.suptitle('Comparación de Vueltas', fontsize=16, fontweight='bold')
        plt.tight_layout()
        if output_file:
            plt.savefig(output_file, dpi=self.dpi, bbox_inches='tight')
            print(f"[OK] Comparacion de vueltas guardada en: {output_file}")
        else:
            plt.show()
        plt.close()

    def plot_track_map(self, df: pd.DataFrame, output_file: Optional[str] = None):
        """
        Mapa de la pista: ejes = longitud y latitud. Se dibuja la trayectoria
        (línea azul) y encima un scatter de los mismos puntos coloreado por
        velocidad (escala 'hot'). Punto verde = inicio, punto rojo = fin.
        Barra de color indica velocidad en km/h.

        Args:
            df: DataFrame con longitude, latitude, speed_kmh.
            output_file: Ruta del PNG (opcional).
        """
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.plot(df['longitude'], df['latitude'], 'b-', linewidth=3, alpha=0.7, label='Trayectoria')
        scatter = ax.scatter(df['longitude'], df['latitude'],
                            c=df['speed_kmh'], cmap='hot',
                            s=20, alpha=0.6, edgecolors='none')
        ax.plot(df['longitude'].iloc[0], df['latitude'].iloc[0],
                'go', markersize=15, label='Inicio', zorder=5)
        ax.plot(df['longitude'].iloc[-1], df['latitude'].iloc[-1],
                'ro', markersize=15, label='Fin', zorder=5)
        ax.set_xlabel('Longitud', fontsize=12)
        ax.set_ylabel('Latitud', fontsize=12)
        ax.set_title('Mapa de Pista - Trayectoria GPS', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_aspect('equal', adjustable='box')
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('Velocidad (km/h)', fontsize=10)

        if output_file:
            plt.savefig(output_file, dpi=self.dpi, bbox_inches='tight')
            print(f"[OK] Mapa de pista guardado en: {output_file}")
        else:
            plt.show()
        plt.close()

    def generate_all_plots(self, csv_file: str, output_dir: str = "telemetry_plots"):
        """
        Carga el CSV, crea la carpeta output_dir si no existe y genera los cuatro
        gráficos (dashboard, speed trace, comparación de vueltas, mapa de pista),
        guardando cada uno como {nombre_sesion}_{tipo}.png en output_dir.
        El nombre de sesión se toma del nombre del archivo CSV sin extensión.

        Args:
            csv_file: Ruta al CSV (ej: telemetry_data/session_1.csv).
            output_dir: Carpeta donde guardar los PNG (ej: telemetry_plots).
        """
        output_path = Path(output_dir)
        output_path.mkdir(exist_ok=True)
        df = self.load_data(csv_file)
        session_name = Path(csv_file).stem

        print(f"\nGenerando gráficos para: {session_name}")
        self.plot_dashboard(df, str(output_path / f"{session_name}_dashboard.png"))
        self.plot_speed_trace(df, str(output_path / f"{session_name}_speed_trace.png"))
        self.plot_lap_comparison(df, str(output_path / f"{session_name}_lap_comparison.png"))
        self.plot_track_map(df, str(output_path / f"{session_name}_track_map.png"))
        print(f"\n[OK] Todos los graficos guardados en: {output_path}")
