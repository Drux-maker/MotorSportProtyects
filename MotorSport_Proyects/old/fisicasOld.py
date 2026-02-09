"""
Simulador de Telemetría de Vehículo
Simula datos realistas de un coche en pista: velocidad, RPM, temperaturas,
presiones, combustible, GPS, aceleraciones, etc. No es un simulador físico
preciso; imita el comportamiento típico (acelerar en rectas, frenar en
curvas, subir temperaturas con el uso) para generar datos creíbles.
"""

import numpy as np
import time
from datetime import datetime
from typing import Dict
import random


class TelemetrySimulator:
    """
    Mantiene un estado interno del vehículo (velocidad, marcha, temperaturas, etc.)
    y en cada update() avanza la simulación un delta_time, actualizando ese estado
    y devolviendo un diccionario con todos los canales de telemetría.
    """

    def __init__(self, car, setup, circuit, track_points):
        """
        Inicializa el simulador con los datos físicos y track_points reales.
        """
        # --- Pista y sesión ---
        self.track_length = circuit["length_m"]
        self.current_distance = 0.0   # metros recorridos en la vuelta actual
        self.lap_count = 0            # número de vueltas completadas
        self.start_time = None        # se fija en el primer update()
        self.session_start = datetime.now()

        # --- Track points y posición sobre la pista ---
        self.track_points = track_points
        self.track_point_index = 0

        # --- Parámetros físicos de coche/setup ---
        self.weight = car["weight"]
        self.brake_force = setup["brake_force"]
        self.aero_factor = setup["aero_factor"]
        self.suspension_factor = setup["suspension_factor"]

        # --- Modelo de conductor ya no se usa driver_cycle_time -- se controla físicamente ---

        # --- Límites del vehículo (constantes) ---
        self.max_speed = setup["max_speed"]   # km/h
        self.max_rpm = setup["max_rpm"]
        self.idle_rpm = setup["idle_rpm"]
        self.max_throttle = setup["max_throttle"] # %
        self.max_brake = setup["max_brake"]     # %

        # --- Estado dinámico: controles y movimiento ---
        self.speed = 0.0      # km/h
        self.rpm = self.idle_rpm
        self.throttle = 0.0   # 0-100 %
        self.brake = 0.0      # 0-100 %
        self.gear = 1         # 1-6 marchas, 0 = punto muerto (solo cuando parado)

        # --- Temperaturas (°C) ---
        self.engine_temp = 90.0
        self.oil_temp = 85.0
        self.water_temp = 88.0
        self.tire_temp_fl = 25.0  # Front Left, Right, Rear Left, Right
        self.tire_temp_fr = 25.0
        self.tire_temp_rl = 25.0
        self.tire_temp_rr = 25.0

        # --- Presiones (bar) ---
        self.oil_pressure = 4.5
        self.fuel_pressure = 3.0
        self.tire_pressure_fl = setup["tire_pressure_fl"]
        self.tire_pressure_fr = setup["tire_pressure_fr"]
        self.tire_pressure_rl = setup["tire_pressure_rl"]
        self.tire_pressure_rr = setup["tire_pressure_rr"]

        # --- Combustible ---
        self.fuel_level = 100.0            # %
        self.fuel_consumption_rate = 0.0   # L/min

        # --- Posición GPS ---
        self.latitude = track_points[0]["y"]   # punto de partida
        self.longitude = track_points[0]["x"]
        self.altitude = track_points[0]["z"]     # metros

        # --- Aceleraciones (m/s²) ---
        self.accel_x = 0.0   # longitudinal (adelante/atrás)
        self.accel_y = 0.0   # lateral
        self.accel_z = 0.0   # vertical

        # --- Ángulos (grados) ---
        self.steering_angle = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        # --- Temperatura de frenos (°C) ---
        self.brake_temp_fl = 25.0
        self.brake_temp_fr = 25.0
        self.brake_temp_rl = 25.0
        self.brake_temp_rr = 25.0

        # --- Sistema eléctrico ---
        self.battery_voltage = 12.6
        self.alternator_current = 0.0

    def update(self, delta_time: float = 0.1) -> Dict:
        """
        Avanza la simulación un paso de tiempo delta_time (en segundos).
        Orden de operaciones: primero se simula el “conductor” (throttle/brake/marchas),
        luego se actualizan velocidad, distancia, temperaturas, presiones, combustible,
        posición GPS, aceleraciones, ángulos, frenos y electricidad. Al final se
        construye y devuelve el diccionario de telemetría.

        Args:
            delta_time: Tiempo transcurrido desde la última llamada a update() (segundos).

        Returns:
            Diccionario con todas las claves que espera el DataLogger (timestamp,
            session_time, speed_kmh, rpm, gear, throttle, brake, temperaturas, etc.).
        """
        if self.start_time is None:
            self.start_time = time.time()

        elapsed_time = time.time() - self.start_time

        # 1 Decidir throttle, brake, dirección y marcha 
        self._simulate_driver_behavior(delta_time, elapsed_time)

        # 2 A partir de throttle/brake/marcha, actualizar velocidad, RPM y distancia
        self._update_vehicle_dynamics(delta_time)

        # 3 Temperaturas motor, aceite, agua y neumáticos
        self._update_temperatures(delta_time)

        # 4 Presiones aceite, combustible y neumáticos
        self._update_pressures(delta_time)

        # 5 Consumo y nivel de combustible
        self._update_fuel(delta_time)

        # 6 Posición GPS (lat/lon/alt) en función de la distancia en la pista
        self._update_position(delta_time)

        # 7 Aceleraciones X, Y, Z según throttle, freno y dirección
        self._update_accelerations()

        # 8 Pitch y roll a partir de aceleraciones
        self._update_angles()

        # 9 Temperatura de los discos de freno
        self._update_brake_temps(delta_time)

        # 10 Batería y alternador
        self._update_electrical_system()

        lap_time = self._calculate_lap_time()
        return self._get_telemetry_data(elapsed_time, lap_time)

    def _simulate_driver_behavior(self, delta_time: float, elapsed_time: float):
        """
        Simula el comportamiento del piloto físicamente según la curvatura, grip y radio
        de los track points (punto anterior, actual, siguiente).
        """
        n = len(self.track_points)
        idx = self.track_point_index
        prev_idx = (idx - 1) % n
        next_idx = (idx + 1) % n
        prev_pt = self.track_points[prev_idx]
        curr_pt = self.track_points[idx]
        next_pt = self.track_points[next_idx]

        # Calcular radio real de curva entre 3 puntos y velocidad máxima física para grip y setup
        radius = self._compute_radius(prev_pt, curr_pt, next_pt)
        import math
        # BANQUING: modificamos grip según el ángulo de peralte
        banking = curr_pt.get('banking', 0.0)  # en grados
        banking_rad = math.radians(banking)
        grip = curr_pt.get('grip', 1.0)
        grip_efectivo = grip * (1.0 + math.sin(banking_rad))
        vmax_curve = self._compute_vmax_curve(radius, grip_efectivo)

        # Estrategia: Si velocidad actual > vmax segura, frenar; si menor, acelerar
        v_ms = self.speed / 3.6
        vmax_ms = vmax_curve / 3.6
        if v_ms > vmax_ms:
            # Si hay que frenar para la curva
            self.throttle = 0.0
            over_speed = min(1.0, (v_ms - vmax_ms) / vmax_ms)
            self.brake = min(100.0, 30.0 + 70.0 * over_speed)  # Frena más si te pasas mucho
        else:
            # Puedes acelerar, aprovecha agarre de curva
            self.throttle = min(100.0, 60.0 + 40.0 * (1.0 - (v_ms / vmax_ms)))
            self.brake = max(0.0, 15.0 - 15.0 * (v_ms / vmax_ms))

        # Cálculo de steering angle según ángulo entre puntos
        self.steering_angle = self._compute_steering_angle(prev_pt, curr_pt, next_pt)

        # Lógica de marchas igual que antes
        if self.speed < 2.0 and self.throttle < 5.0:
            self.gear = 0
        elif self.gear == 0 and self.throttle > 10.0:
            self.gear = 1
        elif self.gear > 0:
            if self.rpm > self.max_rpm and self.gear < 6:
                self.gear += 1
            elif self.rpm < 2000 and self.gear > 1:
                self.gear -= 1

    def _update_vehicle_dynamics(self, delta_time: float):
        """
        Actualiza velocidad, RPM y avance usando fuerzas físicas y cambia el punto de pista.
        """
        import math
        if self.gear > 0:
            motor_force = (self.throttle / 100.0) * self.aero_factor * 7000
            drag = 0.5 * 1.3 * (self.speed / 3.6)**2 * 2.2
            brake = (self.brake / 100.0) * self.brake_force
            net_force = motor_force - drag - brake

            # Pendiente entre puntos
            n = len(self.track_points)
            idx = self.track_point_index
            idx_next = (idx + 1) % n

            # Extraer z, x, y de los puntos actual y siguiente
            z1 = self.track_points[idx]['z']
            z2 = self.track_points[idx_next]['z']
            x1 = self.track_points[idx]['x']
            x2 = self.track_points[idx_next]['x']
            y1 = self.track_points[idx]['y']
            y2 = self.track_points[idx_next]['y']

            # Distancia horizontal entre puntos (en plano)
            dist_horiz = math.dist((x1, y1), (x2, y2))
            if dist_horiz == 0:
                alpha = 0.0
            else:
                alpha = math.atan2(z2 - z1, dist_horiz)

            F_pend = self.weight * 9.81 * math.sin(alpha)

            # Sumar esta fuerza a la neta (ya están motor, drag, freno)
            net_force += F_pend

            acceleration = net_force / self.weight

            self.speed = max(0.0, self.speed + acceleration * delta_time * 3.6)
            self.speed = min(self.max_speed, self.speed)

            if self.gear > 0:
                self.rpm = self.idle_rpm + (self.speed / self.max_speed) * (self.max_rpm - self.idle_rpm) * (7 - self.gear) / 6
                self.rpm = min(self.max_rpm, max(self.idle_rpm, self.rpm))
            else:
                self.rpm = self.idle_rpm
        else:
            self.speed = max(0.0, self.speed - 5.0 * delta_time)
            self.rpm = self.idle_rpm

        # Avance sobre el circuito y cambio de punto de track
        avance_m = (self.speed / 3.6) * delta_time  # m
        self.current_distance += avance_m
        # Cambio de punto cada vez que rebasas la distancia al siguiente
        # (puedes calcular esta distancia entre puntos, para simplificar asume regular o precalcula)
        siguiente_idx = (self.track_point_index + 1) % len(self.track_points)
        dx = self.track_points[siguiente_idx]['x'] - self.track_points[self.track_point_index]['x']
        dy = self.track_points[siguiente_idx]['y'] - self.track_points[self.track_point_index]['y']
        dseg = (dx**2 + dy**2)**0.5
        if self.current_distance >= self.track_length:
            self.current_distance = 0.0
            self.lap_count += 1
            self.track_point_index = 0
        else:
            if avance_m >= dseg:
                self.track_point_index = (self.track_point_index + 1) % len(self.track_points)

    def _update_temperatures(self, delta_time: float):
        """
        Temperatura objetivo del motor según RPM y velocidad; motor, aceite y agua
        se acercan a su objetivo con un factor de suavizado. Neumáticos: suben
        con velocidad y ángulo de dirección, bajan por “enfriamiento” constante.
        Se limitan entre 25 y 120 °C.
        """
        target_engine_temp = 85.0 + (self.rpm / self.max_rpm) * 15.0 + (self.speed / self.max_speed) * 5.0
        self.engine_temp += (target_engine_temp - self.engine_temp) * delta_time * 0.5
        target_oil_temp = self.engine_temp - 5.0
        self.oil_temp += (target_oil_temp - self.oil_temp) * delta_time * 0.4
        target_water_temp = self.engine_temp - 2.0
        self.water_temp += (target_water_temp - self.water_temp) * delta_time * 0.3

        tire_heat = (self.speed / self.max_speed) * 40.0 + abs(self.steering_angle) * 0.3
        cooling = 2.0 * delta_time
        for attr in ('tire_temp_fl', 'tire_temp_fr', 'tire_temp_rl', 'tire_temp_rr'):
            current = getattr(self, attr)
            new_val = current + tire_heat * delta_time - cooling
            setattr(self, attr, max(25.0, min(120.0, new_val)))

    def _update_pressures(self, delta_time: float):
        """
        Presión de aceite hacia un valor que sube con RPM. Combustible con
        pequeño ruido. Neumáticos: presión base + efecto de temperatura
        (más caliente → más presión) más ruido aleatorio.
        """
        target_oil_pressure = 3.0 + (self.rpm / self.max_rpm) * 2.5
        self.oil_pressure += (target_oil_pressure - self.oil_pressure) * delta_time * 2.0
        self.fuel_pressure = 2.8 + random.uniform(-0.1, 0.1)
        temp_factor = (self.tire_temp_fl - 25.0) / 100.0
        for attr in ('tire_pressure_fl', 'tire_pressure_fr', 'tire_pressure_rl', 'tire_pressure_rr'):
            setattr(self, attr, 1.8 + temp_factor * 0.2 + random.uniform(-0.02, 0.02))

    def _update_fuel(self, delta_time: float):
        """
        Consumo instantáneo (L/min) proporcional a RPM y throttle. Reduce
        fuel_level según ese consumo y delta_time; se asume tanque de 50 L
        para convertir litros consumidos en porcentaje.
        """
        self.fuel_consumption_rate = (self.rpm / self.max_rpm) * (self.throttle / 100.0) * 15.0
        fuel_consumed = (self.fuel_consumption_rate / 60.0) * delta_time
        self.fuel_level = max(0.0, self.fuel_level - (fuel_consumed / 50.0) * 100.0)

    def _update_position(self, delta_time: float):
        """
        Actualiza GPS usando el punto del track actual
        """
        actual_pt = self.track_points[self.track_point_index]
        self.latitude = actual_pt['y']
        self.longitude = actual_pt['x']
        self.altitude = actual_pt['z'] if 'z' in actual_pt else 0.0

    def _update_accelerations(self):
        """
        Calcula aceleraciones reales:
        - accel_x: de la aceleración física (longitudinal)
        - accel_y: lateral calculada por el radio de la curva real y velocidad,
                   limitada por el grip efectivo (incluido el banking)
        - accel_z: vertical (gravedad/baches)
        """
        # Longitudinal
        self.accel_x = (self.throttle / 100.0) * 8.0 - (self.brake / 100.0) * 10.0

        # Lateral física real con grip efectivo (incluye banking)
        n = len(self.track_points)
        idx = self.track_point_index
        prev_idx = (idx - 1) % n
        next_idx = (idx + 1) % n
        prev_pt = self.track_points[prev_idx]
        curr_pt = self.track_points[idx]
        next_pt = self.track_points[next_idx]
        radius = self._compute_radius(prev_pt, curr_pt, next_pt)
        v_ms = self.speed / 3.6

        import math
        banking = curr_pt.get('banking', 0.0)
        banking_rad = math.radians(banking)
        grip_efectivo = curr_pt.get('grip', 1.0) * (1.0 + math.sin(banking_rad))
        width = curr_pt.get('width', 10.0)
        if width > 12.0:
            grip_efectivo *= 1.05  # pista muy ancha: se puede trazar mejor
        elif width < 8.0:
            grip_efectivo *= 0.96  # pista muy angosta: se reduce algo el grip
        a_lat_fis_max = grip_efectivo * 9.81 * (1 + 0.2 * self.aero_factor + 0.1 * self.suspension_factor)

        if radius > 0:
            accel_y_calc = (v_ms ** 2) / radius
            self.accel_y = max(-a_lat_fis_max, min(a_lat_fis_max, accel_y_calc))
        else:
            self.accel_y = 0.0
        self.accel_z = 9.81 + random.uniform(-0.5, 0.5)

    def _update_angles(self):
        """Pitch y roll derivados de las aceleraciones longitudinal y lateral."""
        self.pitch = self.accel_x * 2.0
        self.roll = self.accel_y * 3.0

    def _update_brake_temps(self, delta_time: float):
        """
        Subida de temperatura de frenos con uso (brake y velocidad), enfriamiento
        constante. Valores limitados entre 25 y 800 °C.
        """
        brake_heat = (self.brake / 100.0) * (self.speed / self.max_speed) * 50.0
        cooling = 3.0 * delta_time
        for attr in ('brake_temp_fl', 'brake_temp_fr', 'brake_temp_rl', 'brake_temp_rr'):
            current = getattr(self, attr)
            setattr(self, attr, max(25.0, min(800.0, current + brake_heat * delta_time - cooling)))

    # === FUNCIONES AUXILIARES FÍSICAS ===
    def _compute_radius(self, p1, p2, p3):
        """
        Calcula el radio de la curva definida por tres puntos de track
        """
        import math
        a = math.dist((p1['x'], p1['y']), (p2['x'], p2['y']))
        b = math.dist((p2['x'], p2['y']), (p3['x'], p3['y']))
        c = math.dist((p3['x'], p3['y']), (p1['x'], p1['y']))
        s = (a + b + c) / 2
        area = math.sqrt(max(s * (s - a) * (s - b) * (s - c), 1e-6))
        if area == 0:
            return float('inf')
        radius = (a * b * c) / (4 * area)
        return radius

    def _compute_vmax_curve(self, radius, grip):
        """
        Calcula la velocidad segura máxima física para ese radio y grip
        """
        import math
        a_lat_max = grip * 9.81 * (1 + 0.2 * self.aero_factor + 0.1 * self.suspension_factor)
        v_max = math.sqrt(a_lat_max * max(radius,1.0))
        return v_max * 3.6  # km/h

    def _compute_steering_angle(self, prev_pt, curr_pt, next_pt):
        """
        Ángulo relativo entre los segmentos (grados, para reflejar la curva real)
        """
        import math
        v1 = (curr_pt['x'] - prev_pt['x'], curr_pt['y'] - prev_pt['y'])
        v2 = (next_pt['x'] - curr_pt['x'], next_pt['y'] - curr_pt['y'])
        dot = v1[0]*v2[0] + v1[1]*v2[1]
        norm1 = (v1[0]**2 + v1[1]**2) ** 0.5
        norm2 = (v2[0]**2 + v2[1]**2) ** 0.5
        if norm1 * norm2 == 0:
            return 0.0
        ang = math.acos(max(-1.0, min(1.0, dot / (norm1 * norm2))))
        # Dirección del giro (izq/dcha)
        cross = v1[0]*v2[1] - v1[1]*v2[0]
        deg = math.degrees(ang)
        if cross < 0:
            deg = -deg
        # Limita para no generar steering irreales
        return max(-45.0, min(45.0, deg))

    def _update_electrical_system(self):
        """
        Con motor en marcha (RPM > ralentí): voltaje de carga (~13.8 V) y corriente
        del alternador proporcional a RPM. En ralentí/parado: voltaje de batería
        que baja un poco si hay poco combustible (simplificado).
        """
        if self.rpm > self.idle_rpm:
            self.battery_voltage = 13.8 + random.uniform(-0.1, 0.1)
            self.alternator_current = (self.rpm / self.max_rpm) * 50.0
        else:
            self.battery_voltage = 12.6 - (100.0 - self.fuel_level) * 0.01
            self.alternator_current = 0.0

    def _calculate_lap_time(self) -> float:
        """
        Estima el tiempo de vuelta (segundos) con una velocidad media aproximada.
        Si no se ha completado ninguna vuelta, devuelve 0.
        """
        if self.lap_count == 0:
            return 0.0
        avg_speed = self.max_speed * 0.75  # km/h
        lap_time_seconds = (self.track_length / 1000.0) / (avg_speed / 3600.0)
        return lap_time_seconds

    def _get_telemetry_data(self, elapsed_time: float, lap_time: float) -> Dict:
        import math
        # Obtener puntos actuales e índice
        n = len(self.track_points)
        idx = self.track_point_index
        idx_next = (idx + 1) % n
        curr_pt = self.track_points[idx]
        next_pt = self.track_points[idx_next]

        # Banking, width, grip efectivo
        banking = curr_pt.get('banking', 0.0)
        width = curr_pt.get('width', 10.0)
        grip_efectivo = getattr(self, '_last_grip_efectivo', curr_pt.get('grip', 1.0))

        # Pendiente longitudinal y fuerza gravedad
        z1 = curr_pt['z']
        z2 = next_pt['z']
        x1, y1 = curr_pt['x'], curr_pt['y']
        x2, y2 = next_pt['x'], next_pt['y']
        dist_horiz = math.dist((x1, y1), (x2, y2))
        if dist_horiz > 0:
            pendiente_rad = math.atan2(z2 - z1, dist_horiz)
        else:
            pendiente_rad = 0.0
        fuerza_gravedad = self.weight * 9.81 * math.sin(pendiente_rad)

        # Slip ratio = aceleración lateral / aceleración máxima posible
        a_lat_max = grip_efectivo * 9.81 * (1 + 0.2 * self.aero_factor + 0.1 * self.suspension_factor)
        slip_ratio = abs(self.accel_y) / a_lat_max if a_lat_max > 0 else 0.0

        data = {
            'timestamp': datetime.now().isoformat(),
            'session_time': elapsed_time,
            'lap_count': self.lap_count,
            'lap_time': lap_time,
            'distance': self.current_distance,
            'speed_kmh': round(self.speed, 2),
            'speed_ms': round(self.speed / 3.6, 2),
            'rpm': round(self.rpm, 0),
            'gear': self.gear,
            'throttle': round(self.throttle, 1),
            'brake': round(self.brake, 1),
            'steering_angle': round(self.steering_angle, 1),
            'engine_temp': round(self.engine_temp, 1),
            'oil_temp': round(self.oil_temp, 1),
            'water_temp': round(self.water_temp, 1),
            'tire_temp_fl': round(self.tire_temp_fl, 1),
            'tire_temp_fr': round(self.tire_temp_fr, 1),
            'tire_temp_rl': round(self.tire_temp_rl, 1),
            'tire_temp_rr': round(self.tire_temp_rr, 1),
            'brake_temp_fl': round(self.brake_temp_fl, 1),
            'brake_temp_fr': round(self.brake_temp_fr, 1),
            'brake_temp_rl': round(self.brake_temp_rl, 1),
            'brake_temp_rr': round(self.brake_temp_rr, 1),
            'oil_pressure': round(self.oil_pressure, 2),
            'fuel_pressure': round(self.fuel_pressure, 2),
            'tire_pressure_fl': round(self.tire_pressure_fl, 2),
            'tire_pressure_fr': round(self.tire_pressure_fr, 2),
            'tire_pressure_rl': round(self.tire_pressure_rl, 2),
            'tire_pressure_rr': round(self.tire_pressure_rr, 2),
            'fuel_level': round(self.fuel_level, 1),
            'fuel_consumption_rate': round(self.fuel_consumption_rate, 2),
            'latitude': round(self.latitude, 6),
            'longitude': round(self.longitude, 6),
            'altitude': round(self.altitude, 1),
            'accel_x': round(self.accel_x, 2),
            'accel_y': round(self.accel_y, 2),
            'accel_z': round(self.accel_z, 2),
            'pitch': round(self.pitch, 2),
            'roll': round(self.roll, 2),
            'battery_voltage': round(self.battery_voltage, 2),
            'alternator_current': round(self.alternator_current, 1),
            
            # Nuevos campos físicos
            'grip_efectivo': round(grip_efectivo, 3),
            'banking': round(banking, 2),
            'width': round(width, 2),
            'pendiente_actual': round(math.degrees(pendiente_rad), 2),
            'fuerza_gravedad_pendiente': round(fuerza_gravedad, 1),
            'slip_ratio': round(slip_ratio, 2),
            'track_point_index': idx,
        }
