import math
from datetime import datetime
from typing import List, Dict


class TelemetrySimulator:
    def __init__(self, car: Dict, setup: Dict, track_points: List[Dict]):
        # Datos constantes del coche y setup (ya convertidos a unidades físicas correctas)
        self.mass = setup['mass']                     # kg
        self.engine_power_hp = setup['engine_power'] # hp
        self.engine_power = self.engine_power_hp * 745.7  # Watts
        self.max_speed = setup['max_speed']           # m/s (asegúrate de que esté en m/s)
        self.max_rpm = setup['max_rpm']
        self.idle_rpm = setup['idle_rpm']
        self.max_throttle = setup['max_throttle']     # %
        self.max_brake_force = setup['max_brake_force']  # N
        self.engine_brake_force = setup['engine_brake_force']  # N
        self.brake_bias = setup['brake_bias']         # 0.0 - 1.0, proporción eje delantero
        self.max_steering_angle = math.radians(setup['max_steering_angle'])  # radianes
        self.tire_grip = setup['tire_grip']           # coeficiente fricción
        self.tire_radius = setup['tire_radious']       # m
        self.rolling_distance = setup['rolling_distance']  # m
        self.drag_coefficient = setup['drag_coefficioent']
        self.frontal_area = setup['frontal_area']     # m²
        self.downforce_coefficient = setup['downforce_coefficient'] # coeficiente downforce
        self.air_density = setup.get('air_density', 1.225)  # kg/m3, valor por defecto
        self.traction_control = setup['traction_control']  # booleano o % (0-1)
        self.abs = setup['abs']                        # booleano

        # Estado dinámico inicial
        self.pos_x = track_points[0]['x']
        self.pos_y = track_points[0]['y']
        self.pos_z = track_points[0]['z']
        self.velocity = 0.0          # m/s
        self.acceleration = 0.0      # m/s²
        self.heading = 0.0           # radianes dirección pista

        # Track points y posición actual
        self.track_points = track_points
        self.track_point_index = 0

        # Tiempo total de simulación
        self.time_total = 0.0

        # Historial telemetría
        self.telemetry_data = []

    def update(self, delta_time: float = 0.1):
        self.time_total += delta_time

        # 1. Obtener puntos: anterior, actual, siguiente
        n = len(self.track_points)
        idx = self.track_point_index
        p_prev = self.track_points[(idx - 1) % n]
        p_curr = self.track_points[idx]
        p_next = self.track_points[(idx + 1) % n]

        # 2. Calcular radio curva y distancia entre puntos
        radius = self._calculate_radius(p_prev, p_curr, p_next)
        dist_between = self._distance(p_curr, p_next)
        curvature = 1.0 / radius if radius != float('inf') else 0.0
        
        #3. Calcular mu efectivo
        banking_rad = math.radians(p_curr.get('banking', 0.0))
        grip_base = p_curr['grip']
        mu_effective = grip_base * (1 + math.sin(banking_rad))

        # 4. Calcular velocidad máxima permitida en curva
        normal_force = self.mass * 9.81 + self.downforce_coefficient * self.velocity**2
        max_lat_accel = mu_effective * normal_force / self.mass  # m/s^2
        max_curve_speed = math.sqrt(max_lat_accel * radius) if radius != 0 else self.max_speed

        # Limitar velocidad a lo máximo permitido por el coche y la curva
        desired_speed = min(self.velocity + self.acceleration * delta_time, max_curve_speed, self.max_speed)

        # 5. Calcular fuerzas: motor, fricción, centro, drag, downforce, total
        engine_force = self._engine_force(desired_speed)
        friction_force = self._friction_force()
        centripetal_force = self.mass * self.velocity**2 / radius if radius != 0 else 0.0
        drag_force = 0.5 * self.air_density * self.frontal_area * self.drag_coefficient * self.velocity**2
        downforce = 0.5 * self.air_density * self.frontal_area * self.downforce_coefficient * self.velocity**2

        total_force = engine_force - friction_force - drag_force - centripetal_force

        # 6. Calcular aceleración total (simplificado a longitudinal)
        self.acceleration = total_force / self.mass

        # 7. Actualizar velocidad y posición
        self.velocity += self.acceleration * delta_time
        self.velocity = max(0.0, min(self.velocity, self.max_speed))

        # Mover posición hacia siguiente punto proporcional a velocidad y delta_time
        move_dist = self.velocity * delta_time
        while move_dist > dist_between:
            move_dist -= dist_between
            self.track_point_index = (self.track_point_index + 1) % n

            # Actualizar puntos y distancia para siguiente segmento
            idx = self.track_point_index
            p_curr = self.track_points[idx]
            p_next = self.track_points[(idx + 1) % n]
            dist_between = self._distance(p_curr, p_next)

        # Interpolación simple entre p_curr y p_next para posición exacta
        ratio = move_dist / dist_between if dist_between > 0 else 0
        self.pos_x = p_curr['x'] + ratio * (p_next['x'] - p_curr['x'])
        self.pos_y = p_curr['y'] + ratio * (p_next['y'] - p_curr['y'])
        self.pos_z = p_curr['z'] + ratio * (p_next['z'] - p_curr['z'])

        # Actualizar heading según vector p_curr→p_next
        self.heading = math.atan2(p_next['y'] - p_curr['y'], p_next['x'] - p_curr['x'])

        # 8. Check si se sale de la pista (según ancho)
        error_lateral = self._lateral_error()
        if abs(error_lateral) > p_curr['width'] / 2:
            print(f"Advirtiendo: coche fuera de pista en punto {self.track_point_index}")

        # 9. Guardar telemetría actual        
        return self._get_telemetry_data(elapsed_time, lap_time)

    def _distance(self, p1, p2):
        dx = p2['x'] - p1['x']
        dy = p2['y'] - p1['y']
        dz = p2['z'] - p1['z']
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _calculate_radius(self, p1, p2, p3):
        # Usamos fórmula para radio de circunferencia pasando por 3 puntos
        a = self._distance(p1, p2)
        b = self._distance(p2, p3)
        c = self._distance(p3, p1)
        s = (a + b + c) / 2
        area = max(s*(s - a)*(s - b)*(s - c), 0)
        if area == 0:
            return float('inf')  # recta perfecta
        area = math.sqrt(area)
        radius = (a * b * c) / (4 * area)
        return radius

    def _engine_force(self, speed):
        # Simplificamos potencia motor en función de velocidad
        if speed >= self.max_speed:
            return 0
        # Ejemplo lineal decreciente (puedes modelar curvas torque reales)
        return self.engine_power * (1 - speed / self.max_speed)

    def _friction_force(self):
        # Fricción básica ejemplo para desgaste y resistencia rodadura
        return self.mass * 9.81 * 0.015  # coef friccion en suelo plano

    def _lateral_error(self):
        # Suponemos está en centro para ahora
        return 0.0
    def _get_telemetry_data(self, elapsed_time: float, lap_time: float) -> dict:
    import math

    n = len(self.track_points)
    idx = self.track_point_index
    idx_next = (idx + 1) % n
    curr_pt = self.track_points[idx]
    next_pt = self.track_points[idx_next]

    # banking, width, grip efectivo
    banking = curr_pt.get('banking', 0.0)
    banking_rad = math.radians(banking)
    grip_base = curr_pt.get('grip', 1.0)
    grip_efectivo = grip_base * (1 + math.sin(banking_rad))
    width = curr_pt.get('width', 10.0)

    # pendiente longitudinal
    z1, z2 = curr_pt['z'], next_pt['z']
    x1, y1 = curr_pt['x'], curr_pt['y']
    x2, y2 = next_pt['x'], next_pt['y']
    dist_horiz = math.dist((x1, y1), (x2, y2))
    pendiente_rad = math.atan2(z2 - z1, dist_horiz) if dist_horiz > 0 else 0.0

    # Fuerza gravedad pendiente
    g = 9.81
    fuerza_gravedad_pendiente = self.mass * g * math.sin(pendiente_rad)

    # Slip ratio: aceleración lateral / aceleración máxima física permitida
    max_lat_accel = grip_efectivo * (self.mass * g + self.downforce_coefficient * self.velocity**2) / self.mass
    slip_ratio = abs(getattr(self, 'accel_y', 0)) / max_lat_accel if max_lat_accel > 0 else 0

    data = {
        'timestamp': datetime.now().isoformat(),
        'session_time': elapsed_time,
        'lap_count': getattr(self, 'lap_count', 'no calculado'),
        'lap_time': lap_time,
        'distance': getattr(self, 'current_distance', 'no calculado'),
        'speed_kmh': round(self.velocity * 3.6, 2),  # convertir m/s a km/h
        'speed_ms': round(self.velocity, 2),
        'rpm': getattr(self, 'rpm', 'no calculado'),
        'gear': getattr(self, 'gear', 'no calculado'),
        'throttle': getattr(self, 'throttle', 'no calculado'),
        'brake': getattr(self, 'brake', 'no calculado'),
        'steering_angle': getattr(self, 'steering_angle', 'no calculado'),
        'engine_temp': getattr(self, 'engine_temp', 'no calculado'),
        'oil_temp': getattr(self, 'oil_temp', 'no calculado'),
        'water_temp': getattr(self, 'water_temp', 'no calculado'),
        'tire_temp_fl': getattr(self, 'tire_temp_fl', 'no calculado'),
        'tire_temp_fr': getattr(self, 'tire_temp_fr', 'no calculado'),
        'tire_temp_rl': getattr(self, 'tire_temp_rl', 'no calculado'),
        'tire_temp_rr': getattr(self, 'tire_temp_rr', 'no calculado'),
        'brake_temp_fl': getattr(self, 'brake_temp_fl', 'no calculado'),
        'brake_temp_fr': getattr(self, 'brake_temp_fr', 'no calculado'),
        'brake_temp_rl': getattr(self, 'brake_temp_rl', 'no calculado'),
        'brake_temp_rr': getattr(self, 'brake_temp_rr', 'no calculado'),
        'oil_pressure': getattr(self, 'oil_pressure', 'no calculado'),
        'fuel_pressure': getattr(self, 'fuel_pressure', 'no calculado'),
        'tire_pressure_fl': getattr(self, 'tire_pressure_fl', 'no calculado'),
        'tire_pressure_fr': getattr(self, 'tire_pressure_fr', 'no calculado'),
        'tire_pressure_rl': getattr(self, 'tire_pressure_rl', 'no calculado'),
        'tire_pressure_rr': getattr(self, 'tire_pressure_rr', 'no calculado'),
        'fuel_level': getattr(self, 'fuel_level', 'no calculado'),
        'fuel_consumption_rate': getattr(self, 'fuel_consumption_rate', 'no calculado'),
        'latitude': round(self.pos_y, 6),
        'longitude': round(self.pos_x, 6),
        'altitude': round(self.pos_z, 2),
        'accel_x': round(getattr(self, 'accel_x', 0), 3),
        'accel_y': round(getattr(self, 'accel_y', 0), 3),
        'accel_z': round(getattr(self, 'accel_z', 0), 3),
        'pitch': getattr(self, 'pitch', 'no calculado'),
        'roll': getattr(self, 'roll', 'no calculado'),
        'battery_voltage': getattr(self, 'battery_voltage', 'no calculado'),
        'alternator_current': getattr(self, 'alternator_current', 'no calculado'),
        'grip_efectivo': round(grip_efectivo, 3),
        'banking': round(banking, 2),
        'width': round(width, 2),
        'pendiente_actual': round(math.degrees(pendiente_rad), 2),
        'fuerza_gravedad_pendiente': round(fuerza_gravedad_pendiente, 2),
        'slip_ratio': round(slip_ratio, 3),
        'track_point_index': idx,
    }

    return data