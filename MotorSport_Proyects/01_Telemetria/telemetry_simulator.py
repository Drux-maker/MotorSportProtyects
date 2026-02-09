import math, ast
from datetime import datetime
from typing import List, Dict


class TelemetrySimulator:
    def __init__(self, car: Dict, setup: Dict, track_points: List[Dict]):

        # ===== DATOS DEL COCHE =====
        self.mass = setup['mass']
        self.engine_power_hp = setup['engine_power']   # HP
        self.max_speed = setup['max_speed'] / 3.6      # m/s
        self.max_rpm = setup['max_rpm']
        self.idle_rpm = setup['idle_rpm']

        self.max_brake_force = setup['max_brake_force']
        self.engine_brake_force = setup['engine_brake_force']

        self.tire_grip = setup['tire_grip']
        self.tire_radius = setup['tire_radius']

        self.drag_coefficient = setup['drag_coefficient']
        self.frontal_area = setup['frontal_area']
        self.downforce_coefficient = setup['downforce_coefficient']
        self.air_density = setup.get('air_density', 1.225)

        self.fuel_level = 100.0

        # ===== TRANSMISIÓN =====
        self.gear_ratios = ast.literal_eval(setup['gear_ratios'])
        self.final_drive = setup['final_drive']

        # ===== ESTADO DINÁMICO =====
        self.velocity = 0.0
        self.acceleration = 0.0

        self.gear = 1
        self.rpm = self.idle_rpm

        self.throttle = 0.0
        self.brake = 0.0

        # ===== PISTA =====
        self.track_points = track_points
        self.track_point_index = 0
        self.current_distance = 0.0
        self.lap_count = 0

        self.pos_x = track_points[0]['x']
        self.pos_y = track_points[0]['y']
        self.pos_z = track_points[0]['z']

        self.time_total = 0.0

    # =========================================================

    def update(self, delta_time: float = 0.1):
        self.time_total += delta_time

        # === PUNTOS DE PISTA ===
        n = len(self.track_points)
        idx = self.track_point_index
        p_prev = self.track_points[(idx - 1) % n]
        p_curr = self.track_points[idx]
        p_next = self.track_points[(idx + 1) % n]

        # === GEOMETRÍA DE CURVA ===
        radius = self._calculate_radius(p_prev, p_curr, p_next)
        dist_between = self._distance(p_curr, p_next)

        # === GRIP EFECTIVO CON BANKING ===
        banking_rad = math.radians(p_curr.get('banking', 0.0))
        grip_base = p_curr.get('grip', 1.0)
        grip_efectivo = grip_base * (1 + math.sin(banking_rad))

        # === FUERZA NORMAL (peso + downforce) ===
        normal_force = self.mass * 9.81 + self.downforce_coefficient * self.velocity**2

        # === ACELERACIÓN LATERAL MÁXIMA ===
        max_lat_accel = grip_efectivo * normal_force / self.mass

        # === VELOCIDAD MÁXIMA FÍSICA EN CURVA ===
        if radius == float('inf'):
            max_curve_speed = self.max_speed
        else:
            max_curve_speed = math.sqrt(max_lat_accel * radius)

        # === VELOCIDAD OBJETIVO (TRAZADA PERFECTA) ===
        desired_speed = min(max_curve_speed, self.max_speed)

        # =====================================================
        # CONTROL AUTOMÁTICO DE GAS Y FRENO (CLAVE)
        # =====================================================
        speed_error = desired_speed - self.velocity

        if speed_error > 0.5:
            # Necesitamos acelerar
            self.throttle = min(speed_error / desired_speed, 1.0)
            self.brake = 0.0
        elif speed_error < -0.5:
            # Necesitamos frenar
            self.throttle = 0.0
            self.brake = min(abs(speed_error) / desired_speed, 1.0)
        else:
            # Mantener velocidad
            self.throttle = 0.0
            self.brake = 0.0

        # === FUERZAS ===
        engine_force = self._engine_force()
        brake_force = self.max_brake_force * self.brake
        rolling_resistance = self._friction_force()
        drag_force = 0.5 * self.air_density * self.frontal_area * self.drag_coefficient * self.velocity**2

        total_force = engine_force - brake_force - rolling_resistance - drag_force

        # === DINÁMICA LONGITUDINAL ===
        self.acceleration = total_force / self.mass
        self.velocity += self.acceleration * delta_time
        self.velocity = max(0.0, min(self.velocity, self.max_speed))

        # === AVANCE SOBRE LA PISTA ===
        move_dist = self.velocity * delta_time
        self.current_distance += move_dist

        while move_dist > dist_between:
            move_dist -= dist_between
            self.track_point_index = (self.track_point_index + 1) % n

            if self.track_point_index == 0:
                self.lap_count += 1
                self.current_distance = 0.0

            idx = self.track_point_index
            p_curr = self.track_points[idx]
            p_next = self.track_points[(idx + 1) % n]
            dist_between = self._distance(p_curr, p_next)

        # === INTERPOLACIÓN DE POSICIÓN ===
        ratio = move_dist / dist_between if dist_between > 0 else 0.0
        self.pos_x = p_curr['x'] + ratio * (p_next['x'] - p_curr['x'])
        self.pos_y = p_curr['y'] + ratio * (p_next['y'] - p_curr['y'])
        self.pos_z = p_curr['z'] + ratio * (p_next['z'] - p_curr['z'])

        # === ORIENTACIÓN ===
        self.heading = math.atan2(
            p_next['y'] - p_curr['y'],
            p_next['x'] - p_curr['x']
        )

        lap_time = 0.0
        return self._get_telemetry_data(self.time_total, lap_time)

    # =========================================================

    def _engine_force(self):
        if self.throttle <= 0:
            return 0.0

        gear_ratio = self.gear_ratios[self.gear - 1]

        # RPM desde ruedas
        wheel_rpm = (self.velocity / (2 * math.pi * self.tire_radius)) * 60
        self.rpm = max(self.idle_rpm, wheel_rpm * gear_ratio * self.final_drive)

        # Cambio de marchas
        if self.rpm > self.max_rpm and self.gear < len(self.gear_ratios):
            self.gear += 1
        elif self.rpm < self.idle_rpm * 1.2 and self.gear > 1:
            self.gear -= 1

        self.rpm = max(self.idle_rpm, min(self.rpm, self.max_rpm))

        # PAR MOTOR (FÓRMULA CLAVE)
        torque = (self.engine_power_hp * 7022) / self.rpm

        wheel_force = (torque * gear_ratio * self.final_drive) / self.tire_radius
        return wheel_force * self.throttle

    # =========================================================

    def _distance(self, p1, p2):
        return math.sqrt(
            (p2['x'] - p1['x'])**2 +
            (p2['y'] - p1['y'])**2 +
            (p2['z'] - p1['z'])**2
        )

    def _calculate_radius(self, p1, p2, p3):
        a = self._distance(p1, p2)
        b = self._distance(p2, p3)
        c = self._distance(p3, p1)

        s = (a + b + c) / 2
        area_sq = s * (s - a) * (s - b) * (s - c)

        if area_sq <= 0:
            return float('inf')

        area = math.sqrt(area_sq)
        return (a * b * c) / (4 * area)

    def _friction_force(self):
        return self.mass * 9.81 * 0.015

    # =========================================================

    def _get_telemetry_data(self, elapsed_time: float, lap_time: float) -> dict:
        import math
        idx = self.track_point_index
        n = len(self.track_points)
        idx_next = (idx + 1) % n
        curr_pt = self.track_points[idx]
        next_pt = self.track_points[idx_next]

        banking = curr_pt.get('banking', 0.0)
        banking_rad = math.radians(banking)
        grip_base = curr_pt.get('grip', 1.0)
        grip_efectivo = grip_base * (1 + math.sin(banking_rad))
        width = curr_pt.get('width', 10.0)

        z1, z2 = curr_pt['z'], next_pt['z']
        x1, y1 = curr_pt['x'], curr_pt['y']
        x2, y2 = next_pt['x'], next_pt['y']
        dist_horiz = math.dist((x1, y1), (x2, y2))
        pendiente_rad = math.atan2(z2 - z1, dist_horiz) if dist_horiz > 0 else 0.0

        g = 9.81
        fuerza_gravedad_pendiente = self.mass * g * math.sin(pendiente_rad)

        max_lat_accel = grip_efectivo * (self.mass * g + self.downforce_coefficient * self.velocity ** 2) / self.mass
        slip_ratio = abs(getattr(self, 'accel_y', 0)) / max_lat_accel if max_lat_accel > 0 else 0

        data = {
            # Tiempos y distancia
            'timestamp': datetime.now().isoformat(),
            'session_time': elapsed_time,
            'lap_count': getattr(self, 'lap_count', self.lap_count),
            'lap_time': lap_time,
            'distance': getattr(self, 'current_distance', 'no calculado'),
            
            # Velocidad
            'speed_kmh': round(self.velocity * 3.6, 2) if hasattr(self, 'velocity') else 'no calculado',
            'speed_ms': round(self.velocity, 2) if hasattr(self, 'velocity') else 'no calculado',
            
            # Estado del motor y controles
            'rpm': getattr(self, 'rpm', 'no calculado'),
            'gear': getattr(self, 'gear', 'no calculado'),
            'throttle': getattr(self, 'throttle', 'no calculado'),
            'brake': getattr(self, 'brake', 'no calculado'),
            'steering_angle': getattr(self, 'steering_angle', 'no calculado'),
            
            # Temperaturas
            'engine_temp': getattr(self, 'engine_temp', 'no calculado'),
            'oil_temp': getattr(self, 'oil_temp', 'no calculado'),
            'water_temp': getattr(self, 'water_temp', 'no calculado'),
            
            # Temperaturas de neumáticos
            'tire_temp_fl': getattr(self, 'tire_temp_fl', 'no calculado'),
            'tire_temp_fr': getattr(self, 'tire_temp_fr', 'no calculado'),
            'tire_temp_rl': getattr(self, 'tire_temp_rl', 'no calculado'),
            'tire_temp_rr': getattr(self, 'tire_temp_rr', 'no calculado'),
            
            # Temperaturas de frenos
            'brake_temp_fl': getattr(self, 'brake_temp_fl', 'no calculado'),
            'brake_temp_fr': getattr(self, 'brake_temp_fr', 'no calculado'),
            'brake_temp_rl': getattr(self, 'brake_temp_rl', 'no calculado'),
            'brake_temp_rr': getattr(self, 'brake_temp_rr', 'no calculado'),
            
            # Presiones
            'oil_pressure': getattr(self, 'oil_pressure', 'no calculado'),
            'fuel_pressure': getattr(self, 'fuel_pressure', 'no calculado'),
            
            # Presiones de neumáticos
            'tire_pressure_fl': getattr(self, 'tire_pressure_fl', 'no calculado'),
            'tire_pressure_fr': getattr(self, 'tire_pressure_fr', 'no calculado'),
            'tire_pressure_rl': getattr(self, 'tire_pressure_rl', 'no calculado'),
            'tire_pressure_rr': getattr(self, 'tire_pressure_rr', 'no calculado'),
            
            # Combustible
            'fuel_level': getattr(self, 'fuel_level', 'no calculado'),
            'fuel_consumption_rate': getattr(self, 'fuel_consumption_rate', 'no calculado'),
            
            # Posición y orientación
            'latitude': round(self.pos_y, 6) if hasattr(self, 'pos_y') else 'no calculado',
            'longitude': round(self.pos_x, 6) if hasattr(self, 'pos_x') else 'no calculado',
            'altitude': round(self.pos_z, 2) if hasattr(self, 'pos_z') else 'no calculado',
            
            # Aceleración
            'accel_x': round(getattr(self, 'accel_x', 0), 3),
            'accel_y': round(getattr(self, 'accel_y', 0), 3),
            'accel_z': round(getattr(self, 'accel_z', 0), 3),
            
            # Orientación del vehículo
            'pitch': getattr(self, 'pitch', 'no calculado'),
            'roll': getattr(self, 'roll', 'no calculado'),
            
            # Sistema eléctrico
            'battery_voltage': getattr(self, 'battery_voltage', 'no calculado'),
            'alternator_current': getattr(self, 'alternator_current', 'no calculado'),
            
            # Dinámica del vehículo y pista
            #'grip_efectivo': round(grip_efectivo, 3),
            #'banking': round(banking, 2),
            #'width': round(width, 2),
            #'pendiente_actual': round(pendiente_grados, 2),
            #'fuerza_gravedad_pendiente': round(fuerza_gravedad_pendiente, 2),
            #'slip_ratio': round(slip_ratio, 3),*/
            
            # Información de pista
            'track_point_index': idx,
        }    
        return data