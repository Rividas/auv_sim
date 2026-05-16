import socket
import json
import time
import threading
import math
import signal
import sys

class VFHAnalyzer:
    def __init__(self, num_bins=72, max_range=15.0, safety_margin=4.0, robot_radius=0.4):
        self.num_bins = num_bins
        self.max_range = max_range
        self.safety_margin = safety_margin
        self.robot_radius = robot_radius
        self.bin_size = 360.0 / num_bins
        self.h = [0.0] * num_bins

    def analyze(self, sonar_data):
        self.h = [0.0] * self.num_bins
        min_f, min_l, min_r = self.max_range, self.max_range, self.max_range

        for r in sonar_data:
            dist = r.get('dist', self.max_range)
            angle = r.get('angle', 0.0)
            
            if -30 <= angle <= 30: min_f = min(min_f, dist)
            elif angle > 30: min_l = min(min_l, dist)
            else: min_r = min(min_r, dist)
                
            if dist < self.max_range and dist > 0.1:
                val = max(0.0, (self.max_range - dist) / self.max_range)
                bin_idx = int(((angle + 360) % 360) / self.bin_size) % self.num_bins
                spread = max(1, int(self.robot_radius / max(dist, 0.1) * 5))
                for i in range(-spread, spread + 1):
                    b = (bin_idx + i) % self.num_bins
                    if val > self.h[b]: self.h[b] = val

        return min_f, min_l, min_r

class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0, output_limits=(-100, 100)):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.setpoint = setpoint; self.output_limits = output_limits
        self.integral = 0; self.prev_error = 0; self.prev_time = time.time()
    
    def reset(self):
        self.integral = 0; self.prev_error = 0; self.prev_time = time.time()
    
    def update(self, current_value):
        dt = max(0.01, time.time() - self.prev_time)
        error = self.setpoint - current_value
        if abs(error) > 180: error = error - 360 if error > 0 else error + 360
        
        self.integral = max(-1000, min(1000, self.integral + error * dt))
        output = self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error) / dt
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        self.prev_error = error; self.prev_time = time.time()
        return output

class AUVBrain:
    def __init__(self):
        self.udp_ip = "127.0.0.1"
        self.unity_listen_port = 5000
        self.python_listen_port = 5001
        self.state = {}; self.running = True; self.lock = threading.Lock()
        
        self.pid_depth = PIDController(20.0, 0.3, 10.0, output_limits=(-80, 80))
        self.pid_heading = PIDController(45.0, 0.0, 45.0, output_limits=(-80, 80))
        
        self.vfh = VFHAnalyzer(num_bins=72, max_range=15.0, safety_margin=4.0, robot_radius=0.4)
        
        self.waypoints = [
            [0.0, 2.0, 10.0],
            [10.0, 2.0, 20.0],
            [-10.0, 2.0, 30.0],
            [0.0, 2.0, 0.0]]
        self.current_wp_index = 0; self.waypoint_radius = 2.0
        
        self.avoid_state = "CRUISE"
        self.turn_start_time = 0.0
        self.clearing_start_time = 0.0
        self.avoid_start_time = 0.0
        self.stuck_triggers = 0
        self.log_counter = 0
        
        self.alpha = 0.3
        self.s_f, self.s_l, self.s_r = 15.0, 15.0, 15.0
        
        self.proxy_mode = False
        self.original_wp_index = -1
        self.proxy_wp = None
        self.wp_approach_start = 0.0
        self.min_dist_to_wp = 999.0
        self.BLOCK_DETECTION_TIME = 12.0
        
        self.command_sock = None; self.telemetry_sock = None
        print("=" * 60)
        print("AUV Brain")
        print("=" * 60)
    
    def listen_telemetry(self):
        while self.running:
            try:
                data, _ = self.telemetry_sock.recvfrom(8192)
                with self.lock: self.state = json.loads(data.decode('utf-8'))
            except socket.timeout: continue
            except Exception: break
    
    def get_state(self):
        with self.lock: return self.state.copy()
    
    def dist_h(self, p, t): return math.hypot(t[0]-p['x'], t[2]-p['z'])
    def dist_v(self, p, t): return abs(t[1]-p['y'])
    def bearing(self, p, t): return math.degrees(math.atan2(t[0]-p['x'], t[2]-p['z'])) % 360
    def norm(self, a): return (a + 180) % 360 - 180
    def angle_diff(self, a1, a2): return abs(self.norm(a2 - a1))

    def calculate_control(self):
        try:
            state = self.get_state()
            pos = state.get('pos', {'x':0,'y':0,'z':0})
            rot = state.get('rot', {'x':0,'y':0})
            sensors = state.get('sensors', [])
            
            current_heading = rot.get('y', 0)
            current_depth = pos.get('y', 0)
            current_pitch = rot.get('x', 0)
            if current_pitch > 180: current_pitch -= 360
            if current_pitch < -180: current_pitch += 360
            
            if self.current_wp_index >= len(self.waypoints):
                print("Misson complete. Graceful shutdown...")
                self.running = False
                return [0, 0, 0, 0]
            
            raw_f, raw_l, raw_r = self.vfh.analyze(sensors)
            self.s_f = self.alpha * raw_f + (1 - self.alpha) * self.s_f
            self.s_l = self.alpha * raw_l + (1 - self.alpha) * self.s_l
            self.s_r = self.alpha * raw_r + (1 - self.alpha) * self.s_r
            worst_clearance = min(self.s_f, self.s_l, self.s_r)
            
            active_target = self.proxy_wp if self.proxy_mode else self.waypoints[self.current_wp_index]
            dh = self.dist_h(pos, active_target)
            dv = self.dist_v(pos, active_target)
            
            if dh < self.min_dist_to_wp:
                self.min_dist_to_wp = dh
                self.wp_approach_start = time.time()
            
            if not self.proxy_mode and self.avoid_state != "CRUISE":
                avoid_elapsed = time.time() - self.avoid_start_time
                if avoid_elapsed > self.BLOCK_DETECTION_TIME and dh < 3.5:
                    offset_dir = 1 if self.s_l >= self.s_r else -1
                    offset_dist = 4.0
                    wp = self.waypoints[self.current_wp_index]
                    bearing_to_wp = self.bearing(pos, wp)
                    proxy_angle = (bearing_to_wp + 90.0 * offset_dir) % 360
                    rad = math.radians(proxy_angle)
                    
                    self.proxy_wp = [
                        wp[0] + offset_dist * math.sin(rad),
                        wp[1],
                        wp[2] + offset_dist * math.cos(rad)
                    ]
                    self.proxy_mode = True
                    self.original_wp_index = self.current_wp_index
                    self.min_dist_to_wp = 999.0
                    self.avoid_state = "CRUISE"
                    self.pid_heading.reset()
                    self.pid_depth.reset()
                    print(f"WP {self.current_wp_index} blocked -> proxy created ({self.proxy_wp})")
            
            if dh < self.waypoint_radius and dv < 0.5:
                self.min_dist_to_wp = 999.0
                self.pid_heading.reset(); self.pid_depth.reset()
                
                if self.proxy_mode:
                    print("Proxy tarhet reached. Resuming route.")
                    self.proxy_mode = False
                    self.current_wp_index = self.original_wp_index + 1
                    self.proxy_wp = None
                else:
                    print(f"Waypoint {self.current_wp_index} reached!")
                    self.current_wp_index += 1
                
                if self.current_wp_index >= len(self.waypoints): 
                    print("Mission complete. Holding position.")
                    self.running = False
                    return [0, 0, 0, 0]
                
                active_target = self.waypoints[self.current_wp_index]
                dh = self.dist_h(pos, active_target)
                dv = self.dist_v(pos, active_target)

            target_heading = self.bearing(pos, active_target)
            target_depth = active_target[1]
            heading_err = self.angle_diff(current_heading, target_heading)
            base_speed = 15.0 if heading_err > 60 else (25.0 if heading_err > 30 else 35.0)
            
            if self.s_f < 5.5 and self.avoid_state == "CRUISE":
                base_speed = max(6.0, base_speed * (self.s_f / 5.5))
            
            final_heading = target_heading
            forward_force = base_speed
            heading_force = 0.0
            
            if self.avoid_state == "CRUISE":
                if self.s_f < self.vfh.safety_margin:
                    self.avoid_state = "TURNING"
                    self.avoid_start_time = time.time()
                    self.turn_start_time = time.time()
                    self.pid_heading.reset()
                    self.stuck_triggers = 0
                    turn_sign = 1 if self.s_l >= self.s_r else -1
                    self.turn_target_heading = (current_heading + 45.0 * turn_sign) % 360
                    print(f"Sturn turn ({self.turn_target_heading:.0f}°) at F:{self.s_f:.1f}m")
            
            elif self.avoid_state == "TURNING":
                final_heading = self.turn_target_heading
                forward_force = 0.0
                err = self.norm(self.turn_target_heading - current_heading)
                heading_force = 90.0 if err > 0 else -90.0
                
                elapsed = time.time() - self.turn_start_time
                turned = 45.0 - self.angle_diff(current_heading, self.turn_target_heading)
                
                if self.s_f > 4.0 and turned > 25:
                    self.avoid_state = "CLEARING"
                    self.clearing_start_time = time.time()
                    self.clearing_heading = current_heading
                    self.clearing_start_dist = self.s_f
                    print(f"Turn down -> clearing (Δ{turned:.0f}°, Lock:{self.clearing_heading:.1f}°)")
                elif elapsed > 3.0:
                    self.avoid_state = "CLEARING"
                    turn_dir = 1 if self.s_l >= self.s_r else -1
                    self.clearing_heading = (current_heading + 50.0 * turn_dir) % 360
                    self.clearing_start_time = time.time()
                    self.clearing_start_dist = self.s_f
                    print(f"Turn timeout-> clearing (Smart Lock:{self.clearing_heading:.1f}°)")
            
            elif self.avoid_state == "CLEARING":
                if worst_clearance < 1.5: target_speed = 2.0
                elif worst_clearance < 2.5: target_speed = max(2.0, min(6.0, (worst_clearance - 1.5) * 6.0))
                else: target_speed = max(6.0, min(15.0, (worst_clearance - 2.5) * 5.0 + 6.0))
                forward_force = target_speed
                
                if self.s_r < 2.5: final_heading = (self.clearing_heading - 15.0) % 360
                elif self.s_l < 2.5: final_heading = (self.clearing_heading + 15.0) % 360
                else: final_heading = self.clearing_heading
                
                dist_delta = self.s_f - self.clearing_start_dist
                elapsed = time.time() - self.clearing_start_time
                if elapsed > 1.5 and dist_delta < 0.2 and worst_clearance < 2.2:
                    self.stuck_triggers += 1
                    if self.stuck_triggers >= 3:
                        new_dir = 1 if self.s_l >= self.s_r else -1
                        self.clearing_heading = (current_heading + 60.0 * new_dir) % 360
                        final_heading = self.clearing_heading
                        print(f"Re-planning clearing ({self.clearing_heading:.1f}°)")
                        self.stuck_triggers = 0
                        self.clearing_start_dist = self.s_f
                        self.clearing_start_time = time.time()
                    forward_force = -6.0
                else:
                    self.stuck_triggers = max(0, self.stuck_triggers - 0.1)
                
                if self.s_f > 4.5 and worst_clearance > 3.0 and elapsed > 2.0:
                    self.avoid_state = "CRUISE"
                    print("Cleared -> resume cruise")
            
            if current_pitch < -5.0: forward_force = 10.0
            elif current_pitch < -3.0: forward_force = 20.0
            elif dh < self.waypoint_radius * 1.5: forward_force = min(forward_force, 20.0)
            
            if self.avoid_state != "TURNING":
                self.pid_heading.setpoint = final_heading
                heading_force = self.pid_heading.update(current_heading)
            
            self.pid_depth.setpoint = target_depth
            depth_force = self.pid_depth.update(current_depth)
            
            h_left = forward_force + heading_force
            h_right = forward_force - heading_force
            forces = [depth_force, depth_force, h_left, h_right]
            
            self.log_counter += 1
            if self.log_counter % 40 == 0 and sensors:
                mode_tag = "[Proxy]" if self.proxy_mode else ""
                print(f"{mode_tag}[{self.avoid_state}] F:{self.s_f:.1f} L:{self.s_l:.1f} R:{self.s_r:.1f} | "
                        f"T:{target_heading:5.1f}° S:{final_heading:5.1f}° SPD:{forward_force:4.1f}")
            
            return [max(-100, min(100, f)) for f in forces]
            
        except Exception as e:
            print(f"Error: {e}")
            return [0, 0, 35, 35]
    
    def send_command(self, forces):
        try:
            self.command_sock.sendto(json.dumps({"forces": forces}, separators=(',', ':')).encode('utf-8'), 
                                    (self.udp_ip, self.unity_listen_port))
        except Exception as e:
            if self.running: print(f"Cmd err: {e}")
    
    def run(self):
        threading.Thread(target=self.listen_telemetry, daemon=False).start()
        time.sleep(0.5)
        print("Loop started (50 Hz)...")
        try:
            while self.running:
                self.send_command(self.calculate_control())
                time.sleep(0.02)
        except KeyboardInterrupt: print("\nShutdown...")
        finally:
            self.running = False
            for s in [self.command_sock, self.telemetry_sock]:
                if s: 
                    try: s.close()
                    except: pass

if __name__ == "__main__":
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')

    brain = AUVBrain()
    try:
        brain.command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        brain.telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        brain.telemetry_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        brain.telemetry_sock.bind((brain.udp_ip, brain.python_listen_port))
        brain.telemetry_sock.settimeout(0.5)
        print(f"UDP: listen={brain.python_listen_port}, send={brain.unity_listen_port}")
    except Exception as e:
        print(f"Fatal: {e}"); sys.exit(1)
    
    signal.signal(signal.SIGINT, lambda s,f: (print("\nBye"), setattr(brain, 'running', False), sys.exit(0)))
    try: brain.run()
    finally: brain.running = False