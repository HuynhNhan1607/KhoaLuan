import os
import atexit
import signal
import json
import time
import threading
from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS

from config import (
    DEFAULT_ANGLES_FILE,
    JOINT_LIMITS_FILE,
    DEFAULT_JOINT_ANGLES,
    DEFAULT_JOINT_LIMITS,
    load_default_angles,
    load_joint_limits,
    save_default_angles,
    save_joint_limits,
    PICK_PLACE_CONFIG
)
from kinematics import (
    map_angle,
    unmap_angle,
    fk_geometric,
    ik_5dof_optimized,
    calculate_link_angles
)
from tcp_manager import TCPSocketManager

robot_stop_event = threading.Event()
def load_current_angles():
    angles = load_default_angles()
    
    math_angles = {}
    for key in ["j0", "j1", "j2", "j3", "j4", "j5"]:
        math_angles[key] = float(angles.get(key, 90))
        
    return math_angles

current_angles = load_current_angles()

sock_mgr = TCPSocketManager()
app = Flask(__name__, static_folder="web")
CORS(app)


@app.get("/api/ports")
def api_ports():
    return jsonify(sock_mgr.list_ports())


@app.get("/api/connection_status")
def api_connection_status():
    """Get detailed connection health status"""
    status = sock_mgr.get_connection_status()
    return jsonify({"ok": True, "status": status})


@app.post("/api/open")
def api_open():
    """Start TCP server"""
    try:
        sock_mgr.start_server(port=8080)
        return jsonify({"ok": True, "message": "Server started on port 8080"})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.post("/api/close")
def api_close():
    """Stop TCP server"""
    sock_mgr.stop_server()
    return jsonify({"ok": True})


@app.post("/api/init")
def api_init():
    """Manual trigger for servo_init"""
    try:
        data = request.get_json(force=True, silent=True) or {}
        freq = int(data.get("freq", 50))
        sock_mgr.send_init(freq)
        return jsonify({"ok": True, "message": f"Sent servo_init (freq={freq})"})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


def send_gravity_angles(sv0, sv1, sv2, sv3):
    """
    Calculate and send link geometric angles to ESP32 for gravity compensation.
    
    Args:
        sv0-sv3: Current servo angles for J0-J3
        
    Sends: {"cmd": "set_gravity", "angles": [j0_geo, j1_geo, j2_geo, j3_geo, 0, 0]}
    """
    try:
        link_angles = calculate_link_angles(sv0, sv1, sv2, sv3)
        # Round to 1 decimal place
        link_angles = [round(a, 1) for a in link_angles]
        cmd = {"cmd": "set_gravity", "angles": link_angles}
        sock_mgr.write_json(cmd)
        print(f"[GRAVITY] Sent link angles: J1={link_angles[1]:.1f}°, J2={link_angles[2]:.1f}°, J3={link_angles[3]:.1f}°")
    except Exception as e:
        print(f"[GRAVITY] Error sending gravity angles: {e}")


@app.post("/api/send")
def api_send():
    global current_angles
    try:
        data = request.get_json(force=True)
    except Exception as e:
        print(f"[ERROR] JSON Parse Failed: {e}")
        print(f"[ERROR] Raw data: {request.data}")
        return jsonify({"ok": False, "error": f"Invalid JSON: {str(e)}"}), 400
    try:
        cmd = data.get("cmd")
        
        # --- INTERCEPT FK/IK (Server-side calculation) ---
        if cmd == "fk":
            # Data format: {"cmd": "fk", "j": [j0, j1, j2, j3, j4, j5]}
            j_in = data.get("j", [])
            # If empty, use current angles
            if not j_in:
                # Construct from current_angles dict
                j_in = [
                    current_angles.get("j0", 90),
                    current_angles.get("j1", 90),
                    current_angles.get("j2", 90),
                    current_angles.get("j3", 90),
                    current_angles.get("j4", 90),
                    current_angles.get("j5", 90)
                ]
            
            # Extract relevant servos for 3-Link Planar FK
            # Server J0->Base, J1->Shldr, J2->Elbow, J3->WristPitch, J4->WristRoll
            # test_sim.py/fk_geometric Input: J0_deg, J2_servo, J3_servo, J4_servo
            # MAPPING: Server J1 -> Sim J2; Server J2 -> Sim J3; Server J3 -> Sim J4 ??
            # Wait, verify mapping from ik_5dof_optimized:
            # sv2(Shldr) -> j1; sv3(Elbow) -> j2; sv4(Wrist) -> j3
            
            val_j0 = float(j_in[0])
            val_j1 = float(j_in[1]) # Shoulder
            val_j2 = float(j_in[2]) # Elbow
            val_j3 = float(j_in[3]) # Wrist Pitch
            # val_j4 = float(j_in[4]) # Wrist Roll
            
            pos, phi, _ = fk_geometric(val_j0, val_j1, val_j2, val_j3)
            
            # Construct Response similar to Firmware FK
            # Firmware response: {"ok":true, "type":"fk", "pose":{"x":..., "y":..., "z":..., "a":..., "b":..., "c":...}}
            resp = {
                "ok": True,
                "type": "fk",
                "pose": {
                    "x": round(pos[0], 2),
                    "y": round(pos[1], 2),
                    "z": round(pos[2], 2),
                    "a": 0,       # Roll (not calc)
                    "b": round(phi, 2), # Pitch
                    "c": 0        # Yaw (not calc)
                }
            }
            # Inject into SerialManager RX log so frontend 'sees' it
            resp_str = json.dumps(resp)
            sock_mgr.inject_rx_data(resp_str)
            return jsonify({"ok": True})

        if cmd == "ik":
            # Data format: {"cmd": "ik", "p": {"x":.., "y":.., "z":..}}
            p = data.get("p", {})
            x = float(p.get("x", 0))
            y = float(p.get("y", 0))
            z = float(p.get("z", 0))
            phi = float(p.get("b", -60))
            
            sol = ik_5dof_optimized(x, y, z, phi_deg=phi)
            
            if sol:
                # sol = [j0, j1, j2, j3, j4]
                # Firmware response: {"ok":true,"type":"ik","solutions":[{"j":[...]}],"count":1}
                js = sol + [90.0] # Add j5 gripper placeholder
                resp = {
                    "ok": True,
                    "type": "ik",
                    "solutions": [
                        {"j": js, "flags": [0,0,0]}
                    ],
                    "count": 1,
                    "j0": js # First solution
                }
            else:
                resp = {"ok": False, "type":"ik", "error": "Unreachable"}
                
            resp_str = json.dumps(resp)
            sock_mgr.inject_rx_data(resp_str)
            return jsonify({"ok": True})

        # --- OTHER COMMANDS (PASS THROUGH) ---
        sock_mgr.write_json(data)
        
        # Update current_angles if this is a servo command
        if cmd == "servo" and "ch" in data and "deg" in data:
            ch = int(data["ch"])
            deg = float(data["deg"])
            if 0 <= ch <= 5:
                key = f"j{ch}"
                current_angles[key] = deg
        
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.get("/api/read")
def api_read():
    # Optional limit to avoid huge payloads causing UI freeze
    try:
        limit = int(request.args.get("limit", "8192"))
        limit = max(1, min(limit, 65536))
    except Exception:
        limit = 8192
    text = sock_mgr.read_text(max_bytes=limit)
    return jsonify({"ok": True, "data": text})


@app.get("/api/logs")
def api_logs():
    """Get TX/RX logs (structured JSON messages only)"""
    try:
        limit = int(request.args.get("limit", "50"))
        limit = max(1, min(limit, 200))
    except Exception:
        limit = 50
    logs = sock_mgr.get_logs(limit=limit)
    return jsonify({"ok": True, "logs": logs})


@app.get("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.get("/api/default_angles")
def api_get_default_angles():
    """Get default angles"""
    try:
        angles = load_default_angles()
        return jsonify({"ok": True, "angles": angles})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.post("/api/default_angles")
def api_set_default_angles():
    """Save default angles"""
    try:
        data = request.get_json(force=True)
        angles = data.get("angles", {})
        
        # Validate angles
        for key in ["j0", "j1", "j2", "j3", "j4", "j5"]:
            if key not in angles:
                return jsonify({"ok": False, "error": f"Missing {key}"}), 400
        
        # Save to file
        save_default_angles(angles)
        
        return jsonify({"ok": True, "angles": angles})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.get("/api/current_angles")
def api_get_current_angles():
    """Lấy góc hiện tại"""
    global current_angles
    return jsonify({"ok": True, "angles": current_angles})


@app.post("/api/sync_current")
def api_sync_current():
    """Sync current_angles với vị trí hiện tại của robot"""
    global current_angles
    try:
        data = request.get_json(force=True)
        angles = data.get("angles", {})
        
        # Validate
        for key in ["j0", "j1", "j2", "j3", "j4", "j5"]:
            if key not in angles:
                return jsonify({"ok": False, "error": f"Missing {key}"}), 400
        
        # Update current angles without moving servos
        current_angles = {key: float(angles[key]) for key in angles}
        
        return jsonify({"ok": True, "angles": current_angles.copy()})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

@app.post("/api/move_to_position")
def api_move_to_position():
    """
    Tính IK và di chuyển robot đến vị trí (x,y,z)
    Uses server-side Numerical IK for 5-DOF DH model
    """
    global current_angles
    try:
        print(f"\n[MOVE_TO_POSITION] ===== START =====")
        data = request.get_json(force=True)
        position = data.get("position", {})
        validate = data.get("validate", True)
        print(f"[MOVE_TO_POSITION] Target: x={position.get('x')}, y={position.get('y')}, z={position.get('z')}")
        
        # Reset Stop Event at start of new command
        robot_stop_event.clear()
        
        # Validate position input
        required_keys = ["x", "y", "z"]
        for key in required_keys:
            if key not in position:
                return jsonify({"ok": False, "error": f"Missing {key}"}), 400
        
        x, y, z = float(position["x"]), float(position["y"]), float(position["z"])
        phi_deg = float(position.get("b", -60.0)) # Default ideal picking angle
        tol_pos = float(position.get("tol_pos", 10.0))
        tol_phi = float(position.get("tol_phi", 90.0))
        
        # USE NEW IK FUNCTION
        sol_servos = ik_5dof_optimized(x, y, z, phi_deg=phi_deg, tol_pos=tol_pos, tol_phi=tol_phi)
        
        if sol_servos is None:
             return jsonify({
                "ok": False, 
                "error": "Position unreachable (IK Failed)",
                "validation_failed": True
            }), 400
        
        # sol_servos = [sv0, sv1, sv2, sv3, sv4] (Servo Values directly)
        sv0, sv1, sv2, sv3, sv4 = sol_servos
             
        # Build target angles dict (Direct Servo Angles)
        target_angles = {
            "j0": sv0,
            "j1": sv1,
            "j2": sv2,
            "j3": sv3,
            "j4": sv4,
            "j5": current_angles.get("j5", 90) # Gripper unchanged
        }
        
        # Check limits (Should already be safe from IK, but double check file limits)
        limits = load_joint_limits()  # Load limits to validate IK results


        # Move servos - Send target angles to ESP32, ESP32 handles smooth movement
        results = []
        joints_to_move = ["j0", "j1", "j2", "j3", "j4"]
        
        # Store math angles from IK result
        math_angles_from_ik = {
            "j0": unmap_angle("j0", sv0),
            "j1": unmap_angle("j1", sv1),
            "j2": unmap_angle("j2", sv2),
            "j3": unmap_angle("j3", sv3),
            "j4": unmap_angle("j4", sv4),
            "j5": current_angles.get("j5", 90)
        }
        
        for i, key in enumerate(["j0", "j1", "j2", "j3", "j4", "j5"]):
            if key not in joints_to_move:
                continue
            servo_angle = target_angles[key]  # This is servo angle
            math_angle = math_angles_from_ik[key]
            print(f"[MOVE_TO_POSITION] Sending {key}: math={math_angle:.1f}° -> servo={servo_angle:.1f}°")
            cmd = {"cmd": "servo", "ch": i, "deg": servo_angle}
            try:
                sock_mgr.write_json(cmd)
                current_angles[key] = math_angle  # Store math angle
                results.append({"joint": key, "angle": servo_angle, "ok": True})
                time.sleep(0.01)  # Small delay between commands
            except Exception as e:
                results.append({"joint": key, "error": str(e), "ok": False})
        
        print(f"[MOVE_TO_POSITION] ===== END =====\n")
        
        return jsonify({
            "ok": True,
            "position": position,
            "target_angles": target_angles,
            "current_angles": current_angles.copy(),
            "results": results,
            "method": "server-side-analytic-ik-5dof"
        })
        
    except Exception as e:
        import traceback
        print(f"[ERROR] /api/move_to_position exception: {e}")
        traceback.print_exc()
        return jsonify({"ok": False, "error": str(e)}), 500


@app.post("/api/sync_config")
def api_sync_config():
    """Return robot configuration (server-side only)"""
    try:
        from config import d1, a2, a3, d5
        
        cfg_payload = {
            "L_BASE": d1,
            "L_ARM": a2,
            "L_FORE": a3,
            "D_ELBOW": 0,
            "L_WRIST": d5
        }
        
        return jsonify({
            "ok": True, 
            "config": cfg_payload,
            "message": "Config is Server-Side Only"
        })
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.post("/api/sync_config_old")
def api_sync_config_old():
    """Deprecated endpoint - kept for backward compatibility"""
    try:
        return jsonify({
            "ok": True,
            "message": "Deprecated: Server-side only"
        })
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.get("/api/robot_config")
def api_get_robot_config():
    """Get robot configuration"""
    from config import d1, a2, a3, d5
    
    return jsonify({
        "ok": True, 
        "config": {
            "d1": d1,
            "a2": a2, 
            "a3": a3,
            "d5": d5
        }
    })


@app.post("/api/move_to_angles")
def api_move_to_angles():
    """Di chuyển đến góc khớp đã tính từ IK (với validation)"""
    global current_angles
    try:
        print(f"\n[MOVE_TO_ANGLES] ===== CALLED =====\nInput angles: {request.get_json(force=True).get('angles', {})}")
        data = request.get_json(force=True)
        angles = data.get("angles", {})
        
        # Reset Stop Event
        robot_stop_event.clear()
        
        # Validate angles input
        for key in ["j0", "j1", "j2", "j3", "j4", "j5"]:
            if key not in angles:
                return jsonify({"ok": False, "error": f"Missing {key}"}), 400
        
        # Convert DH Angles -> Servo Angles
        target_servo_angles = {}
        for key in angles:
            dh_val = float(angles[key])
            target_servo_angles[key] = map_angle(key, dh_val)

        # Load joint limits for validation
        limits = load_joint_limits()
        
        # Validate Servo Angles against limits
        for key in target_servo_angles:
            angle = target_servo_angles[key]
            if key in limits:
                if angle < limits[key]["min"] or angle > limits[key]["max"]:
                    return jsonify({
                        "ok": False, 
                        "error": f"{key}: {angle:.1f}deg (servo) out of range [{limits[key]['min']}, {limits[key]['max']}]",
                        "validation_failed": True
                    }), 400
        
        # Send target angles to ESP32 - ESP32 handles smooth movement
        results = []
        
        for i, key in enumerate(["j0", "j1", "j2", "j3", "j4", "j5"]):
            servo_angle = target_servo_angles[key]
            math_angle = float(angles[key])  # Original math angle from request
            cmd = {"cmd": "servo", "ch": i, "deg": servo_angle}
            try:
                sock_mgr.write_json(cmd)
                current_angles[key] = math_angle  # Store math angle
                results.append({"joint": key, "angle": servo_angle, "ok": True})
                time.sleep(0.01)  # Small delay between commands
            except Exception as e:
                results.append({"joint": key, "error": str(e), "ok": False})
        
        if robot_stop_event.is_set():
             return jsonify({"ok": False, "error": "Stopped by Emergency Stop", "current_angles": current_angles}), 200
        
        return jsonify({"ok": True, "results": results, "current_angles": current_angles.copy()})
        
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.post("/api/move_all")
def api_move_all():
    """Di chuyển tất cả servo đồng loạt với smooth movement"""
    global current_angles
    try:
        import traceback
        print(f"\n[MOVE_ALL] ===== CALLED =====\nInput angles: {request.get_json(force=True).get('angles', {})}")
        data = request.get_json(force=True)
        angles = data.get("angles", {})
        
        # Reset Stop Event
        robot_stop_event.clear()
        
        # Validate
        for key in ["j0", "j1", "j2", "j3", "j4", "j5"]:
            if key not in angles:
                return jsonify({"ok": False, "error": f"Missing {key}"}), 400
        
        # Convert DH Angles -> Servo Angles
        target_servo_angles = {}
        for key in angles:
            dh_val = float(angles[key])
            target_servo_angles[key] = map_angle(key, dh_val)

        # Load joint limits for validation (ALWAYS reload from file)
        limits = load_joint_limits()
        
        # Validate Servo Angles against limits
        for key in target_servo_angles:
            angle = target_servo_angles[key]
            if key in limits:
                if angle < limits[key]["min"] or angle > limits[key]["max"]:
                    return jsonify({
                        "ok": False, 
                        "error": f"{key}: {angle:.1f}deg (servo) out of range [{limits[key]['min']}, {limits[key]['max']}]"
                    }), 400
        
        results = []
        
        # Send target angles to ESP32 - ESP32 handles smooth movement
        for i, key in enumerate(["j0", "j1", "j2", "j3", "j4", "j5"]):
            servo_angle = target_servo_angles[key]
            math_angle = float(angles[key])  # Original math angle from request
            cmd = {"cmd": "servo", "ch": i, "deg": servo_angle}
            
            try:
                sock_mgr.write_json(cmd)
                current_angles[key] = math_angle  # Store math angle
                results.append({"joint": key, "angle": servo_angle, "ok": True})
                time.sleep(0.01)  # Small delay between commands to avoid overwhelming queue
            except Exception as e:
                results.append({"joint": key, "error": str(e), "ok": False})
        
        if robot_stop_event.is_set():
             return jsonify({"ok": False, "error": "Stopped by Emergency Stop", "current_angles": current_angles}), 200

        return jsonify({"ok": True, "results": results, "current_angles": current_angles.copy()})
    except Exception as e:
        print(f"[ERROR] /api/move_all exception: {e}")
        traceback.print_exc()
        return jsonify({"ok": False, "error": str(e)}), 500


@app.post("/api/home")
def api_home_position():
    """Di chuyển về home position - ESP32 handles smooth movement"""
    global current_angles
    try:
        import traceback
        print(f"\n[HOME] ===== CALLED =====")
        # Load default angles from file (these are MATH angles, need to map to servo angles)
        math_angles = load_default_angles()
        
        # Reset Stop Event
        robot_stop_event.clear()
        
        # Convert math angles to servo angles using map_angle
        target_servo_angles = {}
        for key in ["j0", "j1", "j2", "j3", "j4", "j5"]:
            math_val = float(math_angles.get(key, 90))
            target_servo_angles[key] = map_angle(key, math_val)
        
        print(f"[HOME] Math angles: {math_angles}")
        print(f"[HOME] Servo angles: {target_servo_angles}")
        
        # Send single target command per servo - ESP32 handles smooth interpolation
        for i, key in enumerate(["j0", "j1", "j2", "j3", "j4", "j5"]):
            servo_angle = target_servo_angles[key]
            math_angle = float(math_angles.get(key, 90))  # Math angle from file
            cmd = {"cmd": "servo", "ch": i, "deg": servo_angle}
            try:
                sock_mgr.write_json(cmd)
                current_angles[key] = math_angle  # Store math angle
                time.sleep(0.01)  # Small delay between commands
            except Exception as e:
                print(f"[ERROR] Failed to send servo command: {e}")
                pass
        
        return jsonify({"ok": True, "angles": current_angles.copy(), "smooth": "esp32"})
    except Exception as e:
        print(f"[ERROR] /api/home exception: {e}")
        traceback.print_exc()
        return jsonify({"ok": False, "error": str(e)}), 500


@app.post("/api/rest_position")
def api_rest_position():
    """
    Di chuyển về vị trí nghỉ (REST) với path planning an toàn:
    1. Rotate J0 to Y-axis (90°)
    2. Move J1 → 170° (servo)
    3. Move J2 → 170° (servo)  
    4. Move J3 → 170° (servo)
    5. Open gripper (disable)
    """
    global current_angles
    try:
        print("[REST] Starting path to rest position...")
        robot_stop_event.clear()
        
        cfg = PICK_PLACE_CONFIG
        joint_delay = cfg.get("JOINT_DELAY_MS", 200) / 1000.0
        move_delay = cfg.get("MOVE_DELAY_MS", 500) / 1000.0
        
        rest_servo = cfg["REST_ANGLES_SERVO"]
        j0_rest = cfg.get("J0_REST", 90)
        
        # === Step 1: Rotate J0 to Y-axis (90°) ===
        print(f"[REST] Step 1: Rotating J0 to {j0_rest}°")
        sock_mgr.write_json({"cmd": "servo", "ch": 0, "deg": j0_rest})
        current_angles["j0"] = unmap_angle("j0", j0_rest)
        time.sleep(move_delay)
        
        # Check stop event
        if robot_stop_event.is_set():
            return jsonify({"ok": False, "error": "Stopped by user"}), 200
        
        # === Step 2: Move J1 to 170° ===
        j1_target = rest_servo.get("j1", 170)
        print(f"[REST] Step 2: Moving J1 to {j1_target}°")
        sock_mgr.write_json({"cmd": "servo", "ch": 1, "deg": j1_target})
        current_angles["j1"] = unmap_angle("j1", j1_target)
        time.sleep(move_delay)  # Longer delay for REST
        
        if robot_stop_event.is_set():
            return jsonify({"ok": False, "error": "Stopped by user"}), 200
        
        # === Step 3: Move J2 to 170° ===
        j2_target = rest_servo.get("j2", 170)
        print(f"[REST] Step 3: Moving J2 to {j2_target}°")
        sock_mgr.write_json({"cmd": "servo", "ch": 2, "deg": j2_target})
        current_angles["j2"] = unmap_angle("j2", j2_target)
        time.sleep(move_delay)  # Longer delay for REST
        
        if robot_stop_event.is_set():
            return jsonify({"ok": False, "error": "Stopped by user"}), 200
        
        # === Step 4: Move J3 to 170° ===
        j3_target = rest_servo.get("j3", 170)
        print(f"[REST] Step 4: Moving J3 to {j3_target}°")
        sock_mgr.write_json({"cmd": "servo", "ch": 3, "deg": j3_target})
        current_angles["j3"] = unmap_angle("j3", j3_target)
        time.sleep(move_delay)  # Longer delay for REST
        
        # === Step 5: Open gripper ===
        print(f"[REST] Step 5: Opening gripper")
        sock_mgr.write_json({"cmd": "servo", "ch": 5, "deg": cfg["GRIPPER_OPEN"]})
        current_angles["j5"] = cfg["GRIPPER_OPEN"]
        time.sleep(move_delay)
        
        # === Wait for all servos to settle before disabling ===
        print("[REST] Waiting for servos to settle...")
        time.sleep(1.0)  # 1 second extra wait for all movements to complete
        
        # === Step 6: Disable all servos (power off) ===
        print("[REST] Step 6: Disabling all servos")
        sock_mgr.write_json({"cmd": "servo_off", "all": True})
        time.sleep(0.2)
        
        print("[REST] Rest position reached - Servos OFF!")
        return jsonify({
            "ok": True, 
            "message": "Moved to rest position, servos disabled",
            "servo_angles": {
                "j0": j0_rest,
                "j1": j1_target,
                "j2": j2_target,
                "j3": j3_target
            }
        })
        
    except Exception as e:
        print(f"[ERROR] /api/rest_position exception: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({"ok": False, "error": str(e)}), 500


@app.post("/api/emergency_stop")
def api_emergency_stop():
    """Dừng khẩn cấp - Dừng tất cả servo ngay lập tức"""
    try:
        global current_angles, sock_mgr
        
        print("[EMERGENCY STOP] Triggered!")
        
        # 1. SIGNAL STOP EVENT to break any running loops in other threads
        robot_stop_event.set()
        
        if not sock_mgr._sock:
            return jsonify({"ok": False, "error": "Not connected to ESP32"}), 400
        
        # Get current angles to hold position
        if not current_angles:
            # If no current angles, use safe default position
            current_angles = {
                "j0": 90,
                "j1": 90,
                "j2": 135,
                "j3": 90,
                "j4": 90,
                "j5": 45
            }
        
        # Send immediate stop command - set all servos to current position with 0 delay
        stop_commands = []
        for i in range(6):
            key = f"j{i}"
            angle = current_angles.get(key, 90)
            stop_commands.append({
                "cmd": "servo",
                "ch": i,
                "deg": float(angle)
            })
        
        # Send all stop commands rapidly
        for cmd in stop_commands:
            sock_mgr.write_json(cmd)
            time.sleep(0.01)  # Minimal delay between commands
        
        print(f"[EMERGENCY STOP] All servos stopped at current position: {current_angles}")
        
        return jsonify({
            "ok": True,
            "message": "All servos stopped",
            "stopped_at": current_angles
        })
        
    except Exception as e:
        print(f"[ERROR] /api/emergency_stop exception: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({"ok": False, "error": str(e)}), 500


@app.get("/api/joint_limits")
def api_get_joint_limits():
    """Get joint limits"""
    try:
        limits = load_joint_limits()
        return jsonify({"ok": True, "limits": limits})
    except Exception as e:
        print(f"[ERROR] /api/joint_limits GET exception: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({"ok": False, "error": str(e)}), 500


@app.post("/api/joint_limits")
def api_set_joint_limits():
    """Save joint limits and send to ESP32"""
    try:
        data = request.get_json(force=True)
        limits = data.get("limits", {})
        
        # Validate structure
        for key in ["j0", "j1", "j2", "j3", "j4", "j5"]:
            if key not in limits:
                return jsonify({"ok": False, "error": f"Missing {key}"}), 400
            if "min" not in limits[key] or "max" not in limits[key]:
                return jsonify({"ok": False, "error": f"Missing min/max for {key}"}), 400
        
        # Save to file
        save_joint_limits(limits)
            
        # Send to ESP32
        # Construct payload: {"cmd": "set_limits", "limits": [{"ch": 0, "min_us": ..., "max_us": ...}, ...]}
        esp_limits = []
        for i in range(6):
            key = f"j{i}"
            if key in limits:
                item = {
                    "ch": i,
                    "min_us": int(limits[key].get("min_us", 400)),
                    "max_us": int(limits[key].get("max_us", 2700))
                }
                esp_limits.append(item)
        
        cmd = {
            "cmd": "set_limits",
            "limits": esp_limits
        }
        
        try:
            sock_mgr.write_json(cmd)
            print("[INFO] Sent updated limits to ESP32")
        except Exception as e:
            print(f"[WARNING] Could not send limits to ESP32: {e}")
            # Don't fail the request if just serial is down, but warn user?
            # For now we return OK because file save succeeded, but maybe include warning
            return jsonify({"ok": True, "limits": limits, "warning": "Saved to file but could not send to ESP32 (not connected?)"})
        
        return jsonify({"ok": True, "limits": limits})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.post("/api/servo_off")
def api_servo_off():
    """
    Disable servo PWM output (power off servos)
    
    JSON format:
        {"ch": 0}       - Disable single channel
        {"all": true}   - Disable all channels 0-5
    """
    try:
        data = request.get_json(force=True) or {}
        
        if data.get("all"):
            print("[SERVO-OFF] Disabling all servos")
            sock_mgr.write_json({"cmd": "servo_off", "all": True})
            return jsonify({"ok": True, "message": "All servos disabled"})
        elif "ch" in data:
            ch = int(data["ch"])
            print(f"[SERVO-OFF] Disabling servo CH{ch}")
            sock_mgr.write_json({"cmd": "servo_off", "ch": ch})
            return jsonify({"ok": True, "message": f"Servo CH{ch} disabled", "ch": ch})
        else:
            return jsonify({"ok": False, "error": "ch or all required"}), 400
            
    except Exception as e:
        print(f"[ERROR] /api/servo_off exception: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500


# ===================== Pick and Place APIs =====================
import math

@app.post("/api/gripper")
def api_gripper():
    """Control gripper: open or close"""
    global current_angles
    try:
        data = request.get_json(force=True)
        action = data.get("action", "open")  # "open" or "close"
        
        if action == "open":
            angle = PICK_PLACE_CONFIG["GRIPPER_OPEN"]
        elif action == "close":
            angle = PICK_PLACE_CONFIG["GRIPPER_CLOSED"]
        else:
            angle = float(data.get("angle", 90))
        
        cmd = {"cmd": "servo", "ch": 5, "deg": angle}
        sock_mgr.write_json(cmd)
        current_angles["j5"] = angle
        
        wait_ms = PICK_PLACE_CONFIG["GRIPPER_WAIT_MS"] if action == "close" else 300
        time.sleep(wait_ms / 1000.0)
        
        return jsonify({"ok": True, "action": action, "angle": angle})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

@app.post("/api/calculate_object")
def api_calculate_object():
    """
    Calculate arm target coordinates from car position and object position.
    
    Input JSON:
    {
        "car_x": float,      # Car X position in world coordinates
        "car_y": float,      # Car Y position in world coordinates
        "obj_x": float,      # Object X position in world coordinates
        "obj_y": float,      # Object Y position in world coordinates
        "obj_z": float       # Object Z position (height, usually negative)
    }
    
    Returns:
    {
        "ok": true,
        "arm_target": {
            "x": float,      # Arm target X in arm-local coordinates
            "y": float,      # Arm target Y in arm-local coordinates
            "z": float       # Arm target Z (same as obj_z)
        },
        "reachable": bool    # Whether the position is reachable by IK
    }
    """
    try:
        data = request.get_json(force=True)
        cfg = PICK_PLACE_CONFIG
        
        car_x = float(data.get("car_x", 0))
        car_y = float(data.get("car_y", 0))
        obj_x = float(data.get("obj_x", 0))
        obj_y = float(data.get("obj_y", 0))
        obj_z = float(data.get("obj_z", -150))  # Default pick height
        
        # Arm base position in world coordinates
        arm_offset_y = cfg.get("ARM_OFFSET_Y", 70)
        arm_base_x = car_x
        arm_base_y = car_y + arm_offset_y
        
        # Object position relative to arm base (arm-local coordinates)
        arm_target_x = obj_x - arm_base_x
        arm_target_y = obj_y - arm_base_y
        arm_target_z = obj_z
        
        print(f"[CALC] Car=({car_x}, {car_y}), Obj=({obj_x}, {obj_y}, {obj_z})")
        print(f"[CALC] Arm base=({arm_base_x}, {arm_base_y})")
        print(f"[CALC] Arm target=({arm_target_x:.1f}, {arm_target_y:.1f}, {arm_target_z:.1f})")
        
        # Check if position is reachable
        pitch = PICK_PLACE_CONFIG["PITCH"]
        sol = ik_5dof_optimized(arm_target_x, arm_target_y, arm_target_z, phi_deg=pitch)
        reachable = sol is not None
        
        return jsonify({
            "ok": True,
            "arm_target": {
                "x": round(arm_target_x, 1),
                "y": round(arm_target_y, 1),
                "z": round(arm_target_z, 1)
            },
            "reachable": reachable,
            "car_position": {"x": car_x, "y": car_y},
            "arm_base": {"x": arm_base_x, "y": arm_base_y}
        })
        
    except Exception as e:
        print(f"[CALC] Error: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({"ok": False, "error": str(e)}), 500

@app.post("/api/pick")
def api_pick():
    """
    Pick object at position (x, y, z).
    Path Planning:
    1. Open gripper
    2. Rotate J0 to target direction
    3. Move J1 + J3 together (shoulder + wrist for stable orientation)
    4. Move J2 to target (elbow)
    5. Close gripper
    6. Lift up
    """
    global current_angles
    try:
        data = request.get_json(force=True)
        x = float(data.get("x", 0))
        y = float(data.get("y", 0))
        z = float(data.get("z", -150))
        
        print(f"[PICK] Starting pick at ({x}, {y}, {z})")
        robot_stop_event.clear()
        
        cfg = PICK_PLACE_CONFIG
        pitch = cfg["PITCH"]
        joint_delay = cfg.get("JOINT_DELAY_MS", 500) / 1000.0
        move_delay = cfg.get("MOVE_DELAY_MS", 1000) / 1000.0

        # Calculate IK for target position
        sol_target = ik_5dof_optimized(x, y, z, phi_deg=pitch)
        if sol_target is None:
            return jsonify({"ok": False, "error": "Target position unreachable (IK failed)"}), 400
        
        # Calculate IK for Approach Position (Hover above object)
        target_approach_height = cfg.get("APPROACH_HEIGHT", 40)
        sol_approach = None
        final_approach_z = z
        
        # Try to find a valid approach height, decreasing from max approach height down to 0
        for h in range(target_approach_height, 0, -5):
            test_z = z + h
            sol = ik_5dof_optimized(x, y, test_z, phi_deg=pitch)
            if sol:
                sol_approach = sol
                final_approach_z = test_z
                print(f"[PICK] Found valid approach at z={test_z} (height={h})")
                break
        
        if sol_approach is None:
            print(f"[PICK] Warning: No valid approach found above z={z}, moving directly to target.")
            sol_approach = sol_target
            final_approach_z = z
        
        sv0_a, sv1_a, sv2_a, sv3_a, sv4_a = sol_approach
        sv0, sv1, sv2, sv3, sv4 = sol_target
        
        print(f"[PICK] IK Approach: J0={sv0_a:.1f}, J1={sv1_a:.1f}, J2={sv2_a:.1f}, J3={sv3_a:.1f}")
        print(f"[PICK] IK Target:   J0={sv0:.1f}, J1={sv1:.1f}, J2={sv2:.1f}, J3={sv3:.1f}")
        
        # === Step 1: Open gripper ===
        print("[PICK] Step 1: Opening gripper")
        sock_mgr.write_json({"cmd": "servo", "ch": 5, "deg": cfg["GRIPPER_OPEN"]})
        current_angles["j5"] = cfg["GRIPPER_OPEN"]
        time.sleep(0.3)
        
        if robot_stop_event.is_set():
            return jsonify({"ok": False, "error": "Stopped by user"}), 200
        
        # === Step 2: Move to APPROACH Position (Hover) ===
        print(f"[PICK] Step 2: Approaching z={final_approach_z}...")
        
        # 2a. Rotate J0
        sock_mgr.write_json({"cmd": "servo", "ch": 0, "deg": sv0_a})
        current_angles["j0"] = unmap_angle("j0", sv0_a)
        time.sleep(move_delay)
        
        # Send gravity angles for APPROACH position
        send_gravity_angles(sv0_a, sv1_a, sv2_a, sv3_a)
        
        # 2b. Move J1+J2+J3 (Elbow + Wrist)
        sock_mgr.write_json({"cmd": "servo", "ch": [1, 2, 3], "deg": [sv1_a, sv2_a, sv3_a]})
        current_angles["j1"] = unmap_angle("j1", sv1_a)
        current_angles["j2"] = unmap_angle("j2", sv2_a)
        current_angles["j3"] = unmap_angle("j3", sv3_a)
        time.sleep(joint_delay)

        
        if robot_stop_event.is_set():
            return jsonify({"ok": False, "error": "Stopped by user"})
        
        # === Step 2.5: Rotate J4 (Gripper Roll) based on X offset ===
        # x = 0 → j4 = 90° (perpendicular to X-axis)
        # x ≠ 0 → rotate proportionally to align with object
        import math
        if y != 0:
            # Calculate angle from Y-axis: atan2(x, y) gives angle from +Y
            angle_from_y = math.degrees(math.atan2(x, y))
            j4_target = 90.0 - angle_from_y  # Adjust J4 relative to perpendicular
        else:
            # Edge case: y = 0, object directly on X-axis
            j4_target = 0.0 if x > 0 else 180.0 if x < 0 else 90.0
        
        # Clamp J4 to valid servo range
        j4_target = max(0.0, min(180.0, j4_target))
        print(f"[PICK] Step 2.5: Rotating J4 to {j4_target:.1f}° (based on x={x})")
        sock_mgr.write_json({"cmd": "servo", "ch": 4, "deg": j4_target})
        current_angles["j4"] = j4_target
        time.sleep(0.2)
            
        # === Step 3: Descend to TARGET Position ===
        print(f"[PICK] Step 3: Descending to z={z}...")
        # Send gravity angles for TARGET position
        send_gravity_angles(sv0, sv1, sv2, sv3)

        # Move all joints to target (usually just J1/J2/J3 changes slightly)
        # Move J2+J3 first concurrently
        sock_mgr.write_json({"cmd": "servo", "ch": [1, 2, 3], "deg": [sv1, sv2, sv3]})
        current_angles["j1"] = unmap_angle("j1", sv1)
        current_angles["j2"] = unmap_angle("j2", sv2)
        current_angles["j3"] = unmap_angle("j3", sv3)
        time.sleep(joint_delay)  # Use joint_delay instead of hardcoded 5s
        
        if robot_stop_event.is_set():
            return jsonify({"ok": False, "error": "Stopped by user"}), 200
        
        # === Step 4: Close gripper ===
        print("[PICK] Step 4: Closing gripper")
        sock_mgr.write_json({"cmd": "servo", "ch": 5, "deg": cfg["GRIPPER_CLOSED"]})
        current_angles["j5"] = cfg["GRIPPER_CLOSED"]
        time.sleep(cfg["GRIPPER_WAIT_MS"] / 1000.0)
        time.sleep(2)
        # === Step 5: Lift up (if reachable) ===    
        # Send gravity angles for lifted position
        send_gravity_angles(sv0_a, sv1_a, sv2_a, sv3_a)
        
        # Move J2+J3 concurrently, then J1
        sock_mgr.write_json({"cmd": "servo", "ch": [1, 2, 3], "deg": [sv1_a, sv2_a, sv3_a]})
        current_angles["j1"] = unmap_angle("j1", sv1_a)
        current_angles["j2"] = unmap_angle("j2", sv2_a)
        current_angles["j3"] = unmap_angle("j3", sv3_a)
        time.sleep(joint_delay)
        

        print("[PICK] Pick complete!")
        return jsonify({
            "ok": True,
            "action": "pick_complete",
            "position": {"x": x, "y": y, "z": z},
        })
        
    except Exception as e:
        print(f"[PICK] Error: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({"ok": False, "error": str(e)}), 500


@app.post("/api/place")
def api_place():
    """
    Place object (thả vật).
    Path Planning:
    1. Lower down by LIFT_HEIGHT (same as pick lift)
    2. Open gripper (release object)
    3. Return to REST position (J0→90°, J1→J2→J3→170°)
    
    Note: Assumes robot is already at picked position (after pick operation)
    The target (x, y, z) from request is used to calculate lowering position.
    """
    global current_angles
    try:
        data = request.get_json(force=True)
        x = float(data.get("x", 0))
        y = float(data.get("y", 0))
        z = float(data.get("z", -150))
        
        print(f"[PLACE] Starting place at ({x}, {y}, {z})")
        robot_stop_event.clear()
        
        cfg = PICK_PLACE_CONFIG
        pitch = cfg["PITCH"]
        joint_delay = cfg.get("JOINT_DELAY_MS", 200) / 1000.0
        move_delay = cfg.get("MOVE_DELAY_MS", 300) / 1000.0
        lift_height = cfg.get("LIFT_HEIGHT", 10)

        # Calculate IK for place position (lowered by LIFT_HEIGHT)
        place_z = z  # Target Z is the actual place position
        sol = ik_5dof_optimized(x, y, place_z, phi_deg=pitch)
        if sol is None:
            return jsonify({"ok": False, "error": "Place position unreachable (IK failed)"}), 400
        
        sv0, sv1, sv2, sv3, sv4 = sol
        print(f"[PLACE] IK Solution: J0={sv0:.1f}, J1={sv1:.1f}, J2={sv2:.1f}, J3={sv3:.1f}")
        
        # === Step 1: Lower to place position ===
        print(f"[PLACE] Step 1: Lowering to z={place_z}")
        
        # Move J1, J2, J3 sequentially to lower
        sock_mgr.write_json({"cmd": "servo", "ch": 1, "deg": sv1})
        current_angles["j1"] = unmap_angle("j1", sv1)
        time.sleep(joint_delay)
        
        if robot_stop_event.is_set():
            return jsonify({"ok": False, "error": "Stopped by user"}), 200
        
        sock_mgr.write_json({"cmd": "servo", "ch": 2, "deg": sv2})
        current_angles["j2"] = unmap_angle("j2", sv2)
        time.sleep(joint_delay)
        
        if robot_stop_event.is_set():
            return jsonify({"ok": False, "error": "Stopped by user"}), 200
        
        sock_mgr.write_json({"cmd": "servo", "ch": 3, "deg": sv3})
        current_angles["j3"] = unmap_angle("j3", sv3)
        time.sleep(joint_delay)
        
        if robot_stop_event.is_set():
            return jsonify({"ok": False, "error": "Stopped by user"}), 200
        
        # === Step 2: Open gripper (release object) ===
        print("[PLACE] Step 2: Opening gripper")
        sock_mgr.write_json({"cmd": "servo", "ch": 5, "deg": cfg["GRIPPER_OPEN"]})
        current_angles["j5"] = cfg["GRIPPER_OPEN"]
        time.sleep(0.3)
        
        # === Step 3: Return to REST position ===
        print("[PLACE] Step 3: Returning to REST position...")
        
        rest_servo = cfg["REST_ANGLES_SERVO"]
        j0_rest = cfg.get("J0_REST", 90)
        
        # Step 3a: Rotate J0 to Y-axis (90°)
        print(f"[PLACE] Step 3a: Rotating J0 to {j0_rest}°")
        sock_mgr.write_json({"cmd": "servo", "ch": 0, "deg": j0_rest})
        current_angles["j0"] = unmap_angle("j0", j0_rest)
        time.sleep(move_delay)
        
        if robot_stop_event.is_set():
            return jsonify({"ok": False, "error": "Stopped by user"}), 200
        
        # Step 3b: Move J1 to 170°
        j1_target = rest_servo.get("j1", 170)
        print(f"[PLACE] Step 3b: Moving J1 to {j1_target}°")
        sock_mgr.write_json({"cmd": "servo", "ch": 1, "deg": j1_target})
        current_angles["j1"] = unmap_angle("j1", j1_target)
        time.sleep(joint_delay)
        
        if robot_stop_event.is_set():
            return jsonify({"ok": False, "error": "Stopped by user"}), 200
        
        # Step 3c: Move J2 to 170°
        j2_target = rest_servo.get("j2", 170)
        print(f"[PLACE] Step 3c: Moving J2 to {j2_target}°")
        sock_mgr.write_json({"cmd": "servo", "ch": 2, "deg": j2_target})
        current_angles["j2"] = unmap_angle("j2", j2_target)
        time.sleep(joint_delay)
        
        if robot_stop_event.is_set():
            return jsonify({"ok": False, "error": "Stopped by user"}), 200
        
        # Step 3d: Move J3 to 170°
        j3_target = rest_servo.get("j3", 170)
        print(f"[PLACE] Step 3d: Moving J3 to {j3_target}°")
        sock_mgr.write_json({"cmd": "servo", "ch": 3, "deg": j3_target})
        current_angles["j3"] = unmap_angle("j3", j3_target)
        time.sleep(joint_delay)
        
        print("[PLACE] Place complete!")
        return jsonify({
            "ok": True,
            "action": "place_complete",
            "position": {"x": x, "y": y, "z": z}
        })
        
    except Exception as e:
        print(f"[PLACE] Error: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({"ok": False, "error": str(e)}), 500


if __name__ == "__main__":
    # Graceful cleanup on process exit/signals
    def _cleanup():
        try:
            sock_mgr.stop_server()
        except Exception:
            pass

    atexit.register(_cleanup)

    def _handle_signal(sig, frame):
        _cleanup()
        try:
            # Flask's reloader spawns child; exiting fast avoids hangs
            os._exit(0)
        except Exception:
            pass

    try:
        signal.signal(signal.SIGINT, _handle_signal)
        signal.signal(signal.SIGTERM, _handle_signal)
    except Exception:
        # On some platforms, not all signals are available
        pass

    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "5000"))
    print(f"\n{'='*60}")
    print(f"Flask server starting on http://{host}:{port}")
    print(f"Auto-reload: DISABLED (code changes require manual restart)")
    print(f"{'='*60}\n")
    app.run(host=host, port=port, debug=False, use_reloader=False)

