"""
TCP Socket Manager Module
Manages TCP server for ESP32 communication
"""

import io
import json
import time
import threading
import socket
from typing import Optional, List, Dict


class TCPSocketManager:
    """TCP Socket Manager for ESP32 communication"""
    
    def __init__(self):
        self._lock = threading.Lock()
        self._server_sock: Optional[socket.socket] = None
        self._client_sock: Optional[socket.socket] = None
        self._rx_buffer = io.BytesIO()
        self._reader_thread: Optional[threading.Thread] = None
        self._server_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._tx_log = []  # Log of sent messages
        self._rx_log = []  # Log of received messages
        self._max_log_size = 100  # Keep last 100 messages
        self._last_tx_time = 0.0  # Rate limiting timestamp
        self._min_tx_interval = 0.015  # Minimum 15ms between commands
        self._connection_alive = False  # Track connection health
        self._last_read_time = 0.0  # Track last successful read
        self._is_server_running = False
        self._port = 8888
        self._client_address = None

    def list_ports(self) -> List[Dict[str, str]]:
        """Return TCP server status"""
        status = "Running" if self._is_server_running else "Stopped"
        client = f"Client: {self._client_address}" if self._client_address else "No client"
        return [{
            "device": "TCP Server",
            "name": f"Status: {status}",
            "description": f"Listening on 0.0.0.0:{self._port} | {client}",
            "hwid": "TCP"
        }]
    
    def start_server(self, port: int = 8888) -> None:
        """Start TCP server to listen for ESP32 connections"""
        with self._lock:
            if self._is_server_running:
                print(f"[TCP-SERVER] Already running on port {self._port}")
                return
            
            self._port = port
            self._stop_event.clear()
            self._is_server_running = True
            
        # Start server thread
        self._server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self._server_thread.start()
        print(f"[TCP-SERVER] Started on 0.0.0.0:{port}")
    
    def _server_loop(self) -> None:
        """Main server loop - accept connections"""
        try:
            # Create server socket
            self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server_sock.bind(('0.0.0.0', self._port))
            self._server_sock.listen(1)
            self._server_sock.settimeout(1.0)  # Timeout to check stop event
            
            print(f"[TCP-SERVER] Listening on 0.0.0.0:{self._port}")
            
            while not self._stop_event.is_set():
                try:
                    client_sock, client_addr = self._server_sock.accept()
                    print(f"[TCP-SERVER] Client connected from {client_addr}")
                    
                    with self._lock:
                        self._client_sock = client_sock
                        self._client_address = f"{client_addr[0]}:{client_addr[1]}"
                        self._connection_alive = True
                        self._last_read_time = time.time()
                    
                    # Start reader thread for this client
                    if self._reader_thread and self._reader_thread.is_alive():
                        self._reader_thread.join(timeout=1.0)
                    
                    self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
                    self._reader_thread.start()
                    
                    # Auto-send servo_init after 1s delay to ensure stability
                    threading.Timer(1.0, self.send_init, args=[50]).start()
                    
                    # Wait for client to disconnect
                    while not self._stop_event.is_set() and self._connection_alive:
                        time.sleep(0.1)
                    
                    # Client disconnected, cleanup
                    with self._lock:
                        if self._client_sock:
                            try:
                                self._client_sock.close()
                            except:
                                pass
                            self._client_sock = None
                        self._client_address = None
                    
                    print(f"[TCP-SERVER] Client disconnected")
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    if not self._stop_event.is_set():
                        print(f"[TCP-SERVER] Accept error: {e}")
                    time.sleep(0.1)
                    
        except Exception as e:
            print(f"[TCP-SERVER] Server error: {e}")
        finally:
            if self._server_sock:
                try:
                    self._server_sock.close()
                except:
                    pass
            with self._lock:
                self._is_server_running = False
            print("[TCP-SERVER] Stopped")

    def stop_server(self) -> None:
        """Stop TCP server"""
        with self._lock:
            if not self._is_server_running:
                return
            
        self._stop_event.set()
        self._connection_alive = False
        
        # Close client connection
        if self._client_sock:
            try:
                self._client_sock.close()
            except:
                pass
        
        # Wait for threads to stop
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        if self._server_thread and self._server_thread.is_alive():
            self._server_thread.join(timeout=2.0)
        
        with self._lock:
            self._client_sock = None
            self._server_sock = None
            self._client_address = None
            self._is_server_running = False
            self._rx_buffer = io.BytesIO()
            self._tx_log.clear()
            self._rx_log.clear()
        
        print("[TCP-SERVER] Stopped")

    def _reader_loop(self) -> None:
        """Read data from connected client"""
        line_buffer = b""
        consecutive_errors = 0
        max_consecutive_errors = 10
        
        while not self._stop_event.is_set() and self._connection_alive:
            try:
                if not self._client_sock:
                    break
                
                try:
                    chunk = self._client_sock.recv(4096)
                except socket.timeout:
                    time.sleep(0.01)
                    continue
                except BlockingIOError:
                    time.sleep(0.01)
                    continue
                
                if not chunk:
                    print("[TCP-SERVER] Client disconnected")
                    self._connection_alive = False
                    break
                
                consecutive_errors = 0
                self._last_read_time = time.time()
                
                line_buffer += chunk
                
                while b'\n' in line_buffer:
                    line, line_buffer = line_buffer.split(b'\n', 1)
                    line_str = line.decode('utf-8', errors='replace').strip()
                    
                    if line_str:
                        with self._lock:
                            self._rx_log.append({
                                'time': time.strftime('%H:%M:%S'),
                                'dir': 'rx',
                                'data': line_str
                            })
                            if len(self._rx_log) > self._max_log_size:
                                self._rx_log.pop(0)
                            
                            pos = self._rx_buffer.tell()
                            self._rx_buffer.seek(0, io.SEEK_END)
                            self._rx_buffer.write((line_str + '\n').encode('utf-8'))
                            self._rx_buffer.seek(pos)
                    
            except Exception as e:
                consecutive_errors += 1
                print(f"[TCP-SERVER] Reader error ({consecutive_errors}/{max_consecutive_errors}): {e}")
                if consecutive_errors >= max_consecutive_errors:
                    print("[TCP-SERVER] Too many errors, closing connection")
                    self._connection_alive = False
                    break
                time.sleep(0.1)
        
        print("[TCP-SERVER] Reader thread exiting")
        with self._lock:
            self._connection_alive = False

    def inject_rx_data(self, data_str: str) -> None:
        """Inject simulated data into the RX pipeline"""
        if not data_str.endswith('\n'):
            data_str += '\n'
            
        with self._lock:
            self._rx_log.append({
                'time': time.strftime('%H:%M:%S'),
                'dir': 'rx',
                'data': data_str.strip()
            })
            if len(self._rx_log) > self._max_log_size:
                self._rx_log.pop(0)
            
            pos = self._rx_buffer.tell()
            self._rx_buffer.seek(0, io.SEEK_END)
            self._rx_buffer.write(data_str.encode("utf-8"))
            self._rx_buffer.seek(pos)
            self._last_read_time = time.time()

    def send_init(self, freq: int = 50) -> None:
        """Send servo_init command to ESP32"""
        cmd = {
            "cmd": "servo_init",
            "freq": freq
        }
        self.write_json(cmd)
        print(f"[TCP-SERVER] Sent servo_init (freq={freq}Hz)")

    def write_json(self, obj: dict) -> None:
        """Send JSON command to ESP32"""
        data = json.dumps(obj).encode("utf-8") + b"\n"
        with self._lock:
            if not self._client_sock:
                raise RuntimeError("No client connected")
            
            if not self._connection_alive:
                raise RuntimeError("Connection not alive")
            
            # Rate limiting
            now = time.time()
            elapsed = now - self._last_tx_time
            if elapsed < self._min_tx_interval:
                time.sleep(self._min_tx_interval - elapsed)
            
            try:
                self._client_sock.sendall(data)
                self._last_tx_time = time.time()
                
                # Log with console output for debugging
                print(f"[TCP-TX] Sending: {data.decode('utf-8').strip()}")
                
                self._tx_log.append({
                    'time': time.strftime('%H:%M:%S'),
                    'dir': 'tx',
                    'data': data.decode('utf-8').strip()
                })
                if len(self._tx_log) > self._max_log_size:
                    self._tx_log.pop(0)
                    
            except socket.error as e:
                print(f"[TCP-SERVER] Send error: {e}")
                self._connection_alive = False
                raise RuntimeError(f"TCP send failed: {e}")
            except Exception as e:
                print(f"[TCP-SERVER] Unexpected error: {e}")
                raise

    def read_text(self, max_bytes: int = 65536) -> str:
        """Read text from RX buffer"""
        with self._lock:
            self._rx_buffer.seek(0)
            data = self._rx_buffer.read(max_bytes)
            rest = self._rx_buffer.read()
            self._rx_buffer = io.BytesIO()
            self._rx_buffer.write(rest)
            return data.decode("utf-8", errors="replace")
    
    def get_logs(self, limit: int = 50) -> List[Dict]:
        """Get combined TX/RX logs sorted by time"""
        with self._lock:
            all_logs = self._tx_log + self._rx_log
            return all_logs[-limit:]
    
    def get_connection_status(self) -> Dict:
        """Get detailed connection status"""
        with self._lock:
            return {
                "connected": self._connection_alive,
                "server_running": self._is_server_running,
                "port": self._port,
                "client": self._client_address,
                "alive": self._connection_alive,
                "last_read_seconds_ago": round(time.time() - self._last_read_time, 1) if self._last_read_time > 0 else 0,
                "message": "Server running" if self._is_server_running else "Server stopped"
            }
