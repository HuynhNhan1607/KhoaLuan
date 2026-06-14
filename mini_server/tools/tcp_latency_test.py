#!/usr/bin/env python3
import socket
import json
import time
import argparse
import sys
import select

# ANSI Colors
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BLUE = "\033[94m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"

def main():
    parser = argparse.ArgumentParser(description="Measure TCP latency between this machine and the mini_server.")
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="Target IP address of the mini_server")
    parser.add_argument("--port", type=int, default=2004, help="Target Port (e.g. 2004 for laptop, 8080 for ESP32)")
    parser.add_argument("--interval", type=float, default=1.0, help="Interval between pings in seconds (default: 1.0)")
    parser.add_argument("--count", type=int, default=0, help="Number of pings to send (default: 0 = infinite)")
    parser.add_argument("--timeout", type=float, default=2.0, help="Wait timeout for pong response in seconds (default: 2.0)")
    args = parser.parse_args()

    print(f"{BOLD}{BLUE}=================================================={RESET}")
    print(f"{BOLD}{BLUE}           TCP LATENCY MEASUREMENT TOOL           {RESET}")
    print(f"{BOLD}{BLUE}=================================================={RESET}")
    print(f"Connecting to: {BOLD}{args.ip}:{args.port}{RESET}")
    print(f"Interval:      {args.interval}s")
    print(f"Timeout:       {args.timeout}s")
    print(f"Press {BOLD}Ctrl+C{RESET} to stop and see statistics.\n")

    # Connect to the server
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(args.timeout)
        sock.connect((args.ip, args.port))
        print(f"{GREEN}✓ Connected successfully!{RESET}\n")
    except Exception as e:
        print(f"{RED}✗ Connection failed: {e}{RESET}")
        sys.exit(1)

    rtts = []
    lost_packets = 0
    seq = 0

    try:
        while True:
            if args.count > 0 and seq >= args.count:
                break

            seq += 1
            send_ts = time.time()
            
            # Construct the ping message
            ping_msg = {
                "type": "ping",
                "data": {
                    "ts": send_ts
                }
            }
            
            payload = json.dumps(ping_msg) + "\n"
            
            # Send ping
            try:
                sock.sendall(payload.encode('utf-8'))
            except Exception as e:
                print(f"{RED}[Seq {seq}] Send failed: {e}{RESET}")
                lost_packets += 1
                time.sleep(args.interval)
                continue

            # Wait for response
            recv_buffer = ""
            pong_received = False
            start_wait = time.time()
            
            while time.time() - start_wait < args.timeout:
                # Use select to check if socket is readable
                ready = select.select([sock], [], [], 0.1)
                if ready[0]:
                    try:
                        chunk = sock.recv(1024).decode('utf-8')
                        if not chunk:
                            print(f"{RED}[Seq {seq}] Connection closed by server.{RESET}")
                            sys.exit(1)
                        recv_buffer += chunk
                        
                        # Process received line(s)
                        if "\n" in recv_buffer:
                            lines = recv_buffer.split("\n")
                            # Keep the last incomplete part in buffer
                            recv_buffer = lines[-1]
                            
                            for line in lines[:-1]:
                                line = line.strip()
                                if not line:
                                    continue
                                try:
                                    resp = json.loads(line)
                                    if resp.get("type") == "pong":
                                        resp_ts = resp.get("data", {}).get("ts", 0.0)
                                        # Verify if this matches current ping time (or handle in order)
                                        if abs(resp_ts - send_ts) < args.timeout * 2:
                                            recv_ts = time.time()
                                            rtt = (recv_ts - send_ts) * 1000.0  # in ms
                                            rtts.append(rtt)
                                            print(f"Reply from {args.ip}: seq={seq} rtt={rtt:.2f} ms")
                                            pong_received = True
                                            break
                                except json.JSONDecodeError:
                                    # Ignore non-JSON or other message traffic from server
                                    pass
                            
                            if pong_received:
                                break
                    except socket.timeout:
                        break
                    except Exception as e:
                        print(f"{RED}[Seq {seq}] Recv failed: {e}{RESET}")
                        break
            
            if not pong_received:
                lost_packets += 1
                print(f"{RED}[Seq {seq}] Request timed out.{RESET}")

            # Wait for next interval
            elapsed = time.time() - send_ts
            if elapsed < args.interval:
                time.sleep(args.interval - elapsed)

    except KeyboardInterrupt:
        print("\nMeasurement stopped by user.")
    finally:
        sock.close()

    # Print Summary Statistics
    print(f"\n{BOLD}{BLUE}--- TCP Latency Statistics for {args.ip}:{args.port} ---{RESET}")
    total_sent = seq
    total_recv = len(rtts)
    
    if total_sent > 0:
        loss_pct = (lost_packets / total_sent) * 100.0
    else:
        loss_pct = 0.0

    print(f"Packets: Sent = {total_sent}, Received = {total_recv}, Lost = {lost_packets} ({loss_pct:.1f}% loss)")
    
    if rtts:
        min_rtt = min(rtts)
        max_rtt = max(rtts)
        avg_rtt = sum(rtts) / len(rtts)
        
        # Calculate Jitter (average difference between consecutive RTTs)
        if len(rtts) > 1:
            jitter = sum(abs(rtts[i] - rtts[i-1]) for i in range(1, len(rtts))) / (len(rtts) - 1)
        else:
            jitter = 0.0
            
        print(f"Round-trip times: min/avg/max/jitter = {min_rtt:.2f}/{avg_rtt:.2f}/{max_rtt:.2f}/{jitter:.2f} ms")
    else:
        print(f"{RED}No responses received. Cannot calculate RTT statistics.{RESET}")
    print(f"{BOLD}{BLUE}-------------------------------------------------------{RESET}")

if __name__ == "__main__":
    main()
