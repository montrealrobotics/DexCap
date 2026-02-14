#!/usr/bin/env python3
"""
Simple test script to check if UDP packets are being received from Rokoko Studio.
This helps diagnose Docker networking issues.
"""

import socket
import sys
import time
import json
from ip_config import ROKOKO_PORT, VR_HOST, HAND_INFO_PORT

def get_local_ip():
    """Get the local IP address"""
    try:
        # Connect to a remote address to determine local IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception as e:
        return "unknown"

def test_rokoko_listener():
    """Test if we can receive UDP packets on the Rokoko port"""
    print("=" * 60)
    print("Rokoko UDP Connection Test")
    print("=" * 60)
    print(f"Target port: {ROKOKO_PORT}")
    print(f"VR Host: {VR_HOST}")
    print(f"Hand Info Port: {HAND_INFO_PORT}")
    print(f"Local IP: {get_local_ip()}")
    print("=" * 60)
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Set socket options similar to RokokoModule
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 0)
    
    # Set a timeout so we don't hang forever
    sock.settimeout(5.0)  # 5 second timeout
    
    try:
        # Bind to the port (listening on all interfaces)
        sock.bind(("", ROKOKO_PORT))
        print(f"✓ Successfully bound to 0.0.0.0:{ROKOKO_PORT}")
        
        # Get socket info
        try:
            rcvbuf = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
            print(f"✓ Socket receive buffer size: {rcvbuf}")
        except:
            print("⚠ Could not get socket buffer size")
        
        print("\n" + "=" * 60)
        print("Waiting for UDP packets from Rokoko Studio...")
        print("Make sure Rokoko Studio is running and streaming to this port!")
        print("Press Ctrl+C to cancel")
        print("=" * 60 + "\n")
        
        packet_count = 0
        start_time = time.time()
        
        while True:
            try:
                # Try to receive data
                data, addr = sock.recvfrom(40000)
                packet_count += 1
                elapsed = time.time() - start_time
                
                print(f"[Packet #{packet_count}] Received {len(data)} bytes from {addr[0]}:{addr[1]}")
                print(f"  Time since start: {elapsed:.2f}s")
                print(f"  Packet rate: {packet_count/elapsed:.2f} packets/sec")
                
                # Try to decode as JSON (Rokoko sends JSON)
                try:
                    json_data = json.loads(data.decode())
                    print(f"  ✓ Valid JSON received")
                    # Print a sample of the structure
                    if "scene" in json_data:
                        print(f"  ✓ Contains 'scene' key")
                        if "actors" in json_data["scene"]:
                            print(f"  ✓ Contains 'actors' key")
                            print(f"  Number of actors: {len(json_data['scene']['actors'])}")
                except json.JSONDecodeError as e:
                    print(f"  ⚠ Not valid JSON: {e}")
                except Exception as e:
                    print(f"  ⚠ Decode error: {e}")
                
                print()
                
            except socket.timeout:
                print(f"\n⚠ Timeout: No data received in 5 seconds")
                print(f"  Total packets received: {packet_count}")
                if packet_count == 0:
                    print("\n" + "=" * 60)
                    print("TROUBLESHOOTING TIPS:")
                    print("=" * 60)
                    print("1. Check if Rokoko Studio is running and streaming")
                    print("2. Verify Rokoko Studio is configured to send to:")
                    print(f"   - IP: {get_local_ip()} (or your Docker host IP)")
                    print(f"   - Port: {ROKOKO_PORT}")
                    print("3. If running in Docker:")
                    print(f"   - Port {ROKOKO_PORT} should be exposed (check docker-compose.yml)")
                    print(f"   - Rokoko Studio should send to Docker HOST IP: {get_local_ip()}")
                    print("   - NOT the container's internal IP")
                    print("   - Try 'host' network mode: network_mode: host")
                    print("   - Or ensure UDP packets are reaching the host")
                    print("4. Check firewall settings")
                    print("5. Verify network connectivity: ping the Rokoko source IP")
                    print("=" * 60)
                break
                
    except OSError as e:
        print(f"\n✗ ERROR: Could not bind to port {ROKOKO_PORT}")
        print(f"  Error: {e}")
        print("\nTroubleshooting:")
        print("  - Port might already be in use by another process")
        print("  - Try: netstat -an | findstr {ROKOKO_PORT} (Windows)")
        print("  - Or: netstat -an | grep {ROKOKO_PORT} (Linux)")
        print("  - If in Docker, ensure port mapping is correct")
        
    except KeyboardInterrupt:
        print(f"\n\nStopped by user")
        print(f"Total packets received: {packet_count}")
        
    finally:
        sock.close()
        print("Socket closed")

if __name__ == "__main__":
    try:
        test_rokoko_listener()
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

