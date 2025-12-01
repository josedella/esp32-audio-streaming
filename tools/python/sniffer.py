import socket

# Configuration
UDP_IP = "0.0.0.0"
UDP_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
# --- FIX: Wake up every 0.5 seconds to check for Ctrl+C ---
sock.settimeout(0.5) 

print(f"🕵️  Sniffer listening on port {UDP_PORT}...")
print("Waiting for ESP32...")

while True:
    try:
        # Wait for a packet (blocks for max 0.5 seconds now)
        data, addr = sock.recvfrom(2048)
        print(f"✅ Received {len(data)} bytes from {addr[0]}")

    except socket.timeout:
        # No packet received in 0.5s, loop back and check for Ctrl+C again
        continue 
    except KeyboardInterrupt:
        print("\nStopping...")
        break
    except Exception as e:
        print(f"Error: {e}")
