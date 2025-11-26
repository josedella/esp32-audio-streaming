import socket
import pyaudio
import struct

# --- Configuration ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5000
SAMPLE_RATE = 16000
CHUNK = 64

# 1. Setup Audio
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=SAMPLE_RATE,
                output=True,
                frames_per_buffer=CHUNK)

# 2. Setup Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# --- FIX: Set a timeout so the loop checks for Ctrl+C every 0.5 seconds ---
sock.settimeout(0.5) 

print(f"🎧 Listening on {UDP_PORT}...")
print("📊 Visualizer Active: Speak into the mic!")
print("❌ Press Ctrl+C to stop safely.")

try:
    while True:
        try:
            # Try to get a packet (waits max 0.5s)
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            # No packet arrived? Loop back to top to check for Ctrl+C
            continue
        
        # --- If we are here, we have data! ---

        # A. Play Audio
        stream.write(data)

        # B. Visualizer (Optional - comment out if it slows down audio)
        count = len(data) // 2
        if count > 0:
            samples = struct.unpack(f"<{count}h", data)
            peak = max(abs(s) for s in samples)
            
            # Scale bar graph (Peak / 500 fits nicely on screen)
            bars = "█" * (peak // 500)
            print(f"Vol: {peak:5d} | {bars}")

except KeyboardInterrupt:
    print("\n🛑 Stopping...")
    # Safe Shutdown
    stream.stop_stream()
    stream.close()
    p.terminate()
    sock.close()
    print("Bye!")
