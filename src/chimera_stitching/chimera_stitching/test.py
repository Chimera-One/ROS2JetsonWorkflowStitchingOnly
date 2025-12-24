import socket

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', 6060))

print("Listening...")

while True:
    data, addr = s.recvfrom(1024)
    print("RECEIVED:", data, addr)
