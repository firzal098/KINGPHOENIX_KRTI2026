"""
Supervisor Controller: simulation_bridge.py
Listens for TCP commands from the web app on port 14556.
Supports: RESET_WORLD, RESET_SIMULATION, PING
"""
import socket
import select
from controller import Supervisor

# 1. Initialize Supervisor
supervisor = Supervisor()
TIME_STEP = int(supervisor.getBasicTimeStep())

# Force simulation to unpause immediately
supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)

# 2. Set up the TCP Socket Server
HOST = '127.0.0.1'
PORT = 14556

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT))
server_socket.listen(5)
server_socket.setblocking(False) 

print(f"Supervisor! bridge listening on {HOST}:{PORT}...")

# 3. Main Simulation Loop
while supervisor.step(TIME_STEP) != -1:
    readable, _, _ = select.select([server_socket], [], [], 0.0)
    for sock in readable:
        try:
            client_socket, addr = server_socket.accept()
            client_socket.settimeout(0.05)
            try:
                raw_data = client_socket.recv(1024)
                if not raw_data:
                    client_socket.close()
                    continue
                data = raw_data.decode('utf-8').strip()

                # Endpoint 1: Hard World Reload
                if data == "RESET_WORLD":
                    print(f"[{addr[0]}] Hard reset received. Reloading world...")
                    client_socket.sendall(b"SUCCESS: World reloading\n")
                    client_socket.close()
                    supervisor.worldReload()
                    break

                # Endpoint 2: Soft Simulation Reset
                elif data == "RESET_SIMULATION":
                    print(f"[{addr[0]}] Soft reset received. Resetting simulation and physics...")
                    supervisor.simulationReset()
                    supervisor.simulationResetPhysics()
                    supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)

                    # Restart the drone's controller
                    drone_node = supervisor.getFromDef("KRTIDrone")
                    if drone_node is not None:
                        drone_node.restartController()
                        client_socket.sendall(b"SUCCESS: Sim and Drone restarted\n")
                    else:
                        print("ERROR: Could not find node with DEF 'KRTIDrone'")
                        client_socket.sendall(b"ERROR: Drone DEF not found\n")
                    client_socket.close()

                # Endpoint 3: Connection Check (Ping)
                elif data == "PING":
                    client_socket.sendall(b"PONG\n")
                    client_socket.close()

                # Handle unknown commands
                else:
                    print(f"[{addr[0]}] Unknown command received: {data}")
                    client_socket.sendall(b"ERROR: Unknown command\n")
                    client_socket.close()

            except Exception as e:
                print(f"Error handling client request: {e}")
                try:
                    client_socket.close()
                except Exception:
                    pass
        except Exception as e:
            print(f"Error accepting connection: {e}")