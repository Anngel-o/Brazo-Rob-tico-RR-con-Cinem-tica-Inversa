import serial
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Función de cinemática inversa
def inverse_kinematics(x, y, L1, L2):
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(D) > 1.0:
        return None, None  # Posición no alcanzable
    theta2 = math.atan2(math.sqrt(1 - D**2), D)  # Solución "codo abajo"
    theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    return theta1, theta2

# Función para convertir ángulos articulares a ángulos de servomotor
def joint_to_servo_angles(theta1, theta2):
    angle1_deg = math.degrees(theta1)
    angle2_deg = math.degrees(theta2)

    # Ajuste de los ángulos para los servos
    servo1_angle = angle1_deg + 90  # Ajusta según la orientación física del servo
    servo2_angle = angle2_deg + 90  # Ajusta según la orientación física del servo

    return servo1_angle, servo2_angle

# Conexión al Arduino
Puerto = 'COM4'  # Ajusta el puerto según tu sistema
L1 = 10.5  # Longitud del primer segmento
L2 = 12.0  # Longitud del segundo segmento

# Funciones para dibujar figuras
def draw_square(center_x, center_y, side_length, num_points=50):
    half_side = side_length / 2
    points = [
        (center_x - half_side, center_y - half_side),
        (center_x + half_side, center_y - half_side),
        (center_x + half_side, center_y + half_side),
        (center_x - half_side, center_y + half_side),
        (center_x - half_side, center_y - half_side)
    ]
    # Interpolar puntos para movimientos suaves
    x_points, y_points = [], []
    for i in range(len(points) - 1):
        x_interp = np.linspace(points[i][0], points[i+1][0], num_points)
        y_interp = np.linspace(points[i][1], points[i+1][1], num_points)
        x_points.extend(x_interp)
        y_points.extend(y_interp)
    return list(zip(x_points, y_points))

def draw_circle(center_x, center_y, radius, num_points=100):
    t = np.linspace(0, 2 * np.pi, num_points)
    x_points = center_x + radius * np.sin(t)
    y_points = center_y + radius * np.cos(t)
    return list(zip(x_points, y_points))

def draw_rectangle(center_x, center_y, width, height, num_points=50):
    half_width = width / 2
    half_height = height / 2
    points = [
        (center_x - half_width, center_y - half_height),
        (center_x + half_width, center_y - half_height),
        (center_x + half_width, center_y + half_height),
        (center_x - half_width, center_y + half_height),
        (center_x - half_width, center_y - half_height)
    ]
    x_points, y_points = [], []
    for i in range(len(points) - 1):
        x_interp = np.linspace(points[i][0], points[i+1][0], num_points)
        y_interp = np.linspace(points[i][1], points[i+1][1], num_points)
        x_points.extend(x_interp)
        y_points.extend(y_interp)
    return list(zip(x_points, y_points))

def draw_heart(center_x, center_y, size, num_points=100):
    t = np.linspace(0, 2 * np.pi, num_points)
    x_points = center_x + size * 16 * np.sin(t)**3 / 16
    y_points = center_y + size * (13 * np.cos(t) - 5 * np.cos(2 * t)
                                  - 2 * np.cos(3 * t) - np.cos(4 * t)) / 16
    return list(zip(x_points, y_points))

# Comunicación con Arduino
try:
    arduino = serial.Serial(port=Puerto, baudrate=115200, timeout=1)
    time.sleep(2)
    print("Conexión establecida con el Arduino.")
except serial.SerialException as e:
    print(f"No se pudo conectar con Arduino: {e}")
    arduino = None

# Mostrar menú y ejecutar el programa principal
while True:
    # Mostrar menú al usuario
    print("\nSelecciona la figura que deseas dibujar:")
    print("1. Cuadrado")
    print("2. Círculo")
    print("3. Rectángulo")
    print("4. Corazón")
    print("5. Salir")

    opcion = input("Ingresa el número de la figura: ")

    if opcion == '1':
        figura = draw_square(15, -2, 5)
    elif opcion == '2':
        figura = draw_circle(18, -5, 3)
    elif opcion == '3':
        figura = draw_rectangle(13, 2, 6, 4) 
    elif opcion == '4':
        figura = draw_heart(13, 4, 3)
    elif opcion == '5':
        print("Saliendo del programa.")
        break
    else:
        print("Opción no válida.")
        continue

    # Preparar datos para la simulación y control del brazo
    angles_list = []  # Lista para almacenar los ángulos calculados
    positions = []    # Lista para almacenar las posiciones del brazo

    for x, y in figura:
        theta1, theta2 = inverse_kinematics(x, y, L1, L2)
        if theta1 is None or theta2 is None:
            print(f"Posición no alcanzable: x={x:.2f}, y={y:.2f}")
            continue

        servo_angle1, servo_angle2 = joint_to_servo_angles(theta1, theta2)

        # Asegurar que los ángulos estén dentro del rango [0, 180]
        servo_angle1 = max(0, min(180, servo_angle1))
        servo_angle2 = max(0, min(180, servo_angle2))

        # Almacenar ángulos y posiciones para simulación
        angles_list.append((servo_angle1, servo_angle2))

        # Coordenadas del primer y segundo segmento
        x0, y0 = 0, 0
        x1 = L1 * math.cos(math.radians(servo_angle1 - 90))
        y1 = L1 * math.sin(math.radians(servo_angle1 - 90))
        x2 = x1 + L2 * math.cos(math.radians(servo_angle1 + servo_angle2 - 180))
        y2 = y1 + L2 * math.sin(math.radians(servo_angle1 + servo_angle2 - 180))
        positions.append(((x0, y0), (x1, y1), (x2, y2)))

    # Función para actualizar la animación
    def animate(i):
        if i >= len(positions):
            return line,
        # Actualizar la posición de las líneas del brazo
        x_coords = [positions[i][0][0], positions[i][1][0], positions[i][2][0]]
        y_coords = [positions[i][0][1], positions[i][1][1], positions[i][2][1]]
        line.set_data(x_coords, y_coords)
        return line,

    # Configuración de la figura para la simulación
    fig, ax = plt.subplots()
    ax.set_xlim(-L1 - L2 - 5, L1 + L2 + 5)
    ax.set_ylim(-L1 - L2 - 5, L1 + L2 + 5)
    ax.set_aspect('equal')
    ax.grid(True)

    # Líneas que representan los segmentos del brazo
    line, = ax.plot([], [], 'o-', lw=2)

    # Puntos de la trayectoria deseada
    x_traj = [p[0] for p in figura]
    y_traj = [p[1] for p in figura]
    ax.plot(x_traj, y_traj, 'r--', alpha=0.5)

    # Iniciar la animación
    ani = FuncAnimation(fig, animate, frames=len(positions), interval=100, blit=True)

    # Enviar posiciones al Arduino y mostrar la simulación
    if arduino:
        for idx, (servo_angle1, servo_angle2) in enumerate(angles_list):
            message = f"[{servo_angle1:.2f},{servo_angle2:.2f}],\n"
            arduino.write(message.encode())
            print(f"Enviando: {message.strip()}")

            # Actualizar la animación al mismo tiempo
            animate(idx)
            plt.pause(0.001)  # Pequeña pausa para actualizar la animación

            # Ajustar la velocidad del brazo
            time.sleep(0.1)  # Puedes ajustar este tiempo según sea necesario

        print("Figura dibujada.")
    else:
        print("No se pudo establecer comunicación con el Arduino.")

    plt.show()

if arduino:
    arduino.close()
