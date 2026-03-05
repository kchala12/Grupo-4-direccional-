import serial
import time

# ============================
# CONFIGURAR PUERTO SERIAL
# ============================

ser = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2)  # Esperar a que la ESP32 reinicie

# ============================
# FUNCIÓN PARA ENVIAR VELOCIDAD
# ============================

def enviar_velocidad(izquierda, derecha):
    """
    Envía velocidad a la ESP32.
    izquierda y derecha deben estar entre -255 y 255.
    """

    comando = f"VEL:{izquierda},{derecha}\n"
    ser.write(comando.encode())
    print(f"Enviado: {comando.strip()}")

# ============================
# PRUEBA MANUAL
# ============================

try:
    while True:
        entrada = input("Ingresa velocidades (ej: 100 -100): ")

        partes = entrada.split()

        if len(partes) == 2:
            vel_izq = int(partes[0])
            vel_der = int(partes[1])
            enviar_velocidad(vel_izq, vel_der)

except KeyboardInterrupt:
    print("\nDeteniendo motores...")
    enviar_velocidad(0, 0)
    ser.close()