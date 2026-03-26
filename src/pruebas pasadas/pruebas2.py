import serial
import time

# -------- CONFIGURACION DEL PUERTO --------
PUERTO = "/dev/ttyUSB0"
BAUDRATE = 115200

# abrir puerto serial
ser = serial.Serial(PUERTO, BAUDRATE, timeout=1)

# esperar reinicio de la ESP32
time.sleep(2)

# limpiar buffer inicial
ser.reset_input_buffer()

print("Conexion con ESP32 establecida")

# -------- LOOP PRINCIPAL --------
while True:

    # pedir velocidades
    entrada = input("Velocidades (L,R): ")

    # construir comando
    comando = "VEL:" + entrada + "\n"

    # enviar comando
    ser.write(comando.encode())

    print("Enviado:", comando.strip())

    # esperar respuesta de la ESP
    tiempo_inicio = time.time()

    while time.time() - tiempo_inicio < 1:

        if ser.in_waiting > 0:

            respuesta = ser.readline().decode(errors="ignore").strip()

            if respuesta:
                print("ESP:", respuesta)