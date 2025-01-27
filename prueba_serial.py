import serial
import time

def main():
    # Configuración del puerto serie
    port = '/dev/ttyUSB0'  # Cambia esto si usas otro puerto
    baudrate = 9600        # Velocidad en baudios
    timeout = 1            # Tiempo de espera en segundos (opcional)

    try:
        # Abrir el puerto serie
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            print(f"Conectado al puerto {port} a {baudrate} baudios.")

            # Ciclo para enviar datos
            while True:
                data = input("Escribe el dato que deseas enviar (o 'salir' para terminar): ")
                if data.lower() == 'salir':
                    print("Cerrando conexión...")
                    break

                # Escribir datos al puerto serie
                ser.write(data.encode('utf-8'))  # Codificar los datos como UTF-8
                print(f"Enviado: {data}")

                # Leer respuesta opcional (si el dispositivo responde)
                time.sleep(0.5)  # Esperar un poco antes de leer
                if ser.in_waiting > 0:  # Comprobar si hay datos para leer
                    response = ser.read(ser.in_waiting).decode('utf-8')
                    print(f"Respuesta recibida: {response}")

    except serial.SerialException as e:
        print(f"Error al acceder al puerto serie: {e}")

    except KeyboardInterrupt:
        print("\nConexión interrumpida por el usuario.")

if __name__ == "__main__":
    main()
