import serial

#Clase creada para facilitar la comunicacion serie con el controlador de servos
class ServoController:
    def __init__(self, PORT = '/dev/ttyACM0', BAUDRATE = 115200, TIMEOUT = 1):
        try:
            self.sP = serial.Serial(
                port=PORT,
                baudrate=BAUDRATE,
                timeout=TIMEOUT
            )
            self.sp.write("~RTBTRT\r\n~OL".encode('ascii'))
        except serial.SerialException as e:
            print(f"Error al acceder al puerto serie: {e}")
        except Exception as e:
            print(f"Ocurrió un error: {e}")
        
    def moveServo(self, id, pulsewidth, velocity = 1000, delay = 0):
        instruction = f'#{id}P{pulsewidth}T{velocity}D{delay}\r\n'
        self.sP.write(instruction.encode('ascii'))

    def moveMultipleServos(self, movements, velocity = 1000, delay = 0):
        if (movements.length == 0):
            return
        instruction = ""
        for movement in movements:
            instruction += f'#{movement['id']}P{movement['pulsewidth']}'
        instruction += f'T{velocity}D{delay}\r\n'
        self.sP.write(instruction.encode('ascii'))

    def closeConnection(self):
        self.sp.write('~BY\r\n'.encode('ascii'))