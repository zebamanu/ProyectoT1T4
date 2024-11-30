import rclpy #ROS Client Library for the Python language.
from rclpy.node import Node #Importo clase Nodo
from geometry_msgs.msg import Vector3 #Importo tipo de mensajes que deseo utilizar
import time
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
            self.sP.write("~RTBTRT\r\n~OL".encode('ascii'))
        except serial.SerialException as e:
            print(f"Error al acceder al puerto serie: {e}")
        except Exception as e:
            print(f"Ocurrió un error: {e}")

    def sendInitialValues(self):
        initialValues = [{'id' : 1, 'pulsewidth' : 1480}, {'id' : 2, 'pulsewidth' : 1850}, {'id' : 3, 'pulsewidth' : 2300}, {'id' : 4, 'pulsewidth' : 1930}, {'id' : 5, 'pulsewidth' : 1920},
                         {'id' : 10, 'pulsewidth' : 1100}, {'id' : 11, 'pulsewidth' : 760}, {'id' : 12, 'pulsewidth' : 1660},
                         {'id' : 16, 'pulsewidth' : 1830},
                         {'id' : 21, 'pulsewidth' : 1150}, {'id' : 22, 'pulsewidth' : 1950},{'id' : 23, 'pulsewidth' : 1600},
                         {'id' : 28, 'pulsewidth' : 850}, {'id' : 29, 'pulsewidth' : 1230}, {'id' : 30, 'pulsewidth' : 960}, {'id' : 31, 'pulsewidth' : 1500}, {'id' : 32, 'pulsewidth' : 700}]
        self.moveMultipleServos(initialValues)
        
    def moveServo(self, id, pulsewidth, velocity = 1000, delay = 0):
        instruction = f'#{id}P{pulsewidth}T{velocity}D{delay}\r\n'
        self.sP.write(instruction.encode('ascii'))

    def moveMultipleServos(self, movements, velocity = 1000, delay = 0):
        if (len(movements) == 0):
            return
        instruction = ""
        for movement in movements:
            instruction += f'#{movement['id']}P{movement['pulsewidth']}'
        instruction += f'T{velocity}D{delay}\r\n'
        self.sP.write(instruction.encode('ascii'))

    def closeConnection(self):
        self.sP.write('~BY\r\n'.encode('ascii'))


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.servoController = ServoController()
        self.subscription = self.create_subscription(
            Vector3,
            'topicoRobot',
            self.listener_callback,
            10) #creo un suscriptor
        self.subscription # prevent unused variable warning
        self.servoController.sendInitialValues()

    #funcion del callbak del suscriptor
    def listener_callback(self, msg):
        self.get_logger().info('Moviendo: "%d"' % msg.x)
        self.servoController.moveServo(int(msg.x),int(msg.y))
    
    def destroy_node(self):
        self.servoController.closeConnection()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destruimos el nodo en forma explicita 
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
