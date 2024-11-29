import rclpy #ROS Client Library for the Python language.
from rclpy.node import Node #Importo clase Nodo
from std_msgs.msg import String #Importo tipo de mensajes que deseo utilizar
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
        self.sP.write('~BY\r\n'.encode('ascii'))


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.servoController = ServoController()
        self.subscription = self.create_subscription(
            String,
            'topicoRobot',
            self.listener_callback,
            10) #creo un suscriptor
        self.subscription # prevent unused variable warning

    #funcion del callbak del suscriptor
    def listener_callback(self, msg):
        self.get_logger().info('Moviendo: "%s"' % msg.data)
        self.servoController.moveServo(1,600)
        time.sleep(1)
        self.servoController.moveServo(1,1300)
    
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
