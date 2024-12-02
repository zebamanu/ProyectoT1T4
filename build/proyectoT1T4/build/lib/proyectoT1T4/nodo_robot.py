import rclpy #ROS Client Library for the Python language.
from rclpy.node import Node #Importo clase Nodo
from geometry_msgs.msg import Vector3 #Importo tipo de mensajes que deseo utilizar
from std_msgs.msg import Float32
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

    def walk(self):
        walkSequence = [ {'id' : 2, 'pulsewidth' : 1520}, {'id' : 4,   'pulsewidth' : 2280},
                         {'id' : 10, 'pulsewidth' : 1520}, 
                         {'id' : 23, 'pulsewidth' : 1920},
                         {'id' : 29, 'pulsewidth' : 1430}, {'id' : 31, 'pulsewidth' : 1210}]
        walkSequence2 = [ {'id' : 2, 'pulsewidth' : 2050}, {'id' : 4,   'pulsewidth' : 1520},
                         {'id' : 10, 'pulsewidth' : 990}, 
                         {'id' : 23, 'pulsewidth' : 1300},
                         {'id' : 29, 'pulsewidth' : 950}, {'id' : 31, 'pulsewidth' : 1750}]
        for i in range(0,3):
            self.moveMultipleServos(walkSequence,700,0)
            time.sleep(0.9)
            self.moveMultipleServos(walkSequence2,700,0)
            time.sleep(0.9)
        
    def saludar(self):
        self.moveServo(10,2000,500)
        time.sleep(0.5)
        self.moveServo(10,1100,500)
        time.sleep(0.5)
        self.moveServo(10,2000,500)
        time.sleep(0.5)
        self.moveServo(10,1100,500)
        time.sleep(0.5)
        self.moveServo(10,2000,500)
        time.sleep(0.5)
        self.moveServo(12,800,200)
        time.sleep(0.5)
        self.moveServo(12,1660,500)
        time.sleep(0.5)
        self.moveServo(12,800,500)
        time.sleep(0.5)
        self.sendInitialValues()


    def sendInitialValues(self):
        initialValues = [{'id' : 1, 'pulsewidth' : 1450}, {'id' : 2, 'pulsewidth' : 1800}, {'id' : 3, 'pulsewidth' : 2300}, {'id' : 4, 'pulsewidth' : 1800}, {'id' : 7, 'pulsewidth' : 1000},
                         {'id' : 10, 'pulsewidth' : 1100}, {'id' : 11, 'pulsewidth' : 760}, {'id' : 12, 'pulsewidth' : 1660},
                         {'id' : 16, 'pulsewidth' : 1830},
                         {'id' : 21, 'pulsewidth' : 1150}, {'id' : 22, 'pulsewidth' : 1950},{'id' : 23, 'pulsewidth' : 1600},
                         {'id' : 28, 'pulsewidth' : 900}, {'id' : 29, 'pulsewidth' : 1200}, {'id' : 30, 'pulsewidth' : 1000}, {'id' : 31, 'pulsewidth' : 1500}, {'id' : 32, 'pulsewidth' : 720}]
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
            'T1T4/moverServo',
            self.listener_callback,
            10) #creo un suscriptor
        self.subscription = self.create_subscription(
            Float32,
            'T1T4/walk',
            self.walk_callback,
            10) #creo un suscriptor
        self.subscription = self.create_subscription(
            Float32,
            'T1T4/reset',
            self.reset_callback,
            10) #creo un suscriptor
        self.subscription = self.create_subscription(
            Float32,
            'T1T4/saludar',
            self.saludar_callback,
            10) #creo un suscriptor
        self.subscription # prevent unused variable warning
        self.servoController.sendInitialValues()

    #funcion del callbak del suscriptor
    def listener_callback(self, msg):
        self.get_logger().info('Moviendo: "%d"' % msg.x)
        self.servoController.moveServo(int(msg.x),int(msg.y))

    def walk_callback(self,msg):
        self.servoController.walk()
        self.servoController.sendInitialValues()
    
    def reset_callback(self,msg):
        self.servoController.sendInitialValues()

    def saludar_callback(self,msg):
        self.servoController.saludar()
    
    def destroy_node(self,msg):
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
