#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

class RetornoComDesvio:
    def __init__(self):
        rospy.init_node('retorno_odometrico_desvio')

        # Estado do robô
        self.pose = None
        self.yaw = 0.0

        # Sensores
        self.distancias = {'front': float('inf'), 'left': float('inf'), 'right': float('inf')}

        # Publicador de velocidade
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Odometria
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Sensores Sonar e ToF (tipo Range)
        for direcao in ['front', 'left', 'right']:
            rospy.Subscriber(f'/sonar_sensor/sonar_{direcao}', Range, self.range_callback(direcao))
            rospy.Subscriber(f'/tof3d_sensor/tof3d_{direcao}/data', Range, self.range_callback(direcao))

        self.rate = rospy.Rate(10)

        # Espera odometria
        rospy.loginfo("Aguardando odometria para iniciar retorno...")
        while not rospy.is_shutdown() and self.pose is None:
            self.rate.sleep()

        rospy.loginfo("Odometria capturada. Iniciando retorno ao marco zero.")
        self.navegar_ate_origem()

    def range_callback(self, direcao):
        def callback(msg):
            if msg.range >= msg.min_range and msg.range <= msg.max_range:
                self.distancias[direcao] = msg.range
        return callback

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        q = self.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def chegou_ao_marco_zero(self, tolerancia=0.2):
        if not self.pose:
            return False
        dx = self.pose.position.x
        dy = self.pose.position.y
        distancia = math.sqrt(dx**2 + dy**2)
        return distancia <= tolerancia

    def obstaculo_a_frente(self, limite=0.4):
        return self.distancias['front'] < limite

    def lado_mais_livre(self):
        return 'left' if self.distancias['left'] > self.distancias['right'] else 'right'

    def normalizar_angulo(self, ang):
        while ang > math.pi:
            ang -= 2 * math.pi
        while ang < -math.pi:
            ang += 2 * math.pi
        return ang

    def navegar_ate_origem(self):
        while not rospy.is_shutdown():
            if self.chegou_ao_marco_zero():
                rospy.loginfo("✅ Robô chegou ao marco zero!")
                self.cmd_pub.publish(Twist())
                break

            twist = Twist()

            if self.obstaculo_a_frente():
                direcao = self.lado_mais_livre()
                twist.angular.z = 0.5 if direcao == 'left' else -0.5
                rospy.loginfo(f"⚠️ Obstáculo à frente. Desviando para {direcao}.")
            else:
                x = self.pose.position.x
                y = self.pose.position.y
                dx = -x
                dy = -y
                dist = math.sqrt(dx**2 + dy**2)
                ang_objetivo = math.atan2(dy, dx)
                erro = self.normalizar_angulo(ang_objetivo - self.yaw)

                if abs(erro) > 0.15:
                    twist.angular.z = 0.4 * erro
                else:
                    twist.linear.x = 0.2

            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        RetornoComDesvio()
    except rospy.ROSInterruptException:
        pass
