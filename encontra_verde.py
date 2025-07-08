#!/usr/bin/env python3
# Script ROS melhorado para exploração e aproximação de cores - sem barulho, perfeito fora isso

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from enum import Enum

class EstadoRobo(Enum):
    EXPLORANDO = 1
    APROXIMANDO = 2
    DESVIANDO = 3
    FINALIZADO = 4

class DetectorCor:
    def __init__(self, cor):
        self.limite_cor = self._definir_limites_cor(cor)
        self.area_min = rospy.get_param('~area_min_deteccao', 500)

    def _definir_limites_cor(self, cor):
        """Define os limites HSV para cada cor"""
        if cor == 'vermelho':
            return [
                (np.array([0, 70, 50]), np.array([10, 255, 255])),
                (np.array([170, 70, 50]), np.array([180, 255, 255]))
            ]
        elif cor == 'azul':
            return [(np.array([100, 150, 0]), np.array([140, 255, 255]))]
        elif cor == 'verde':
            return [(np.array([40, 70, 70]), np.array([80, 255, 255]))]
        else:
            raise ValueError(f"Cor '{cor}' não suportada")

    def detectar(self, frame):
        """Detecta a cor na imagem e retorna o centro do maior objeto"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Cria máscara combinada para todas as faixas de cor
        mask_total = None
        for lower, upper in self.limite_cor:
            mask = cv2.inRange(hsv, lower, upper)
            if mask_total is None:
                mask_total = mask
            else:
                mask_total = cv2.bitwise_or(mask_total, mask)

        # Encontra contornos
        contornos, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        melhor_contorno = self._selecionar_melhor_contorno(contornos, frame.shape)
        if melhor_contorno is not None:
            return self._calcular_centro(melhor_contorno)

        return None

    def _selecionar_melhor_contorno(self, contornos, frame_shape):
        """Seleciona o melhor contorno baseado em critérios de qualidade"""
        candidatos = []

        for contorno in contornos:
            area = cv2.contourArea(contorno)

            if area < self.area_min:
                continue

            # Verifica compacidade (evita ruído)
            perimetro = cv2.arcLength(contorno, True)
            if perimetro == 0:
                continue

            compacidade = 4 * np.pi * area / (perimetro * perimetro)

            # Verifica aspect ratio
            x, y, w, h = cv2.boundingRect(contorno)
            aspect_ratio = w / h if h != 0 else 0

            # Filtros de qualidade
            if (compacidade > 0.1 and  # Não muito irregular
                0.3 < aspect_ratio < 3.0 and  # Formato razoável
                area > 0.001 * frame_shape[0] * frame_shape[1]):  # % mínima da imagem

                candidatos.append((contorno, area))

        return max(candidatos, key=lambda x: x[1])[0] if candidatos else None

    def _calcular_centro(self, contorno):
        """Calcula o centro do contorno usando momentos"""
        M = cv2.moments(contorno)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
        return None

class AnalisadorProfundidade:
    def __init__(self, dist_min=0.5):
        self.dist_min = dist_min

    def analisar_obstaculos(self, depth_image):
        """Analisa obstáculos em setores da imagem"""
        altura, largura = depth_image.shape

        # Divide imagem em 3 setores
        setor_esq = depth_image[:, :largura//3]
        setor_centro = depth_image[:, largura//3:2*largura//3]
        setor_dir = depth_image[:, 2*largura//3:]

        def calcular_distancia_setor(setor):
            validos = setor[np.isfinite(setor)]
            return np.percentile(validos, 10) if validos.size > 0 else np.inf

        dist_esq = calcular_distancia_setor(setor_esq)
        dist_centro = calcular_distancia_setor(setor_centro)
        dist_dir = calcular_distancia_setor(setor_dir)

        return {
            'esquerda': dist_esq < self.dist_min,
            'centro': dist_centro < self.dist_min,
            'direita': dist_dir < self.dist_min,
            'melhor_direcao': 'esquerda' if dist_esq > dist_dir else 'direita',
            'distancias': {'esq': dist_esq, 'centro': dist_centro, 'dir': dist_dir}
        }

    def calcular_distancia_alvo(self, depth_image, centro_cor):
        """Calcula distância até o objeto colorido"""
        if centro_cor is None:
            return np.inf

        altura, largura = depth_image.shape
        cx, cy = centro_cor

        # Janela ao redor do centro
        janela = 10
        xmin = max(cx - janela, 0)
        xmax = min(cx + janela, largura - 1)
        ymin = max(cy - janela, 0)
        ymax = min(cy + janela, altura - 1)

        recorte = depth_image[ymin:ymax+1, xmin:xmax+1]
        validos = recorte[np.isfinite(recorte)]

        if validos.size > 0:
            return np.percentile(validos, 20)
        return np.inf

class ControladorMovimento:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel_linear_explorar = rospy.get_param('~vel_linear_explorar', 0.2)
        self.vel_linear_aproximar = rospy.get_param('~vel_linear_aproximar', 0.15)
        self.vel_angular_girar = rospy.get_param('~vel_angular_girar', 0.5)
        self.kp_angular = rospy.get_param('~kp_angular', 0.005)

    def calcular_comando_angular(self, centro_cor, largura_imagem):
        """Calcula comando angular para se orientar em direção à cor"""
        if centro_cor is None:
            return 0.0

        centro_x = centro_cor[0]
        centro_tela = largura_imagem / 2
        erro = centro_x - centro_tela

        # Controle proporcional
        angular_z = -self.kp_angular * erro

        # Limita velocidade angular
        return max(-0.5, min(0.5, angular_z))

    def executar_movimento(self, linear_x, angular_z):
        """Executa comando de movimento"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def parar(self):
        """Para o robô"""
        self.cmd_pub.publish(Twist())

class ExploraEAproximaCor:
    def __init__(self, cor='vermelho'):
        rospy.init_node('explora_e_aproxima_cor')

        # Componentes
        self.detector_cor = DetectorCor(cor)
        self.analisador_prof = AnalisadorProfundidade()
        self.controlador = ControladorMovimento()
        self.bridge = CvBridge()

        # Parâmetros
        self.distancia_desejada = rospy.get_param('~distancia_desejada', 0.5)
        self.timeout_imagem = rospy.Duration(rospy.get_param('~timeout_imagem', 2.0))
        self.timeout_missao = rospy.Duration(rospy.get_param('~timeout_missao', 300.0))
        self.debug_ativo = rospy.get_param('~debug', False)

        # Estado
        self.estado = EstadoRobo.EXPLORANDO
        self.centro_cor = None
        self.largura_imagem = 640  # Será atualizado
        self.distancia_alvo = np.inf
        self.info_obstaculos = {}

        # Histórico para robustez
        self.historico_deteccoes = []
        self.max_historico = 5
        self.min_deteccoes_consecutivas = 3

        # Timestamps
        self.ultima_imagem = rospy.Time.now()
        self.inicio_missao = rospy.Time.now()

        # Subscribers
        rospy.Subscriber('/camera/color/image_raw', Image, self.imagem_callback)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)

        # Publisher de debug (opcional)
        if self.debug_ativo:
            self.debug_pub = rospy.Publisher('/debug_image', Image, queue_size=1)

        self.rate = rospy.Rate(10)

        rospy.loginfo(f"Robô iniciado. Procurando por cor: {cor}")

    def imagem_callback(self, msg):
        """Callback da imagem colorida"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.largura_imagem = frame.shape[1]
            self.ultima_imagem = rospy.Time.now()

            # Detecta cor
            centro_detectado = self.detector_cor.detectar(frame)

            # Valida detecção com histórico
            cor_detectada = centro_detectado is not None
            if self.validar_deteccao(cor_detectada):
                self.centro_cor = centro_detectado
            else:
                self.centro_cor = None

            # Debug visual
            if self.debug_ativo and hasattr(self, 'debug_pub'):
                self.publicar_debug(frame)

        except Exception as e:
            rospy.logwarn(f'Erro ao processar imagem: {e}')

    def depth_callback(self, msg):
        """Callback da imagem de profundidade"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Analisa obstáculos
            self.info_obstaculos = self.analisador_prof.analisar_obstaculos(depth_image)

            # Calcula distância ao alvo
            self.distancia_alvo = self.analisador_prof.calcular_distancia_alvo(
                depth_image, self.centro_cor)

        except Exception as e:
            rospy.logwarn(f'Erro ao processar depth: {e}')
            self.info_obstaculos = {}
            self.distancia_alvo = np.inf

    def validar_deteccao(self, cor_detectada):
        """Valida detecção usando histórico temporal"""
        self.historico_deteccoes.append(cor_detectada)

        if len(self.historico_deteccoes) > self.max_historico:
            self.historico_deteccoes.pop(0)

        # Só considera detectado se teve várias detecções recentes
        if len(self.historico_deteccoes) < self.min_deteccoes_consecutivas:
            return False

        deteccoes_recentes = sum(self.historico_deteccoes[-self.min_deteccoes_consecutivas:])
        return deteccoes_recentes >= self.min_deteccoes_consecutivas

    def verificar_timeouts(self):
        """Verifica timeouts de segurança"""
        agora = rospy.Time.now()

        # Timeout de imagem
        if agora - self.ultima_imagem > self.timeout_imagem:
            rospy.logwarn("Timeout: sem imagens há muito tempo!")
            self.controlador.parar()
            return True

        # Timeout de missão
        if agora - self.inicio_missao > self.timeout_missao:
            rospy.loginfo("Timeout da missão atingido!")
            self.estado = EstadoRobo.FINALIZADO
            return True

        return False

    def atualizar_estado(self):
        """Atualiza a máquina de estados"""
        cor_detectada = self.centro_cor is not None
        obstaculo_frontal = self.info_obstaculos.get('centro', False)

        if self.estado == EstadoRobo.EXPLORANDO:
            if cor_detectada and not obstaculo_frontal:
                self.estado = EstadoRobo.APROXIMANDO
                rospy.loginfo("Cor detectada! Mudando para modo aproximação")
            elif obstaculo_frontal or self.info_obstaculos.get('esquerda', False) or self.info_obstaculos.get('direita', False):
                self.estado = EstadoRobo.DESVIANDO

        elif self.estado == EstadoRobo.APROXIMANDO:
            if self.distancia_alvo <= self.distancia_desejada:
                self.estado = EstadoRobo.FINALIZADO
                rospy.loginfo(f"Chegou a {self.distancia_desejada:.2f}m da cor alvo!")
            elif obstaculo_frontal:
                self.estado = EstadoRobo.DESVIANDO
            elif not cor_detectada:
                self.estado = EstadoRobo.EXPLORANDO
                rospy.loginfo("Cor perdida. Voltando para exploração")

        elif self.estado == EstadoRobo.DESVIANDO:
            if not obstaculo_frontal:
                if cor_detectada:
                    self.estado = EstadoRobo.APROXIMANDO
                else:
                    self.estado = EstadoRobo.EXPLORANDO

    def executar_comportamento(self):
        """Executa o comportamento baseado no estado atual"""
        if self.estado == EstadoRobo.EXPLORANDO:
            self.controlador.executar_movimento(
                self.controlador.vel_linear_explorar, 0.0)

        elif self.estado == EstadoRobo.APROXIMANDO:
            # Calcula comando angular para se orientar à cor
            angular_z = self.controlador.calcular_comando_angular(
                self.centro_cor, self.largura_imagem)

            self.controlador.executar_movimento(
                self.controlador.vel_linear_aproximar, angular_z)

        elif self.estado == EstadoRobo.DESVIANDO:
            # Escolhe direção de desvio baseada na análise de obstáculos
            if self.info_obstaculos.get('melhor_direcao') == 'esquerda':
                angular_z = self.controlador.vel_angular_girar
            else:
                angular_z = -self.controlador.vel_angular_girar

            self.controlador.executar_movimento(0.0, angular_z)

        elif self.estado == EstadoRobo.FINALIZADO:
            self.controlador.parar()

    def publicar_debug(self, frame):
        """Publica imagem de debug com informações visuais"""
        if self.debug_pub.get_num_connections() == 0:
            return

        debug_img = frame.copy()

        # Desenha centro da cor detectada
        if self.centro_cor:
            cv2.circle(debug_img, self.centro_cor, 10, (0, 255, 0), -1)
            cv2.putText(debug_img, f"Dist: {self.distancia_alvo:.2f}m",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Informações de obstáculos
        if self.info_obstaculos.get('centro', False):
            cv2.putText(debug_img, "OBSTACULO!",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Estado atual
        cv2.putText(debug_img, f"Estado: {self.estado.name}",
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        # Publica imagem de debug
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            rospy.logwarn(f"Erro ao publicar debug: {e}")

    def run(self):
        """Executa o loop principal do robô"""
        rospy.sleep(2)  # Espera estabilizar sensores

        while not rospy.is_shutdown():
            # Verifica timeouts
            if self.verificar_timeouts():
                break

            # Atualiza estado
            self.atualizar_estado()

            # Executa comportamento
            self.executar_comportamento()

            # Verifica se terminou
            if self.estado == EstadoRobo.FINALIZADO:
                rospy.loginfo("Missão finalizada com sucesso!")
                break

            self.rate.sleep()

        # Para o robô ao final
        self.controlador.parar()

if __name__ == '__main__':
    try:
        cor_alvo = rospy.get_param('~cor_alvo', 'verde')
        robo = ExploraEAproximaCor(cor=cor_alvo)
        robo.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Programa interrompido pelo usuário")
    except Exception as e:
        rospy.logerr(f"Erro fatal: {e}")
        # Para o robô em caso de erro
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        pub.publish(Twist())
