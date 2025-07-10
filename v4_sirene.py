# navegacao ruim, checar se sirene funciona. Adicionar so parte da sirene na V3 e testar


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from enum import Enum
import os
import threading
import time
import subprocess
import shutil

class EstadoRobo(Enum):
    EXPLORANDO = 1
    APROXIMANDO = 2
    DESVIANDO = 3
    FINALIZADO = 4

class GerenciadorSom:
    def __init__(self):
        """Inicializa o sistema de som"""
        self.som_habilitado = True
        self.ultimo_som_tocado = 0
        self.intervalo_min_som = 3.0  # Intervalo mínimo entre sons (segundos)

        # Diretório de sons (pode ser configurado via parâmetro)
        self.som_dir = rospy.get_param('~diretorio_sons', os.path.expanduser('~/catkin_ws/src/sons/'))

        # Verifica qual comando de som está disponível
        self.comando_som = self._detectar_comando_som()

        if self.comando_som:
            rospy.loginfo(f"Sistema de som inicializado com: {self.comando_som}")
        else:
            rospy.logwarn("Nenhum comando de som disponível - usando beep do sistema")

        # PREPARA OS SONS JÁ NA INICIALIZAÇÃO!
        self._preparar_sons()

    def tocar_som_objeto_encontrado(self):
        """Toca som quando objeto é encontrado"""
        if not self.som_habilitado:
            return

        agora = time.time()
        if agora - self.ultimo_som_tocado >= self.intervalo_min_som:
            try:
                # Não precisa mais checar hasattr, pois os sons já foram preparados no __init__
                threading.Thread(target=self._tocar_som_thread,
                                args=(self.som_objeto_encontrado, "objeto"),
                                daemon=True).start()
                self.ultimo_som_tocado = agora
                rospy.loginfo("🔊 Som: Objeto encontrado!")
            except Exception as e:
                rospy.logwarn(f"Erro ao tocar som: {e}")

    def tocar_som_missao_completa(self):
        """Toca som quando missão é completada"""
        if not self.som_habilitado:
            return

        try:
            threading.Thread(target=self._tocar_som_thread,
                           args=(self.som_missao_completa, "missao"),
                           daemon=True).start()
            rospy.loginfo("🎵 Som: Missão completa!")
        except Exception as e:
            rospy.logwarn(f"Erro ao tocar som: {e}")

    def _detectar_comando_som(self):
        """Detecta qual comando de som está disponível no sistema"""
        comandos = ['aplay', 'paplay', 'play', 'ffplay']

        for cmd in comandos:
            if shutil.which(cmd):
                return cmd

        return None

    def _criar_som_beep_wav(self, filename, frequency=800, duration=0.3):
        """Cria um arquivo WAV sintético usando SoX ou ffmpeg"""
        try:
            # Tenta usar SoX primeiro
            if shutil.which('sox'):
                cmd = [
                    'sox', '-n', filename,
                    'synth', str(duration), 'sine', str(frequency),
                    'fade', '0.01', str(duration), '0.01'
                ]
                subprocess.run(cmd, capture_output=True, check=True)
                return True

            # Tenta usar ffmpeg
            elif shutil.which('ffmpeg'):
                cmd = [
                    'ffmpeg', '-f', 'lavfi', '-i',
                    f'sine=frequency={frequency}:duration={duration}',
                    '-y', filename
                ]
                subprocess.run(cmd, capture_output=True, check=True)
                return True

            return False

        except Exception as e:
            rospy.logwarn(f"Erro ao criar som sintético: {e}")
            return False

    def _criar_som_melodia_wav(self, filename):
        """Cria uma melodia simples usando SoX"""
        try:
            if not shutil.which('sox'):
                return False

            # Frequências das notas: Dó, Mi, Sol, Dó (oitava)
            notas = [523, 659, 784, 1047]
            duracao_nota = 0.3

            # Cria arquivos temporários para cada nota
            arquivos_temp = []
            for i, freq in enumerate(notas):
                temp_file = f"/tmp/nota_{i}.wav"
                cmd = [
                    'sox', '-n', temp_file,
                    'synth', str(duracao_nota), 'sine', str(freq),
                    'fade', '0.01', str(duracao_nota), '0.01'
                ]
                subprocess.run(cmd, capture_output=True, check=True)
                arquivos_temp.append(temp_file)

            # Concatena as notas
            cmd = ['sox'] + arquivos_temp + [filename]
            subprocess.run(cmd, capture_output=True, check=True)

            # Remove arquivos temporários
            for temp_file in arquivos_temp:
                try:
                    os.remove(temp_file)
                except:
                    pass

            return True

        except Exception as e:
            rospy.logwarn(f"Erro ao criar melodia: {e}")
            return False

    def _preparar_sons(self):
        """Prepara os arquivos de som necessários"""
        # Cria diretório se não existir
        os.makedirs(self.som_dir, exist_ok=True)

        # Caminhos dos arquivos
        self.som_objeto_encontrado = os.path.join(self.som_dir, 'objeto_encontrado.wav')
        self.som_missao_completa = os.path.join(self.som_dir, 'missao_completa.wav')

        # Verifica/cria som de objeto encontrado
        if not os.path.exists(self.som_objeto_encontrado):
            if not self._criar_som_beep_wav(self.som_objeto_encontrado):
                self.som_objeto_encontrado = None
                rospy.loginfo("Usando beep do sistema para objeto encontrado")
            else:
                rospy.loginfo("Som sintético criado para objeto encontrado")
        else:
            rospy.loginfo(f"Som personalizado encontrado: {self.som_objeto_encontrado}")

        # Verifica/cria som de missão completa
        if not os.path.exists(self.som_missao_completa):
            if not self._criar_som_melodia_wav(self.som_missao_completa):
                self.som_missao_completa = None
                rospy.loginfo("Usando beep do sistema para missão completa")
            else:
                rospy.loginfo("Melodia sintética criada para missão completa")
        else:
            rospy.loginfo(f"Som personalizado encontrado: {self.som_missao_completa}")

    def tocar_som_objeto_encontrado(self):
        """Toca som quando objeto é encontrado"""
        if not self.som_habilitado:
            return

        agora = time.time()
        if agora - self.ultimo_som_tocado >= self.intervalo_min_som:
            try:
                # Prepara sons na primeira execução
                if not hasattr(self, 'som_objeto_encontrado'):
                    self._preparar_sons()

                # Toca som em thread separada para não bloquear
                threading.Thread(target=self._tocar_som_thread,
                               args=(self.som_objeto_encontrado, "objeto"),
                               daemon=True).start()
                self.ultimo_som_tocado = agora
                rospy.loginfo("🔊 Som: Objeto encontrado!")
            except Exception as e:
                rospy.logwarn(f"Erro ao tocar som: {e}")

    def tocar_som_missao_completa(self):
        """Toca som quando missão é completada"""
        if not self.som_habilitado:
            return

        try:
            # Prepara sons na primeira execução
            if not hasattr(self, 'som_missao_completa'):
                self._preparar_sons()

            # Toca som em thread separada
            threading.Thread(target=self._tocar_som_thread,
                           args=(self.som_missao_completa, "missao"),
                           daemon=True).start()
            rospy.loginfo("🎵 Som: Missão completa!")
        except Exception as e:
            rospy.logwarn(f"Erro ao tocar som: {e}")

    def _tocar_som_thread(self, arquivo_som, tipo_som):
        """Toca som em thread separada"""
        try:
            if arquivo_som and os.path.exists(arquivo_som) and self.comando_som:
                # Toca arquivo de som
                cmd = [self.comando_som, arquivo_som]
                subprocess.run(cmd, capture_output=True, check=True)
            else:
                # Fallback para beep do sistema
                self._beep_sistema(tipo_som)

        except Exception as e:
            rospy.logwarn(f"Erro ao tocar som via {self.comando_som}: {e}")
            # Fallback para beep do sistema
            self._beep_sistema(tipo_som)

    def _beep_sistema(self, tipo_som):
        """Beep usando recursos do sistema como fallback"""
        try:
            if tipo_som == "objeto":
                # Beep curto para objeto encontrado
                if shutil.which('beep'):
                    subprocess.run(['beep', '-f', '800', '-l', '300'], capture_output=True)
                else:
                    # Fallback para printf/echo
                    subprocess.run(['bash', '-c', 'echo -e "\\007"'], capture_output=True)

            elif tipo_som == "missao":
                # Sequência de beeps para missão completa
                if shutil.which('beep'):
                    for freq in [523, 659, 784, 1047]:
                        subprocess.run(['beep', '-f', str(freq), '-l', '200'], capture_output=True)
                        time.sleep(0.1)
                else:
                    # Múltiplos beeps
                    for _ in range(4):
                        subprocess.run(['bash', '-c', 'echo -e "\\007"'], capture_output=True)
                        time.sleep(0.3)

        except Exception as e:
            rospy.logwarn(f"Erro no beep do sistema: {e}")

    def finalizar(self):
        """Finaliza o sistema de som"""
        # Nada específico para finalizar nesta implementação
        pass

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
        self.gerenciador_som = GerenciadorSom()  # Novo componente de som
        self.bridge = CvBridge()

        # Parâmetros
        self.distancia_desejada = rospy.get_param('~distancia_desejada', 0.5)
        self.timeout_imagem = rospy.Duration(rospy.get_param('~timeout_imagem', 2.0))
        self.timeout_missao = rospy.Duration(rospy.get_param('~timeout_missao', 300.0))
        self.debug_ativo = rospy.get_param('~debug', False)

        # Estado
        self.estado = EstadoRobo.EXPLORANDO
        self.estado_anterior = EstadoRobo.EXPLORANDO
        self.centro_cor = None
        self.largura_imagem = 640  # Será atualizado
        self.distancia_alvo = np.inf
        self.info_obstaculos = {}
        self.objeto_ja_encontrado = False  # Flag para controlar som

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

        # Handler para shutdown limpo
        rospy.on_shutdown(self.shutdown_handler)

        self.rate = rospy.Rate(10)

        rospy.loginfo(f"Robô iniciado. Procurando por cor: {cor}")

    def shutdown_handler(self):
        """Handler para shutdown limpo"""
        rospy.loginfo("Finalizando robô...")
        self.controlador.parar()
        self.gerenciador_som.finalizar()

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

                # Toca som quando objeto é encontrado pela primeira vez
                if not self.objeto_ja_encontrado:
                    self.gerenciador_som.tocar_som_objeto_encontrado()
                    self.objeto_ja_encontrado = True

            else:
                self.centro_cor = None
                # Reset flag se perdeu objeto por muito tempo
                if len(self.historico_deteccoes) >= self.max_historico:
                    if sum(self.historico_deteccoes) == 0:
                        self.objeto_ja_encontrado = False

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
        self.estado_anterior = self.estado
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
                # Toca som de missão completa
                self.gerenciador_som.tocar_som_missao_completa()
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

        # Indicador de som
        if self.objeto_ja_encontrado:
            cv2.putText(debug_img, "🔊 OBJETO ENCONTRADO!",
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

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
