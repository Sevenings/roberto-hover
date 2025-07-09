# Roberto Hover

## Configuração de Rede
O robô se conecta ao computador via cabo ethernet, que envia comandos a todas as partes do robô através do gateway,
pelo protocolo ROS Serial. No entanto, o robo procura o servidor do ROS Serial em um IP específico, então é 
necessário configurar o IP do computador para que ele possa ser encontrado pelo robô.

IP Fixo, sem DHCP
Ipv4: 10.10.10.1
Sub-rede: 255.255.255.0
Gateway: 10.10.10.199

## Dependências

Instalar o rosserial:

```bash
sudo apt install ros-noetic-rosserial-python
```

Instalar o telop:
```bash
sudo apt install ros-noetic-teleop-twist-keyboard
```

## Comandos
---

## Executar o ROSCore

```bash
roscore
```


```bash
rosrun rosserial_python serial_node.py _port:=tcp
```

### Executar o Teleop

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Ver bateria

```bash
rostopic echo /battery_voltage
```
