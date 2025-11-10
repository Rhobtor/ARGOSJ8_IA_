#!/usr/bin/env python3
import sys
import time
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from argj801_ctl_platform_interfaces.msg import CmdThrottleMsg


HELP = """
Controles (terminal activo):
  W / ↑ : acelerar (+throttle)
  S / ↓ : frenar    (-throttle)
  A / ← : girar izq (steering -)
  D / → : girar der (steering +)
  ESPACIO: parar (throttle=0, steering=0)
  H: ayuda
  Q: salir

Consejo: mantén esta terminal enfocada. Publicando en /ARGJ801/cmd_throttle_msg
"""

# Secuencias de flechas (ANSI)
ARROW_UP    = "\x1b[A"
ARROW_DOWN  = "\x1b[B"
ARROW_RIGHT = "\x1b[C"
ARROW_LEFT  = "\x1b[D"


class RawTerminal:
    """Pone stdin en modo raw y lo restaura al salir."""
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

    def __enter__(self):
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


def read_key_nonblocking():
    """Lee una tecla sin bloquear. Devuelve None si no hay nada.
    Soporta flechas (devuelve '\x1b[A', etc.)"""
    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    ch1 = sys.stdin.read(1)
    if ch1 == '\x1b':
        # posible secuencia ESC[
        if select.select([sys.stdin], [], [], 0)[0]:
            ch2 = sys.stdin.read(1)
            if ch2 == '[' and select.select([sys.stdin], [], [], 0)[0]:
                ch3 = sys.stdin.read(1)
                return '\x1b[' + ch3
        return '\x1b'
    return ch1


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        # Parámetros
        self.topic = self.declare_parameter('cmd_topic', '/ARGJ801/cmd_throttle_msg').get_parameter_value().string_value
        self.rate_hz = float(self.declare_parameter('rate_hz', 20.0).value)
        self.step_throttle = float(self.declare_parameter('step_throttle', 5.0).value)   # % por pulsación
        self.step_steering = float(self.declare_parameter('step_steering', 5.0).value)   # % por pulsación
        self.max_throttle = float(self.declare_parameter('max_throttle', 100.0).value)   # % saturación
        self.max_steering = float(self.declare_parameter('max_steering', 100.0).value)   # % saturación
        self.auto_decay = bool(self.declare_parameter('auto_decay', False).value)        # si True, decae hacia 0
        self.decay_throttle_per_s = float(self.declare_parameter('decay_throttle_per_s', 0.0).value)
        self.decay_steering_per_s = float(self.declare_parameter('decay_steering_per_s', 0.0).value)
        self.print_every_s = float(self.declare_parameter('print_every_s', 0.5).value)

        self.pub = self.create_publisher(CmdThrottleMsg, self.topic, 10)

        self.throttle = 0.0  # [%]
        self.steering = 0.0  # [%]
        self._last_print = 0.0

        self.get_logger().info(f"Iniciado control por teclado. Publicando en {self.topic}")
        self.get_logger().info(HELP.strip())

        period = 1.0 / max(1e-3, self.rate_hz)
        self.timer = self.create_timer(period, self._tick)
        self._last_time = time.monotonic()

    def _tick(self):
        # 1) Leer todas las teclas pendientes
        key_pressed = False
        while True:
            k = read_key_nonblocking()
            if k is None:
                break
            key_pressed = True
            self._handle_key(k)

        # 2) Decaimiento opcional hacia 0
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        if self.auto_decay and dt > 0:
            self._apply_decay(dt)

        # 3) Publicar comando
        self._publish_cmd()

        # 4) Info de estado
        if key_pressed or (now - self._last_print) >= self.print_every_s:
            self._last_print = now
            self.get_logger().info(f"Throttle: {self.throttle:5.1f}% | Steering: {self.steering:5.1f}%")

    def _apply_decay(self, dt):
        # mueve throttle/steering linealmente hacia 0
        def decay(value, rate_per_s):
            if rate_per_s <= 0:
                return value
            delta = rate_per_s * dt
            if value > 0:
                value = max(0.0, value - delta)
            elif value < 0:
                value = min(0.0, value + delta)
            return value

        self.throttle = decay(self.throttle, self.decay_throttle_per_s)
        self.steering = decay(self.steering, self.decay_steering_per_s)

    def _handle_key(self, k: str):
        k_low = k.lower()
        if k_low in ('q',):   # salir
            self.get_logger().info("Saliendo por 'q'…")
            rclpy.shutdown()
            return

        if k_low in ('h',):
            self.get_logger().info(HELP.strip())
            return

        if k_low == ' ':
            self.throttle = 0.0
            self.steering = 0.0
            return

        # Aceleración / frenado
        if k_low in ('w', ARROW_UP):
            self.throttle = min(self.max_throttle, self.throttle + self.step_throttle)
        elif k_low in ('s', ARROW_DOWN):
            self.throttle = max(-self.max_throttle, self.throttle - self.step_throttle)

        # Giro
        if k_low in ('a', ARROW_LEFT):
            self.steering = max(-self.max_steering, self.steering - self.step_steering)
        elif k_low in ('d', ARROW_RIGHT):
            self.steering = min(self.max_steering, self.steering + self.step_steering)

    def _publish_cmd(self):
        msg = CmdThrottleMsg()
        msg.throttle = float(self.throttle)
        msg.steering = float(self.steering)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        # terminal en modo raw para captar teclas al vuelo
        with RawTerminal():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
