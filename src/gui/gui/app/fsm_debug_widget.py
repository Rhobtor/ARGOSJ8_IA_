import json
import os
import subprocess
import threading

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QLineEdit,
    QPlainTextEdit,
)


class FsmDebugWidget(QWidget):
    statusMessage = Signal(str)
    transitionsLoaded = Signal(list)
    currentModeLoaded = Signal(str)

    def __init__(self, namespace: str = 'ARGJ801', parent=None):
        super().__init__(parent)
        self._namespace = str(namespace or 'ARGJ801').strip().strip('/') or 'ARGJ801'
        self._build_ui()
        self.statusMessage.connect(self._append_log)
        self.transitionsLoaded.connect(self._apply_transitions)
        self.currentModeLoaded.connect(self._apply_current_mode)
        self._refresh_snapshot()

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)

        ns_row = QHBoxLayout()
        self.ed_namespace = QLineEdit(self._namespace)
        self.lbl_mode = QLabel('Modo actual: ?')
        self.btn_refresh_mode = QPushButton('Leer modo')
        self.btn_refresh_mode.clicked.connect(self.refresh_mode)
        self.btn_refresh_transitions = QPushButton('Leer transiciones')
        self.btn_refresh_transitions.clicked.connect(self.refresh_transitions)
        ns_row.addWidget(QLabel('Namespace:'))
        ns_row.addWidget(self.ed_namespace, 1)
        ns_row.addWidget(self.btn_refresh_mode)
        ns_row.addWidget(self.btn_refresh_transitions)
        root.addLayout(ns_row)

        action_row = QHBoxLayout()
        self.cmb_transition = QComboBox()
        self.cmb_transition.addItem('Sin datos', userData=None)
        self.btn_send_transition = QPushButton('Cambiar FSM')
        self.btn_send_transition.clicked.connect(self.send_transition)
        action_row.addWidget(QLabel('Transicion:'))
        action_row.addWidget(self.cmb_transition, 1)
        action_row.addWidget(self.btn_send_transition)
        action_row.addWidget(self.lbl_mode)
        root.addLayout(action_row)

        self.txt_log = QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setPlaceholderText('Logs de la pestaña FSM aislada...')
        root.addWidget(self.txt_log, 1)

    def set_namespace(self, namespace: str):
        normalized = str(namespace or '').strip().strip('/') or 'ARGJ801'
        if normalized == self._namespace:
            return
        self._namespace = normalized
        self.ed_namespace.setText(normalized)
        self._append_log(f'[FSM Debug] namespace -> {normalized}')
        self._refresh_snapshot()

    def _effective_namespace(self) -> str:
        return str(self.ed_namespace.text() or '').strip().strip('/') or self._namespace or 'ARGJ801'

    @staticmethod
    def _mode_label(mode: int) -> str:
        labels = {
            0: 'Ready',
            1: 'PathFollowing',
            2: 'Teleoperation',
            3: 'GoingHome',
            4: 'EmergencyStop',
            5: 'RecordPath',
            6: 'FollowZED',
            7: 'External cmd_vel',
        }
        return labels.get(int(mode), f'FSM {int(mode)}')

    @staticmethod
    def _transition_label(transition_id: int) -> str:
        labels = {
            0: 'Ready -> PathFollowing',
            1: 'PathFollowing -> Ready',
            2: 'Ready -> Teleoperation',
            3: 'Teleoperation -> Ready',
            4: 'Ready -> GoingHome',
            5: 'GoingHome -> Ready',
            6: 'Ready -> RecordPath',
            7: 'RecordPath -> Ready',
            8: 'E-Stop -> Ready',
            9: 'All -> E-Stop',
            10: 'Ready -> FollowZED',
            11: 'FollowZED -> Ready',
            12: 'Ready -> External cmd_vel',
            13: 'External cmd_vel -> Ready',
        }
        return labels.get(int(transition_id), f'Transition {int(transition_id)}')

    def _append_log(self, message: str):
        self.txt_log.appendPlainText(str(message))

    def _run_command(self, args, on_success):
        env = os.environ.copy()

        def _worker():
            try:
                self.statusMessage.emit(f"[FSM Debug] $ {' '.join(args)}")
                proc = subprocess.run(
                    args,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=8,
                    env=env,
                )
            except Exception as exc:
                self.statusMessage.emit(f'[FSM Debug] command failed: {exc}')
                return

            stdout = (proc.stdout or '').strip()
            stderr = (proc.stderr or '').strip()
            if stdout:
                self.statusMessage.emit(stdout)
            if stderr:
                self.statusMessage.emit(stderr)
            if proc.returncode != 0:
                self.statusMessage.emit(f'[FSM Debug] exit code {proc.returncode}')
                return

            try:
                on_success(stdout)
            except Exception as exc:
                self.statusMessage.emit(f'[FSM Debug] parse failed: {exc}')

        threading.Thread(target=_worker, daemon=True).start()

    def _run_python_helper(self, helper_code: str, helper_args, on_success):
        args = ['python3', '-c', helper_code, *helper_args]
        self._run_command(args, on_success)

    @staticmethod
    def _topic_snapshot_helper() -> str:
        return """
import json
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray

namespace = sys.argv[1].strip().strip('/') or 'ARGJ801'
rclpy.init()
node = Node('fsm_debug_topic_helper')
payload = {'mode': None, 'transitions': None}

node.create_subscription(Int32, f'/{namespace}/fsm_mode', lambda msg: payload.__setitem__('mode', int(msg.data)), 10)
node.create_subscription(Int32MultiArray, f'/{namespace}/possible_transitions', lambda msg: payload.__setitem__('transitions', list(msg.data)), 10)

deadline = time.time() + 3.0
while time.time() < deadline and (payload['mode'] is None or payload['transitions'] is None):
    rclpy.spin_once(node, timeout_sec=0.1)

print(json.dumps(payload))
node.destroy_node()
rclpy.shutdown()
"""

    @staticmethod
    def _change_mode_helper() -> str:
        return """
import json
import sys
import rclpy
from rclpy.node import Node
from ctl_mission_interfaces.srv import ChangeMode

namespace = sys.argv[1].strip().strip('/') or 'ARGJ801'
transition = int(sys.argv[2])

rclpy.init()
node = Node('fsm_debug_change_helper')
client = node.create_client(ChangeMode, f'/{namespace}/change_fsm')

result = {'service_ready': client.wait_for_service(timeout_sec=2.0), 'done': False, 'success': None}
if result['service_ready']:
    future = client.call_async(ChangeMode.Request(transition=transition))
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result['done'] = future.done()
    if future.done() and future.result() is not None:
        result['success'] = bool(future.result().success)

print(json.dumps(result))
node.destroy_node()
rclpy.shutdown()
"""

    def refresh_mode(self):
        self._refresh_snapshot()

    def refresh_transitions(self):
        namespace = self._effective_namespace()
        self._run_python_helper(self._topic_snapshot_helper(), [namespace], self._parse_topic_snapshot)

    def _refresh_snapshot(self):
        namespace = self._effective_namespace()
        self._run_python_helper(self._topic_snapshot_helper(), [namespace], self._parse_topic_snapshot)

    def _parse_topic_snapshot(self, stdout: str):
        payload = json.loads(stdout.strip() or '{}')
        mode = payload.get('mode', None)
        transitions = payload.get('transitions', None)
        if isinstance(mode, int):
            self.currentModeLoaded.emit(f'Modo actual: {self._mode_label(mode)} ({mode})')
        else:
            self.currentModeLoaded.emit('Modo actual: ?')
        filtered = [int(value) for value in (transitions or []) if int(value) >= 0]
        self.transitionsLoaded.emit(filtered)

    def send_transition(self):
        transition = self.cmb_transition.currentData()
        if not isinstance(transition, int):
            self._append_log('[FSM Debug] no hay transicion valida seleccionada')
            return

        namespace = self._effective_namespace()
        def _parse(stdout: str):
            payload = json.loads(stdout.strip() or '{}')
            success = bool(payload.get('success', False))
            done = bool(payload.get('done', False))
            ready = bool(payload.get('service_ready', False))
            self.statusMessage.emit(
                f'[FSM Debug] transition {transition} -> ready={ready} done={done} success={success}'
            )
            self._refresh_snapshot()

        self._run_python_helper(self._change_mode_helper(), [namespace, str(int(transition))], _parse)

    def _apply_transitions(self, transitions):
        self.cmb_transition.blockSignals(True)
        self.cmb_transition.clear()
        if not transitions:
            self.cmb_transition.addItem('Sin transiciones validas', userData=None)
        else:
            for transition in transitions:
                self.cmb_transition.addItem(
                    f'{int(transition)} - {self._transition_label(int(transition))}',
                    userData=int(transition),
                )
        self.cmb_transition.blockSignals(False)
        self._append_log(f'[FSM Debug] transiciones -> {transitions}')

    def _apply_current_mode(self, text: str):
        self.lbl_mode.setText(text)
        self._append_log(f'[FSM Debug] {text}')