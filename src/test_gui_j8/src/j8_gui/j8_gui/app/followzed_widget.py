from PySide6 import QtCore
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox, QPlainTextEdit
from PySide6.QtGui import QImage, QPixmap
import numpy as np
import rclpy
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import qos_profile_sensor_data



class FollowZEDWidget(QWidget):
    image_signal = QtCore.Signal(QImage)

    def __init__(self, node, topic_base='/follow_zed/image_for_gui'):
        super().__init__()
        self.node = node
        self.topic_raw = topic_base
        self.topic_comp = topic_base + '/compressed'


        lay = QVBoxLayout(self)
        hdr = QHBoxLayout()
        self.status = QLabel('FollowZED: esperando…')
        self.fit = QCheckBox('Ajustar a panel')
        self.fit.setChecked(True)
        hdr.addWidget(self.status)
        hdr.addStretch(1)
        hdr.addWidget(self.fit)
        lay.addLayout(hdr)


        self.view = QLabel(alignment=QtCore.Qt.AlignCenter)
        self.view.setMinimumSize(320, 240)
        lay.addWidget(self.view)


        self.logs = QPlainTextEdit(); self.logs.setReadOnly(True); self.logs.setMaximumHeight(120)
        lay.addWidget(self.logs)


        self.image_signal.connect(self._on_qimage)
        self._sub_raw = None
        self._sub_comp = None
        self._subscribe()

    def log(self, s):
        try:
            self.logs.appendPlainText(s)
        except Exception:
            pass

    def _subscribe(self):
        try:
            self._sub_raw = self.node.create_subscription(Image, self.topic_raw, self._on_raw, qos_profile_sensor_data)
            self.status.setText(f'FollowZED: sub a {self.topic_raw}')
        except Exception:
            try:
                self._sub_comp = self.node.create_subscription(CompressedImage, self.topic_comp, self._on_comp, qos_profile_sensor_data)
                self.status.setText(f'FollowZED: sub a {self.topic_comp}')
            except Exception as e:
                self.status.setText(f'FollowZED: error sub ({e})')

    def _on_raw(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            enc = msg.encoding.lower()
            if enc == 'rgb8':
                # QImage espera RGB, perfecto
                qimg = QImage(arr.data, msg.width, msg.height, msg.width * arr.shape[2], QImage.Format_RGB888).copy()
            elif enc == 'bgr8':
                arr = arr[..., ::-1].copy() # BGR->RGB
                qimg = QImage(arr.data, msg.width, msg.height, msg.width * arr.shape[2], QImage.Format_RGB888)
            else:
                self.status.setText(f'FollowZED: encoding no soportado {msg.encoding}')
                return
            self.image_signal.emit(qimg)
            self.status.setText(f'FollowZED: frame {msg.width}x{msg.height}')
        except Exception as e:
            self.status.setText(f'FollowZED: error {e}')

    def _on_comp(self, msg: CompressedImage):
        try:
            import cv2
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_bgr is None:
                return
            cv_rgb = cv_bgr[..., ::-1] # BGR->RGB
            h, w, ch = cv_rgb.shape
            qimg = QImage(cv_rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
            self.image_signal.emit(qimg)
        except Exception as e:
            self.status.setText(f'FollowZED: error {e}')

    def _on_qimage(self, qimg: QImage):
        if self.fit.isChecked():
            pix = QPixmap.fromImage(qimg).scaled(self.view.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        else:
            pix = QPixmap.fromImage(qimg)
        self.view.setPixmap(pix)


    def resizeEvent(self, ev):
        pm = self.view.pixmap()
        if pm and self.fit.isChecked():
            self.view.setPixmap(pm.scaled(self.view.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        super().resizeEvent(ev)