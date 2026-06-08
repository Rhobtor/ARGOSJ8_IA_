"""video_tab.py – Pestaña de vídeo limpia e independiente.

Diseño:
  - Polling HTTP en un hilo de fondo por cada fuente activa.
  - Resultado enviado al hilo principal mediante una Qt Signal (thread-safe).
  - showEvent / hideEvent detienen y arrancan los pollers automáticamente:
    cuando la pestaña no está visible no hay red ni CPU extra, y no
    puede interferir con otras pestañas (p. ej. Personas).
"""
from __future__ import annotations

import threading
import time

import requests
from PySide6.QtCore import Qt, Signal, QObject
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

# Fuentes disponibles: clave → (etiqueta, ruta relay)
_SOURCES: dict[str, tuple[str, str]] = {
    'rgb_original':  ('RGB original',     '/snapshot/rgb_original'),
    'thermal':       ('Térmica original', '/snapshot/thermal'),
    'rgb_annotated': ('RGB anotada',      '/snapshot/rgb_overlay'),
}
_ALL_THREE = 'all_three'


# ---------------------------------------------------------------------------
class _Poller(QObject):
    """Hilo de polling HTTP para una única fuente de imagen."""

    # Señal emitida desde el hilo de fondo → recibida en el hilo principal.
    frame_ready: Signal = Signal(str, QImage)   # (source_key, imagen)

    def __init__(self, key: str, url: str, fps: float = 15.0):
        super().__init__()
        self._key = key
        self._url = url
        self._interval = max(1.0 / fps, 0.033)
        self._stop_event = threading.Event()
        self._session = requests.Session()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name=f'video-poll-{key}'
        )
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        try:
            self._session.close()
        except Exception:
            pass

    def _loop(self):
        while not self._stop_event.is_set():
            try:
                r = self._session.get(self._url, timeout=(0.5, 1.5))
                if r.status_code == 200:
                    img = QImage.fromData(r.content)
                    if not img.isNull():
                        self.frame_ready.emit(self._key, img)
            except Exception:
                pass
            time.sleep(self._interval)


# ---------------------------------------------------------------------------
class VideoTabWidget(QWidget):
    """Pestaña de vídeo – selector de fuente + visualización de imágenes.

    Uso desde MainWindow:
        self.video_tab = VideoTabWidget(relay_base_url=...)
        self.tabs.addTab(self.video_tab, 'Video')

    Para actualizar la URL del relay cuando cambia el robot:
        self.video_tab.set_relay_url(nueva_url)
    """

    def __init__(self, relay_base_url: str = 'http://127.0.0.1:8080', parent=None):
        super().__init__(parent)
        self._relay_base = relay_base_url.rstrip('/')
        self._pollers: dict[str, _Poller] = {}
        self._selected = 'rgb_original'

        # ── barra de controles ──────────────────────────────────
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(4)

        bar = QHBoxLayout()
        bar.addWidget(QLabel('Fuente:'))
        self.cmb = QComboBox()
        for key, (label, _) in _SOURCES.items():
            self.cmb.addItem(label, userData=key)
        self.cmb.addItem('Las tres a la vez', userData=_ALL_THREE)
        self.cmb.currentIndexChanged.connect(self._on_source_changed)
        bar.addWidget(self.cmb)
        bar.addStretch(1)
        self._lbl_status = QLabel('Sin señal')
        bar.addWidget(self._lbl_status)
        root.addLayout(bar)

        # ── panel imagen única ──────────────────────────────────
        self._single_panel = QWidget()
        sl = QVBoxLayout(self._single_panel)
        sl.setContentsMargins(0, 0, 0, 0)
        self._lbl_single = QLabel(alignment=Qt.AlignCenter)
        self._lbl_single.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._lbl_single.setMinimumSize(320, 240)
        self._lbl_single.setStyleSheet('background:#111;')
        sl.addWidget(self._lbl_single)
        root.addWidget(self._single_panel)

        # ── panel triple imagen ─────────────────────────────────
        self._triple_panel = QWidget()
        tl = QHBoxLayout(self._triple_panel)
        tl.setContentsMargins(0, 0, 0, 0)
        tl.setSpacing(6)
        self._triple_labels: dict[str, QLabel] = {}
        for key, (label, _) in _SOURCES.items():
            col_w = QWidget()
            col = QVBoxLayout(col_w)
            col.setContentsMargins(0, 0, 0, 0)
            col.setSpacing(2)
            title = QLabel(label, alignment=Qt.AlignCenter)
            title.setStyleSheet('color:#ccc; font-size:11px;')
            img_lbl = QLabel(alignment=Qt.AlignCenter)
            img_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            img_lbl.setMinimumSize(240, 180)
            img_lbl.setStyleSheet('background:#111;')
            col.addWidget(title)
            col.addWidget(img_lbl)
            tl.addWidget(col_w)
            self._triple_labels[key] = img_lbl
        self._triple_panel.hide()
        root.addWidget(self._triple_panel)

    # ── API pública ───────────────────────────────────────────────
    def set_relay_url(self, url: str):
        """Actualiza la URL base del relay (p. ej. al cambiar el robot)."""
        new_base = url.rstrip('/')
        if new_base == self._relay_base:
            return
        self._relay_base = new_base
        # Solo reiniciar pollers si la pestaña está visible.
        if self._pollers:
            self._stop_all_pollers()
            self._start_pollers_for(self._selected)

    def shutdown(self):
        """Llamar al cerrar la ventana principal."""
        self._stop_all_pollers()

    # ── ciclo de vida Qt (tab visible / oculto) ───────────────────
    def showEvent(self, ev):
        """La pestaña se vuelve visible → arrancar los pollers."""
        super().showEvent(ev)
        self._start_pollers_for(self._selected)

    def hideEvent(self, ev):
        """La pestaña se oculta → parar los pollers (sin red, sin CPU)."""
        self._stop_all_pollers()
        super().hideEvent(ev)

    # ── slots internos ────────────────────────────────────────────
    def _on_source_changed(self, _idx: int):
        sel = self.cmb.currentData()
        if not sel or sel == self._selected:
            return
        self._selected = sel
        if sel == _ALL_THREE:
            self._single_panel.hide()
            self._triple_panel.show()
        else:
            self._triple_panel.hide()
            self._single_panel.show()
        # Reiniciar pollers solo si la pestaña está activa.
        if self.isVisible():
            self._stop_all_pollers()
            self._start_pollers_for(sel)

    def _on_frame_ready(self, key: str, img: QImage):
        """Slot en el hilo principal: actualiza el label correspondiente."""
        pix = QPixmap.fromImage(img)
        if self._selected == _ALL_THREE:
            lbl = self._triple_labels.get(key)
            if lbl and not lbl.size().isEmpty():
                lbl.setPixmap(
                    pix.scaled(lbl.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                )
        else:
            if not self._lbl_single.size().isEmpty():
                self._lbl_single.setPixmap(
                    pix.scaled(
                        self._lbl_single.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                    )
                )
        label = _SOURCES.get(key, (key,))[0]
        self._lbl_status.setText(f'{label}: OK')

    # ── helpers ───────────────────────────────────────────────────
    def _url_for(self, key: str) -> str:
        return self._relay_base + _SOURCES[key][1]

    def _make_poller(self, key: str) -> _Poller:
        p = _Poller(key, self._url_for(key))
        p.frame_ready.connect(self._on_frame_ready)
        return p

    def _stop_all_pollers(self):
        for p in self._pollers.values():
            p.stop()
        self._pollers.clear()

    def _start_pollers_for(self, selection: str):
        self._stop_all_pollers()
        if selection == _ALL_THREE:
            for key in _SOURCES:
                self._pollers[key] = self._make_poller(key)
        else:
            self._pollers[selection] = self._make_poller(selection)
