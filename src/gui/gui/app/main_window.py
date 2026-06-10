import os
import re
import json
import math
import threading
import time
import mimetypes
import http.server
import socketserver

import requests

from ament_index_python.packages import get_package_share_directory

from PySide6.QtCore import Qt, QUrl, QTimer, Signal
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QLineEdit, QFormLayout, QTabWidget, QTableWidget, QTableWidgetItem,
    QSplitter, QComboBox, QFileDialog, QCheckBox
)
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWebEngineCore import (
    QWebEnginePage, QWebEngineSettings, QWebEngineProfile, QWebEngineUrlRequestInterceptor
)
from PySide6.QtWebChannel import QWebChannel
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QTransform
from PySide6.QtCore import QSize


from PySide6.QtWidgets import QSizePolicy, QGridLayout
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path, Odometry

from gui.app.map_bridge import MapBridge
from gui.app.state_widget import MissionWidget
from gui.app.control_widget import ControlWidget
from gui.app.person_manager_widget import PersonsWidget
from gui.app.video_tab import VideoTabWidget
from gui.app.fsm_debug_widget import FsmDebugWidget




# ========= entorno WebEngine =========
for k in ('http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY'):
    os.environ.pop(k, None)
os.environ.setdefault('NO_PROXY', '127.0.0.1,localhost')
flags = os.environ.get('QTWEBENGINE_CHROMIUM_FLAGS', '')
add = ' --disable-gpu --disable-dev-shm-usage --no-sandbox'
if add not in flags:
    os.environ['QTWEBENGINE_CHROMIUM_FLAGS'] = flags + add


STATES = [
    ('disarmed', 'DISARMED'),
    ('ready', 'Ready'),
    ('standby', 'Standby'),
    ('guided', 'Guided'),
    ('auto', 'Auto'),
    ('follow_zed', 'FollowZED'),
    ('control_ai', 'Control IA'),
    
]

ROBOT_ROLE_LABELS = [
    'explorador',
    'cargador',
    'rescatador',
    'mixto',
    'personalizado',
]

ROBOT_DISCOVERY_TOPIC_SUFFIXES = (
    '/fsm_mode',
    '/possible_transitions',
)

ROBOT_DISCOVERY_SERVICE_SUFFIXES = (
    '/change_fsm',
    '/get_fsm_mode',
    '/get_possible_transitions',
    '/receive_ll_path',
)

DEFAULT_PRIMARY_NAMESPACE = 'ARGJ801'
DEFAULT_KNOWN_ROBOT_NAMESPACES = (
    'ARGJ801',
    'cuadriga',
)

ROBOT_POSE_FIX_TOPIC_SUFFIX = '/fixposition/navsatfix'
ROBOT_POSE_ODOM_TOPIC_SUFFIX = '/fixposition/odometry_enu'
ROBOT_POSE_GPS_VEL_TOPIC_SUFFIX = '/fixposition/navsatfix/vel'
REMOTE_PERSONS_LATLON_TOPIC_SUFFIX = '/detected_persons_latlon'
REMOTE_SARNET_POSITIONS_TOPIC_SUFFIX = '/sarnet/person_positions_robot'
REMOTE_SARNET_DISTANCES_TOPIC_SUFFIX = '/sarnet/detections'
GLOBAL_FIXPOSITION_FALLBACK_NAMESPACES = (DEFAULT_PRIMARY_NAMESPACE,)
ROBOT_MARKER_COLORS = (
    '#e53e3e',
    '#2563eb',
    '#16a34a',
    '#d97706',
    '#7c3aed',
    '#0891b2',
    '#db2777',
    '#65a30d',
)


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius_m = 6371000.0
    dlat = math.radians(float(lat2) - float(lat1))
    dlon = math.radians(float(lon2) - float(lon1))
    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(math.radians(float(lat1)))
        * math.cos(math.radians(float(lat2)))
        * math.sin(dlon / 2.0) ** 2
    )
    return 2.0 * radius_m * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def _offset_latlon(lat_deg: float, lon_deg: float, east_m: float, north_m: float):
    radius_m = 6378137.0
    lat_rad = math.radians(float(lat_deg))
    out_lat = float(lat_deg) + math.degrees(north_m / radius_m)
    cos_lat = max(math.cos(lat_rad), 1e-9)
    out_lon = float(lon_deg) + math.degrees(east_m / (radius_m * cos_lat))
    return out_lat, out_lon


def _latlon_to_local_offsets(robot_lat: float, robot_lon: float, target_lat: float, target_lon: float, heading_deg: float):
    radius_m = 6378137.0
    robot_lat_f = float(robot_lat)
    robot_lon_f = float(robot_lon)
    target_lat_f = float(target_lat)
    target_lon_f = float(target_lon)
    heading_rad = math.radians(float(heading_deg))

    north_m = math.radians(target_lat_f - robot_lat_f) * radius_m
    east_m = math.radians(target_lon_f - robot_lon_f) * radius_m * max(math.cos(math.radians(robot_lat_f)), 1e-9)

    forward_m = east_m * math.sin(heading_rad) + north_m * math.cos(heading_rad)
    left_m = -east_m * math.cos(heading_rad) + north_m * math.sin(heading_rad)
    return forward_m, left_m


def _quat_to_bearing_deg(qx: float, qy: float, qz: float, qw: float):
    siny_cosp = 2.0 * (float(qw) * float(qz) + float(qx) * float(qy))
    cosy_cosp = 1.0 - 2.0 * (float(qy) * float(qy) + float(qz) * float(qz))
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg_enu = math.degrees(yaw_rad) % 360.0
    return (90.0 - yaw_deg_enu) % 360.0


def _resolve_html_path():
    here = os.path.dirname(__file__)

    # Prefer the share tree colocated with the imported Python package.
    install_local = os.path.abspath(
        os.path.join(here, '..', '..', '..', '..', '..', 'share', 'gui', 'resources', 'map.html')
    )
    if os.path.exists(install_local):
        print(f"[GUI] map.html (module install) -> {install_local}")
        return install_local

    try:
        share = get_package_share_directory('gui')
        p = os.path.join(share, 'resources', 'map.html')
        if os.path.exists(p):
            print(f"[GUI] map.html (install) -> {p}")
            return p
    except Exception:
        pass

    p2 = os.path.abspath(os.path.join(here, '..', 'resources', 'map.html'))
    if os.path.exists(p2):
        print(f"[GUI] map.html (src) -> {p2}")
        return p2
    raise FileNotFoundError("No encuentro map.html")


class ConsolePage(QWebEnginePage):
    def javaScriptConsoleMessage(self, level, message, lineNumber, sourceID):
        try:
            name = getattr(level, "name", None) or str(level)
        except Exception:
            name = str(level)
        print(f"[JS {name}] {sourceID}:{lineNumber} -> {message}")


class HeaderInjector(QWebEngineUrlRequestInterceptor):
    def interceptRequest(self, info):
        # No tocamos localhost; solo dominios públicos si se usaran directos
        url = info.requestUrl().toString()
        if ("tile.openstreetmap.org" in url or "arcgisonline.com" in url or "esri.com" in url):
            info.setHttpHeader(b"Referer", b"https://localhost/")
            info.setHttpHeader(
                b"User-Agent",
                b"Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/122.0 Safari/537.36"
            )

class HUDWidget(QWidget):
    """
    HUD sencillo: cielo/tierra, línea de horizonte con pitch/roll, mira central y aviso DISARMED.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        # Estado
        self.pitch_deg = 0.0   # + arriba, - abajo
        self.roll_deg  = 0.0   # + derecha
        self.yaw_deg   = 0.0   # 0..360
        
        # Estado (texto y color)
        self.state_text = "DISARMED"
        self.state_color_override = QColor("#ff4444")  # <- fuerza ROJO para cualquier estado; pon None si quieres mapeo por estado
        self.state_colors = {
            "DISARMED": QColor(255, 80, 80),
            "READY": QColor(80, 200, 120),
            "STANDBY": QColor(255, 190, 60),
            "GUIDED": QColor(120, 190, 255),
            "FOLLOWPATH": QColor(120, 220, 255),
            "FOLLOWZED": QColor(140, 220, 180),
            "CONTROLIA": QColor(180, 160, 255),
        }
        # GPS status
        self.gps_fix = None    # puede ser int (ROS NavSatStatus o APM fix_type) o str
        self.gps_sats = None
        self.gps_hdop = None
        self.gps_label = "GPS: —"
        self.gps_color = QColor(255, 80, 80)  # por defecto rojo
        # Apariencia
        self._sky    = QColor(110, 170, 255)
        self._ground = QColor(170, 140, 100)
        self._lines  = QColor(255, 255, 255)

    def sizeHint(self) -> QSize:
        return QSize(600, 260)

    # --- NUEVO: setter del estado ---
    def set_state(self, text: str):
        t = (text or "").strip()
        self.state_text = t if t else "DISARMED"
        self.update()
    def set_state_color(self, color):  # acepta QColor o '#rrggbb' o None
        if color is None:
            self.state_color_override = None
        else:
            self.state_color_override = QColor(color) if not isinstance(color, QColor) else color
        self.update()

    def set_gps_status(self, fix=None, sats=None, hdop=None, label: str=None):
        """
        fix: puede ser
            - ROS sensor_msgs/NavSatStatus.status:  -1 NO_FIX, 0 FIX, 1 SBAS, 2 GBAS
            - APM/MAVLink fix_type: 0/1 no fix, 2 2D, 3 3D, 4 DGPS, 5 RTK Float, 6 RTK Fixed
            - o None/str
        """
        if sats is not None: self.gps_sats = int(sats)
        if hdop is not None: self.gps_hdop = float(hdop)

        text, color = None, QColor(200, 200, 200)
        if isinstance(fix, int):
            # Map ROS
            ros_map = {
                -1: ("No GPS", QColor(255, 80, 80)),
                0: ("3D Fix", QColor( 80, 220, 120)),
                1: ("SBAS",   QColor( 80, 200, 240)),
                2: ("GBAS",   QColor( 80, 200, 240)),
            }
            # Map APM (si los valores son de 0..6)
            apm_map = {
                0: ("No GPS",     QColor(255, 80, 80)),
                1: ("No Fix",     QColor(255, 80, 80)),
                2: ("2D Fix",     QColor(255, 190, 60)),
                3: ("3D Fix",     QColor( 80, 220, 120)),
                4: ("DGPS",       QColor( 80, 200, 240)),
                5: ("RTK Float",  QColor(160, 120, 255)),
                6: ("RTK Fixed",  QColor(140,  90, 255)),
            }
            if fix in apm_map:
                text, color = apm_map[fix]
            elif fix in ros_map:
                text, color = ros_map[fix]
            else:
                text, color = (f"Fix {fix}", QColor(200, 200, 200))
        elif isinstance(fix, str):
            text = fix
            color = QColor(255, 80, 80) if "NO" in fix.upper() else QColor(80, 220, 120)

        if label is not None:
            text = label

        # Confecciona el texto final con sats/hdop si hay
        parts = [f"GPS: {text or '—'}"]
        if self.gps_sats is not None: parts.append(f"Sats: {self.gps_sats}")
        if self.gps_hdop is not None: parts.append(f"HDOP: {self.gps_hdop:.1f}")
        self.gps_label = "  ".join(parts)
        self.gps_color = color
        self.update()
    # --- API pública ---
    def set_attitude(self, pitch: float=None, roll: float=None, yaw: float=None):
        if pitch is not None: self.pitch_deg = float(pitch)
        if roll  is not None: self.roll_deg  = float(roll)
        if yaw   is not None: self.yaw_deg   = float(yaw) % 360.0
        self.update()

    def set_armed(self, armed: bool):
        self.armed = bool(armed)
        self.update()

    # --- Pintado ---
    def paintEvent(self, ev):
        w = self.width()
        h = self.height()
        cx, cy = w / 2.0, h / 2.0

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        # ----- Cielo/Tierra + horizonte -----
        p.save()
        p.translate(cx, cy)
        p.rotate(-self.roll_deg)

        px_per_deg = h / 120.0
        dy = self.pitch_deg * px_per_deg

        p.setPen(Qt.NoPen)
        p.setBrush(self._sky)
        p.drawRect(-w, -h - dy, 2 * w, h + dy)   # cielo

        p.setBrush(self._ground)
        p.drawRect(-w, -dy, 2 * w, h + dy)       # tierra

        p.setPen(QPen(self._lines, 2))
        p.drawLine(-w, -dy, w, -dy)              # línea horizonte

        # Marcas de pitch
        p.setPen(QPen(self._lines, 1))
        for ang in range(-40, 50, 10):
            y = -(ang * px_per_deg) - dy
            if -h / 2 - 40 <= y <= h / 2 + 40:
                length = 50 if ang % 20 == 0 else 30
                p.drawLine(-length, y, length, y)
                if ang != 0:
                    p.drawText(length + 6, y + 4, f"{ang}")
                    p.drawText(-length - 26, y + 4, f"{ang}")
        p.restore()

        # ----- Mira central -----
        p.setPen(QPen(self._lines, 2))
        p.drawLine(cx - 18, cy, cx + 18, cy)
        p.drawLine(cx, cy - 12, cx, cy + 12)

        # ----- Rumbo (heading) -----
        p.setPen(QPen(self._lines, 1))
        f = QFont(self.font())
        f.setPointSizeF(f.pointSizeF() * 0.9)
        p.setFont(f)
        p.drawText(8, 18, f"HDG {self.yaw_deg:06.2f}°")

        # ----- Estado centrado (color forzado si quieres siempre rojo) -----
        if getattr(self, "state_text", ""):
            txt = str(self.state_text)
            key = txt.upper().replace("_", "").replace(" ", "")
            col = self.state_color_override or self.state_colors.get(key, QColor(255, 255, 255))

            f2 = QFont(self.font()); f2.setBold(True)
            f2.setPointSizeF(f2.pointSizeF() * 1.6)
            p.setFont(f2)
            p.setPen(QPen(col))
            p.drawText(0, 0, w, h, Qt.AlignCenter, txt)

        # ----- Cartela de GPS sobre la "tierra" (abajo-izquierda) -----
        if getattr(self, "gps_label", None):
            txt = self.gps_label
            f3 = QFont(self.font())
            f3.setPointSizeF(f3.pointSizeF() * 0.95)
            p.setFont(f3)

            metrics = p.fontMetrics()
            pad_x, pad_y = 10, 6
            tw = metrics.horizontalAdvance(txt)
            th = metrics.height()
            rect_w = tw + pad_x * 2
            rect_h = th + pad_y * 2

            # Colocar 12px desde el borde inferior-izquierdo
            rx, ry = 12, h - rect_h - 10

            # Fondo semitransparente
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(0, 0, 0, 140))
            p.drawRoundedRect(rx, ry, rect_w, rect_h, 8, 8)

            # Texto
            p.setPen(QPen(self.gps_color))
            p.drawText(rx + pad_x, ry + pad_y + metrics.ascent(), txt)





class StatTile(QWidget):
    def __init__(self, title: str, unit: str="", color: QColor=QColor("#66E")):
        super().__init__()
        self.title = QLabel(title)
        self.title.setStyleSheet("color:#BBB; font-size:12px;")
        self.value = QLabel("0.00")
        self.value.setStyleSheet(f"color:{color.name()}; font-weight:800; font-size:34px;")
        self.unit  = QLabel(unit)
        self.unit.setStyleSheet("color:#999; font-size:12px;")

        box = QVBoxLayout(self); box.setContentsMargins(10,8,10,8); box.setSpacing(2)
        box.addWidget(self.title)
        box.addWidget(self.value)
        box.addWidget(self.unit)

        self.setStyleSheet("background:#111; border-radius:10px;")
    
    def set_value(self, text: str):
        self.value.setText(text)

class StatsPanel(QWidget):
    """Cuadrícula 3x2 con tiles: Altitude, GroundSpeed, Dist to WP, Yaw, Vertical Speed, DistToMAV"""
    def __init__(self):
        super().__init__()
        grid = QGridLayout(self); grid.setContentsMargins(0,0,0,0); grid.setSpacing(8)

        self.t_alt   = StatTile("Altitude (m)",        "", QColor("#E88"))
        self.t_gspd  = StatTile("GroundSpeed (m/s)",   "", QColor("#6AE"))
        self.t_dwp   = StatTile("Dist to WP (m)",      "", QColor("#6AE"))
        self.t_yaw   = StatTile("Yaw (deg)",           "", QColor("#6E6"))
        self.t_vspd  = StatTile("Vertical Speed (m/s)", "", QColor("#6AE"))
        self.t_dmav  = StatTile("DistToMAV (m)",       "", QColor("#EE6"))

        tiles = [self.t_alt, self.t_gspd, self.t_dwp, self.t_yaw, self.t_vspd, self.t_dmav]
        for i, tile in enumerate(tiles):
            r, c = divmod(i, 3)
            grid.addWidget(tile, r, c)

        self.setStyleSheet("background:#0A0A0A; border:1px solid #222; border-radius:12px; padding:8px;")

    def update_stats(self, **kw):
        if "altitude" in kw:     self.t_alt.set_value(f"{kw['altitude']:.2f}")
        if "groundspeed" in kw:  self.t_gspd.set_value(f"{kw['groundspeed']:.2f}")
        if "dist_wp" in kw:      self.t_dwp.set_value(f"{kw['dist_wp']:.2f}")
        if "yaw" in kw:          self.t_yaw.set_value(f"{kw['yaw']:.2f}")
        if "vspeed" in kw:       self.t_vspd.set_value(f"{kw['vspeed']:.2f}")
        if "dist_mav" in kw:     self.t_dmav.set_value(f"{kw['dist_mav']:.2f}")



# ====== Proxy de tiles + servidor estático /ui/* ======
class _TileProxyHandler(http.server.BaseHTTPRequestHandler):
    ROOT_DIR = None  # se setea desde TileProxy.start(root_dir)

    UPSTREAMS = {
        'osm':  'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
        'esri': 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    }
    session = requests.Session()
    timeout = 15
    headers = {
        'User-Agent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 '
                      '(KHTML, like Gecko) Chrome/122.0 Safari/537.36',
        'Referer': 'https://localhost/'
    }
    _re_tiles = re.compile(r'^/(osm|esri)/(\d+)/(\d+)/(\d+)\.(png|jpg)$')

    def do_GET(self):
        path = self.path.split('?', 1)[0]

        # 1) /ui/* -> servir ficheros estáticos (map.html, vendor/leaflet/*, etc.)
        if path.startswith('/ui/'):
            if not self.ROOT_DIR:
                self.send_error(500, "ROOT_DIR no configurado"); return
            rel = path[len('/ui/'):]  # lo que venga detrás
            fs_path = os.path.normpath(os.path.join(self.ROOT_DIR, rel))
            # seguridad: que no escape del directorio raíz
            if not fs_path.startswith(os.path.abspath(self.ROOT_DIR)):
                self.send_error(403); return
            if not os.path.exists(fs_path):
                self.send_error(404); return
            if os.path.isdir(fs_path):
                self.send_error(403); return
            ctype, _ = mimetypes.guess_type(fs_path)
            if not ctype: ctype = 'application/octet-stream'
            try:
                with open(fs_path, 'rb') as f:
                    data = f.read()
                self.send_response(200)
                self.send_header('Content-Type', ctype)
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(data)
            except Exception as e:
                self.send_error(500, str(e))
            return

        # 2) /osm/... o /esri/... -> proxy a upstream
        m = self._re_tiles.match(path)
        if m:
            kind, z, x, y, ext = m.groups()
            url = self.UPSTREAMS[kind].format(z=z, x=x, y=y)
            try:
                r = self.session.get(url, headers=self.headers, timeout=self.timeout, stream=True)
                ct = r.headers.get('Content-Type', 'image/png')
                if r.status_code != 200:
                    print(f"[tiles] {url} -> HTTP {r.status_code}")
                    self.send_response(502)
                    self.send_header('Content-Type', 'text/plain; charset=utf-8')
                    self.end_headers()
                    self.wfile.write(f'Upstream {kind} {r.status_code}\n'.encode())
                    return
                self.send_response(200)
                self.send_header('Content-Type', ct)
                self.send_header('Cache-Control', 'public, max-age=86400')
                self.end_headers()
                for chunk in r.iter_content(8192):
                    if chunk:
                        self.wfile.write(chunk)
            except Exception as e:
                print(f"[tiles] ERROR {url}: {e}")
                self.send_response(502)
                self.send_header('Content-Type', 'text/plain; charset=utf-8')
                self.end_headers()
                self.wfile.write(f'Error fetching {url}: {e}\n'.encode())
            return

        # 3) cualquier otra ruta
        self.send_error(404)

    def log_message(self, *args, **kwargs):
        pass


class TileProxy:
    """ http://127.0.0.1:<port> sirve:
        - /ui/<archivos>  (map.html, vendor/leaflet/*)
        - /osm/{z}/{x}/{y}.png
        - /esri/{z}/{y}/{x}
    """
    def __init__(self, host='127.0.0.1', port=0, root_dir=None):
        self._host = host
        self._port = port
        self._server = None
        self.port = None
        self.thread = None
        self.root_dir = root_dir

    def start(self, root_dir=None):
        if root_dir:
            self.root_dir = root_dir
        if not self.root_dir:
            raise ValueError("TileProxy.start necesita root_dir para /ui/*")
        _TileProxyHandler.ROOT_DIR = os.path.abspath(self.root_dir)
        self._server = socketserver.ThreadingTCPServer((self._host, self._port), _TileProxyHandler)
        self._server.daemon_threads = True
        self.port = self._server.server_address[1]
        self.thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self.thread.start()
        print(f"[tiles] proxy+static en http://127.0.0.1:{self.port}  (root: {self.root_dir})")
        return self.port

    def stop(self):
        if self._server:
            self._server.shutdown()
            self._server.server_close()
            self._server = None


# ====== ROS ======
class RosSide(Node):
    def __init__(self, namespace: str = 'ARGJ801'):
        super().__init__('gui_node')
        self._path_control_mode = 'internal'
        self._fsm_request_watchdog_timeout_s = 1.5

        # --- Compatibilidad con GUI_pkg (parámetros y defaults) ---
        from gui.ros_api import cuadrigaGuiDefaults, cuadrigaGuiParams, namespaced

        p = cuadrigaGuiParams()
        d = cuadrigaGuiDefaults()

        # Nota: GUI_pkg fija el namespace internamente a 'ARGJ801'.
        # Aquí lo dejamos como parámetro, pero el default debe ser ARGJ801.
        self.declare_parameter('namespace', namespace or p.namespace)
        namespace = self.get_parameter('namespace').get_parameter_value().string_value or p.namespace
        self.namespace = str(namespace).strip().strip('/') or p.namespace

        # services (mismos parámetros que GUI_pkg)
        self.declare_parameter(p.fsm_change_fsm_mode_srv_name, d.fsm_change_fsm_mode_srv_name)
        self.declare_parameter(p.fsm_get_fsm_srv_name, d.fsm_get_fsm_srv_name)
        self.declare_parameter(p.fsm_get_possible_transition_srv_name, d.fsm_get_possible_transition_srv_name)
        self.declare_parameter(p.path_planner_srv_name, d.path_planner_srv_name)
        self.declare_parameter(p.receive_ll_path_srv_name, d.receive_ll_path_srv_name)
        self.declare_parameter(p.receive_external_path_srv_name, d.receive_external_path_srv_name)
        self.declare_parameter(p.read_path_service, d.read_path_service)
        self.declare_parameter(p.read_external_path_service, d.read_external_path_service)
        self.declare_parameter(p.write_path_service, d.write_path_service)
        self.declare_parameter(p.write_external_path_service, d.write_external_path_service)
        self.declare_parameter(p.return_path_service, d.return_path_service)
        self.declare_parameter(p.return_external_path_service, d.return_external_path_service)
        self.declare_parameter(p.external_path_command_topic_name, d.external_path_command_topic_name)
        self.declare_parameter(p.config_controller_srv_name, d.config_controller_srv_name)
        self.declare_parameter(p.change_controller_srv_name, d.change_controller_srv_name)
        self.declare_parameter(p.config_pure_pursuit_srv_name, d.config_pure_pursuit_srv_name)
        self.declare_parameter(p.config_stanley_srv_name, d.config_stanley_srv_name)
        self.declare_parameter(p.config_dynamic_pure_srv_name, d.config_dynamic_pure_srv_name)
        self.declare_parameter(p.config_dynamic_la_pure_srv_name, d.config_dynamic_la_pure_srv_name)
        self.declare_parameter(p.config_regulated_pure_srv_name, d.config_regulated_pure_srv_name)
        self.declare_parameter(p.enable_security_check_srv_name, d.enable_security_check_srv_name)
        self.declare_parameter(p.get_security_check_srv_name, d.get_security_check_srv_name)

        srv_change_fsm = namespaced(self.namespace, self.get_parameter(p.fsm_change_fsm_mode_srv_name).value)
        srv_get_state = namespaced(self.namespace, self.get_parameter(p.fsm_get_fsm_srv_name).value)
        srv_receive_ll_path = namespaced(self.namespace, self.get_parameter(p.receive_ll_path_srv_name).value)
        srv_receive_external_path = namespaced(self.namespace, self.get_parameter(p.receive_external_path_srv_name).value)

        # clients (equivalentes a GUI_pkg/ros_classes.py)
        from ctl_mission_interfaces.srv import ChangeMode, GetMode, GetPossibleTransitions
        from path_manager_interfaces.srv import RobotPath, ReadPathFromFile, WritePathToFile, ReturnRobotPath
        from std_msgs.msg import String as MsgString

        self._srv_ChangeMode = ChangeMode
        self._srv_GetMode = GetMode
        self._srv_GetPossibleTransitions = GetPossibleTransitions
        self._srv_RobotPath = RobotPath
        self._srv_ReadPathFromFile = ReadPathFromFile
        self._srv_WritePathToFile = WritePathToFile
        self._srv_ReturnRobotPath = ReturnRobotPath
        self._fsm_feedback_signals = None
        self._pending_service_futures = set()

        self.cli_change_fsm = self.create_client(ChangeMode, srv_change_fsm)
        self.cli_get_state = self.create_client(GetMode, srv_get_state)
        srv_get_possible_transitions = namespaced(self.namespace, self.get_parameter(p.fsm_get_possible_transition_srv_name).value)
        self.cli_get_possible_transitions = self.create_client(GetPossibleTransitions, srv_get_possible_transitions)
        self.cli_send_draw_path = self.create_client(RobotPath, srv_receive_ll_path)
        self.cli_send_external_path = self.create_client(RobotPath, srv_receive_external_path)

        srv_read_path = namespaced(self.namespace, self.get_parameter(p.read_path_service).value)
        srv_read_external_path = namespaced(self.namespace, self.get_parameter(p.read_external_path_service).value)
        srv_write_path = namespaced(self.namespace, self.get_parameter(p.write_path_service).value)
        srv_write_external_path = namespaced(self.namespace, self.get_parameter(p.write_external_path_service).value)
        srv_return_path = namespaced(self.namespace, self.get_parameter(p.return_path_service).value)
        srv_return_external_path = namespaced(self.namespace, self.get_parameter(p.return_external_path_service).value)
        self.cli_read_path = self.create_client(ReadPathFromFile, srv_read_path)
        self.cli_read_external_path = self.create_client(ReadPathFromFile, srv_read_external_path)
        self.cli_write_path = self.create_client(WritePathToFile, srv_write_path)
        self.cli_write_external_path = self.create_client(WritePathToFile, srv_write_external_path)
        self.cli_return_path = self.create_client(ReturnRobotPath, srv_return_path)
        self.cli_return_external_path = self.create_client(ReturnRobotPath, srv_return_external_path)
        topic_external_path_command = namespaced(self.namespace, self.get_parameter(p.external_path_command_topic_name).value)
        self.pub_external_path_command = self.create_publisher(MsgString, topic_external_path_command, 10)
        self._msg_String = MsgString

        # Controller config services (ctl_mission CtrlNode)
        from ctl_mission_interfaces.srv import (
            ChangeController,
            ConfigPurePursuitCtrl,
            ConfigRegulatedPureCtrl,
            ConfigDynamicPureCtrl,
            ConfigDynamicLAPureCtrl,
            ConfigStanleyCtrl,
        )

        # Names are parameters in CtrlNode, but the default launch normally sets them.
        # We assume the common service names under namespace.
        self._srv_ChangeController = ChangeController
        self._srv_ConfigPurePursuitCtrl = ConfigPurePursuitCtrl
        self._srv_ConfigRegulatedPureCtrl = ConfigRegulatedPureCtrl
        self._srv_ConfigDynamicPureCtrl = ConfigDynamicPureCtrl
        self._srv_ConfigDynamicLAPureCtrl = ConfigDynamicLAPureCtrl
        self._srv_ConfigStanleyCtrl = ConfigStanleyCtrl

        srv_change_ctrl = namespaced(self.namespace, self.get_parameter(p.change_controller_srv_name).value)
        srv_config_pp = namespaced(self.namespace, self.get_parameter(p.config_pure_pursuit_srv_name).value)
        srv_config_regulated = namespaced(self.namespace, self.get_parameter(p.config_regulated_pure_srv_name).value)
        srv_config_dynamic = namespaced(self.namespace, self.get_parameter(p.config_dynamic_pure_srv_name).value)
        srv_config_dynamic_la = namespaced(self.namespace, self.get_parameter(p.config_dynamic_la_pure_srv_name).value)
        srv_config_stanley = namespaced(self.namespace, self.get_parameter(p.config_stanley_srv_name).value)

        self.cli_change_controller = self.create_client(ChangeController, srv_change_ctrl)
        self.cli_config_pp = self.create_client(ConfigPurePursuitCtrl, srv_config_pp)
        self.cli_config_regulated = self.create_client(ConfigRegulatedPureCtrl, srv_config_regulated)
        self.cli_config_dynamic = self.create_client(ConfigDynamicPureCtrl, srv_config_dynamic)
        self.cli_config_dynamic_la = self.create_client(ConfigDynamicLAPureCtrl, srv_config_dynamic_la)
        self.cli_config_stanley = self.create_client(ConfigStanleyCtrl, srv_config_stanley)

    # NOTE: no /gui/* publishers. We talk to J8 via services (legacy GUI_pkg contract).

    def set_fsm_feedback_signals(self, signals):
        self._fsm_feedback_signals = signals

    def _track_future(self, future, callback=None):
        self._pending_service_futures.add(future)

        def _done(done_future):
            self._pending_service_futures.discard(done_future)
            if callback is not None:
                callback(done_future)

        future.add_done_callback(_done)
        return future

    def _emit_fsm_mode_feedback(self, mode: int):
        signals = self._fsm_feedback_signals
        if signals is None:
            return
        try:
            signals.fsm_mode.emit(int(mode))
        except Exception:
            pass

    def _emit_possible_transitions_feedback(self, transitions):
        signals = self._fsm_feedback_signals
        if signals is None:
            return
        try:
            payload = [int(value) for value in (transitions or [])]
        except Exception:
            payload = []
        try:
            signals.possible_transitions.emit(json.dumps(payload, separators=(',', ':')))
        except Exception:
            pass

    def request_fsm_mode(self) -> bool:
        if not self.cli_get_state.service_is_ready():
            return False
        try:
            req = self._srv_GetMode.Request()
            future = self.cli_get_state.call_async(req)
            self._track_future(future, self._handle_get_fsm_mode_response)
            return True
        except Exception as exc:
            self.get_logger().warn(f"GetMode request failed: {exc}")
            return False

    def request_possible_transitions(self) -> bool:
        if not self.cli_get_possible_transitions.service_is_ready():
            return False
        try:
            req = self._srv_GetPossibleTransitions.Request()
            future = self.cli_get_possible_transitions.call_async(req)
            self._track_future(future, self._handle_get_possible_transitions_response)
            return True
        except Exception as exc:
            self.get_logger().warn(f"GetPossibleTransitions request failed: {exc}")
            return False

    def _handle_get_fsm_mode_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"GetMode response failed: {exc}")
            return
        mode = getattr(response, 'mode', None)
        if mode is None:
            return
        self._emit_fsm_mode_feedback(mode)

    def _handle_get_possible_transitions_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"GetPossibleTransitions response failed: {exc}")
            return
        transitions = getattr(response, 'possible_transitions', None)
        self._emit_possible_transitions_feedback(transitions)

    def _handle_change_fsm_response(self, future, transition: int):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"ChangeMode response failed for transition {transition}: {exc}")
            return

        success = bool(getattr(response, 'success', False))
        self.get_logger().info(f"ChangeMode response -> transition={transition} success={success}")

        if not success:
            self.get_logger().warn(f"FSM transition rejected by backend: {transition}")
            return

        # Fuerza un refresco inmediato del estado real y de las transiciones válidas.
        self.request_fsm_mode()
        self.request_possible_transitions()

    def _watch_change_fsm_future(self, future, transition: int, service_name: str):
        if future.done():
            return
        self.get_logger().warn(
            f"ChangeMode future still pending after {self._fsm_request_watchdog_timeout_s:.1f}s "
            f"for transition {transition} on {service_name}"
        )

    def send_state(self, key: str):
        # Intentar traducir a transición FSM real (GUI_pkg usa servicio ChangeMode)
        # Aquí asumimos que key llega como string numérica o 'int-like'.
        try:
            transition = int(key)
        except Exception:
            transition = None

        if transition is not None and self.cli_change_fsm.service_is_ready():
            service_name = getattr(self.cli_change_fsm, 'srv_name', '<unknown>')
            self.get_logger().info(f"FSM transition -> {transition} via {service_name}")
            try:
                req = self._srv_ChangeMode.Request()
                req.transition = transition
                future = self.cli_change_fsm.call_async(req)
                threading.Timer(
                    self._fsm_request_watchdog_timeout_s,
                    self._watch_change_fsm_future,
                    args=(future, transition, service_name),
                ).start()
                self._track_future(
                    future,
                    lambda done_future, selected_transition=transition:
                        self._handle_change_fsm_response(done_future, selected_transition),
                )
            except Exception as exc:
                self.get_logger().warn(f"ChangeMode call failed for transition {transition}: {exc}")
        else:
            self.get_logger().warn(f"FSM ChangeMode not ready/invalid key, ignored: {key}")

    def send_guided(self, lat: float, lon: float):
        # Legacy GUI_pkg doesn't have a guided-target API.
        self.get_logger().warn(
            "Guided target requested but J8 has no known guided-target API in the legacy contract"
        )

    def set_path_control_mode(self, mode: str):
        normalized = 'external' if str(mode or '').strip().lower() == 'external' else 'internal'
        if normalized == self._path_control_mode:
            return
        self._path_control_mode = normalized
        self.get_logger().info(f"Path control mode -> {normalized}")

    def is_external_path_control_enabled(self) -> bool:
        return self._path_control_mode == 'external'

    def _build_wgs84_path(self, pts):
        path = Path()
        path.header.frame_id = 'wgs84'
        path.header.stamp = self.get_clock().now().to_msg()
        for (lat, lon) in (pts or []):
            ps = PoseStamped()
            ps.header.frame_id = 'wgs84'
            ps.pose.position.x = float(lon)
            ps.pose.position.y = float(lat)
            path.poses.append(ps)
        return path

    def _handle_robot_path_response(self, future, label: str, waypoint_count: int):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"{label} response failed: {exc}")
            return

        ack = getattr(response, 'ack', None)
        if ack is False:
            self.get_logger().warn(f"{label} rejected {waypoint_count} waypoints")
            return

        self.get_logger().info(f"{label} accepted {waypoint_count} waypoints")

    def _send_path_via_service(self, client, pts, label: str):
        if not client.service_is_ready():
            self.get_logger().warn(f"{label} service not ready, ignored {len(pts)} waypoints")
            return False

        try:
            req = self._srv_RobotPath.Request()
            req.path = self._build_wgs84_path(pts)
            future = client.call_async(req)
            future.add_done_callback(
                lambda done, service_label=label, waypoint_count=len(pts):
                    self._handle_robot_path_response(done, service_label, waypoint_count)
            )
            self.get_logger().info(f"{label} request queued -> {len(pts)} waypoints")
            return True
        except Exception as exc:
            self.get_logger().warn(f"{label} call failed: {exc}")
            return False

    def _publish_external_path_command(self, command: str):
        cmd = str(command or '').strip().lower()
        if not cmd:
            return False
        try:
            msg = self._msg_String()
            msg.data = cmd
            self.pub_external_path_command.publish(msg)
            self.get_logger().info(f"External path command -> {cmd}")
            return True
        except Exception as exc:
            self.get_logger().warn(f"External path command publish failed: {exc}")
            return False

    def send_path(self, pts):
        if self.is_external_path_control_enabled():
            self._send_path_via_service(self.cli_send_external_path, pts, 'ExternalPath')
            return
        self._send_path_via_service(self.cli_send_draw_path, pts, 'Path')

    def save_current_path(self, filename: str):
        name = str(filename or '').strip()
        client = self.cli_write_external_path if self.is_external_path_control_enabled() else self.cli_write_path
        label = 'WriteExternalPathToFile' if self.is_external_path_control_enabled() else 'WritePathToFile'
        if not name or not client.service_is_ready():
            self.get_logger().warn(f"{label} service not ready/invalid filename: {name}")
            return

        req = self._srv_WritePathToFile.Request()
        req.filename = name
        client.call_async(req)
        self.get_logger().info(f"Save path ({self._path_control_mode}) -> {name}")

    def load_saved_path(self, filename: str):
        name = str(filename or '').strip()
        client = self.cli_read_external_path if self.is_external_path_control_enabled() else self.cli_read_path
        label = 'ReadExternalPathFromFile' if self.is_external_path_control_enabled() else 'ReadPathFromFile'
        if not name or not client.service_is_ready():
            self.get_logger().warn(f"{label} service not ready/invalid filename: {name}")
            return

        req = self._srv_ReadPathFromFile.Request()
        req.filename = name
        client.call_async(req)
        self.get_logger().info(f"Load path ({self._path_control_mode}) -> {name}")

    def start_path_following(self):
        if self.is_external_path_control_enabled():
            self._publish_external_path_command('start')
            return
        self.send_state(0)

    def stop_path_following(self):
        if self.is_external_path_control_enabled():
            self._publish_external_path_command('stop')
            return
        self.send_state(1)

    def clear_active_path(self):
        if self.is_external_path_control_enabled():
            self.send_path([])
            self._publish_external_path_command('clear')
            return
        self.send_path([])

    def send_cfg(self, cfg: dict):
        """Apply controller selection + parameters.

        Expected dict keys (from ControlWidget):
          - controller_type: str
          - v_forward: float
          - l_ahead_dist: float
          - (optional) k_error_lineal, k_error_angular
          - (optional) look_ahead_dis, r_min
        """

        if not isinstance(cfg, dict):
            self.get_logger().warn(f"send_cfg ignored (not a dict): {type(cfg)}")
            return

        controller_type = str(cfg.get('controller_type', '')).strip()
        if not controller_type:
            self.get_logger().warn('send_cfg ignored: missing controller_type')
            return

        # 1) Change controller type (if service available)
        if self.cli_change_controller.service_is_ready():
            try:
                req = self._srv_ChangeController.Request()
                req.controller_type = controller_type
                self.cli_change_controller.call_async(req)
            except Exception as e:
                self.get_logger().warn(f"ChangeController call failed: {e}")
        else:
            self.get_logger().warn('ChangeController service not ready')

        # 2) Configure parameters
        try:
            v_forward = float(cfg.get('v_forward'))
            l_ahead_dist = float(cfg.get('l_ahead_dist'))
        except Exception:
            self.get_logger().warn('send_cfg ignored: invalid v_forward/l_ahead_dist')
            return

        if controller_type in {'pure_pursuit', 'follow_the_carrot'}:
            if self.cli_config_pp.service_is_ready():
                try:
                    req = self._srv_ConfigPurePursuitCtrl.Request()
                    req.v_forward = v_forward
                    req.l_ahead_dist = l_ahead_dist
                    self.cli_config_pp.call_async(req)
                    self.get_logger().info(f"ConfigPurePursuitCtrl -> v={v_forward} la={l_ahead_dist}")
                    return
                except Exception as e:
                    self.get_logger().warn(f"ConfigPurePursuitCtrl call failed: {e}")
            else:
                self.get_logger().warn('ConfigPurePursuitCtrl service not ready')
            return

        if controller_type == 'dynamic_pure_pursuit':
            try:
                max_v_forward = float(cfg.get('lin_max', v_forward))
                max_ang_acc = float(cfg.get('acc_ang'))
                max_ang_dec = float(cfg.get('acc_ang'))
                max_lin_acc = float(cfg.get('acc_lin'))
                max_lin_dec = float(cfg.get('acc_lin'))
            except Exception:
                self.get_logger().warn('send_cfg ignored: invalid dynamic pure params')
                return

            if self.cli_config_dynamic.service_is_ready():
                try:
                    req = self._srv_ConfigDynamicPureCtrl.Request()
                    req.look_ahead_dis = l_ahead_dist
                    req.max_v_forward = max_v_forward
                    req.max_ang_acc = max_ang_acc
                    req.max_ang_dec = max_ang_dec
                    req.max_lin_acc = max_lin_acc
                    req.max_lin_dec = max_lin_dec
                    self.cli_config_dynamic.call_async(req)
                    self.get_logger().info(
                        f"ConfigDynamicPureCtrl -> la={l_ahead_dist} v_max={max_v_forward} a_ang={max_ang_acc} a_lin={max_lin_acc}"
                    )
                except Exception as e:
                    self.get_logger().warn(f"ConfigDynamicPureCtrl call failed: {e}")
            else:
                self.get_logger().warn('ConfigDynamicPureCtrl service not ready')
            return

        if controller_type == 'dynamic_la_pure_pursuit':
            try:
                max_v_forward = float(cfg.get('lin_max', v_forward))
                max_ang_acc = float(cfg.get('acc_ang'))
                max_ang_dec = float(cfg.get('acc_ang'))
                max_lin_acc = float(cfg.get('acc_lin'))
                max_lin_dec = float(cfg.get('acc_lin'))
            except Exception:
                self.get_logger().warn('send_cfg ignored: invalid dynamic LA params')
                return

            if self.cli_config_dynamic_la.service_is_ready():
                try:
                    req = self._srv_ConfigDynamicLAPureCtrl.Request()
                    req.look_ahead_v_gain = 1.0
                    req.max_v_forward = max_v_forward
                    req.max_ang_acc = max_ang_acc
                    req.max_ang_dec = max_ang_dec
                    req.max_lin_acc = max_lin_acc
                    req.max_lin_dec = max_lin_dec
                    req.speed_pow = 1.0
                    req.min_look_ahead_d = l_ahead_dist
                    self.cli_config_dynamic_la.call_async(req)
                    self.get_logger().info(
                        f"ConfigDynamicLAPureCtrl -> la_min={l_ahead_dist} v_max={max_v_forward} a_ang={max_ang_acc} a_lin={max_lin_acc}"
                    )
                except Exception as e:
                    self.get_logger().warn(f"ConfigDynamicLAPureCtrl call failed: {e}")
            else:
                self.get_logger().warn('ConfigDynamicLAPureCtrl service not ready')
            return

        if controller_type == 'regulated_pure_pursuit':
            try:
                look_ahead_dis = float(cfg.get('look_ahead_dis', l_ahead_dist))
                r_min = float(cfg.get('r_min'))
            except Exception:
                self.get_logger().warn('send_cfg ignored: invalid regulated params (r_min/look_ahead_dis)')
                return

            if self.cli_config_regulated.service_is_ready():
                try:
                    req = self._srv_ConfigRegulatedPureCtrl.Request()
                    req.look_ahead_dis = look_ahead_dis
                    req.v_forward = v_forward
                    req.r_min = r_min
                    self.cli_config_regulated.call_async(req)
                    self.get_logger().info(
                        f"ConfigRegulatedPureCtrl -> la={look_ahead_dis} v={v_forward} r_min={r_min}"
                    )
                except Exception as e:
                    self.get_logger().warn(f"ConfigRegulatedPureCtrl call failed: {e}")
            else:
                self.get_logger().warn('ConfigRegulatedPureCtrl service not ready')
            return

        if controller_type == 'stanley':
            try:
                k_lin = float(cfg.get('k_error_lineal'))
                k_ang = float(cfg.get('k_error_angular'))
            except Exception:
                self.get_logger().warn('send_cfg ignored: invalid stanley params (k_error_lineal/k_error_angular)')
                return

            if self.cli_config_stanley.service_is_ready():
                try:
                    req = self._srv_ConfigStanleyCtrl.Request()
                    req.v_forward = v_forward
                    req.l_ahead_dist = l_ahead_dist
                    req.k_error_lineal = k_lin
                    req.k_error_angular = k_ang
                    self.cli_config_stanley.call_async(req)
                    self.get_logger().info(
                        f"ConfigStanleyCtrl -> v={v_forward} la={l_ahead_dist} k_lin={k_lin} k_ang={k_ang}"
                    )
                except Exception as e:
                    self.get_logger().warn(f"ConfigStanleyCtrl call failed: {e}")
            else:
                self.get_logger().warn('ConfigStanleyCtrl service not ready')
            return

        self.get_logger().warn(
            f"Control config for controller_type='{controller_type}' not mapped yet"
        )


# ====== MainWindow ======
class MainWindow(QMainWindow):
    _requestRemotePersonsFlush = Signal()
    _requestRobotMarkersFlush = Signal()
    _requestPersonsViewRefresh = Signal()

    def __init__(self, defer_ros_start=True):
        super().__init__()
        self._planned = []
        self._person_records = []
        self._next_person_id = 1
        self._person_match_threshold_m = 8.0
        self._person_record_stale_timeout_s = 300.0
        self._fanet_geo_smoothing_alpha = 0.25
        self._person_pending_distance_m = 20.0
        self._person_rescued_distance_m = 5.0
        self._auto_person_tracking_enabled = False
        self._persons_fanet_centroids = []
        self._persons_fanet_positions = []
        self._persons_fanet_distances = []
        self._persons_fanet_frame_count = 0
        self._persons_fanet_subscriptions = []
        self._persons_fanet_pending_detections = None
        self._persons_fanet_lock = threading.Lock()
        self._ros = None
        self._robot_pose_node = None
        self._defer_ros = defer_ros_start
        self._tile_proxy = None
        self._exec = None
        self._th = None
        self._current_namespace = self._default_namespace()
        self._path_control_mode = 'internal'
        self._known_robot_namespaces = self._initial_known_robot_namespaces(self._current_namespace)
        self._robot_roles = {self._current_namespace: 'explorador'}
        self._robot_relay_urls = self._load_robot_relay_urls()
        self._robot_relay_urls.setdefault(self._current_namespace, self._default_relay_base_url())
        self._robot_marker_color_by_namespace = {}
        self._robot_pose_subscriptions = {}
        self._robot_pose_states = {}
        self._robot_pose_lock = threading.Lock()
        self._robot_pose_warn_timeout_s = 5.0
        self._robot_pose_drop_timeout_s = 120.0
        self._remote_person_subscriptions = {}
        self._remote_person_streams = {}
        self._remote_persons_lock = threading.Lock()
        self._remote_persons_flush_timer = QTimer(self)
        self._remote_persons_flush_timer.setSingleShot(True)
        self._remote_persons_flush_timer.timeout.connect(self._flush_remote_person_streams)
        self._requestRemotePersonsFlush.connect(self._schedule_remote_person_flush_main_thread)
        self._pending_js_by_key = {}
        self._pending_js_generic = []
        self._map_ready = False
        self._js_flush_timer = QTimer(self)
        self._js_flush_timer.setSingleShot(True)
        self._js_flush_timer.timeout.connect(self._flush_js_queue)
        self._robot_markers_timer = QTimer(self)
        self._robot_markers_timer.setSingleShot(True)
        self._robot_markers_timer.timeout.connect(self._flush_robot_markers)
        self._requestRobotMarkersFlush.connect(self._refresh_robot_markers_main_thread)
        self._namespace_refresh_timer = QTimer(self)
        self._namespace_refresh_timer.setInterval(1500)
        self._namespace_refresh_timer.timeout.connect(self._refresh_robot_namespace_choices)
        self._persons_refresh_timer = QTimer(self)
        self._persons_refresh_timer.setSingleShot(True)
        self._persons_refresh_timer.timeout.connect(self._apply_refresh_person_records_view)
        self._requestPersonsViewRefresh.connect(self._refresh_person_records_view_main_thread)
        self._persons_fanet_timer = QTimer(self)
        self._persons_fanet_timer.setInterval(100)
        self._persons_fanet_timer.timeout.connect(self._flush_persons_fanet_detections)
        self._persons_fanet_timer.start()
        self._person_stale_timer = QTimer(self)
        self._person_stale_timer.setInterval(500)
        self._person_stale_timer.timeout.connect(self._on_person_records_watchdog)
        self._person_stale_timer.start()

        self.tabs = QTabWidget()
        self._build_tab_mission()
        self._build_tab_control()
        self._build_tab_persons()
        self._build_tab_video()
        self._build_tab_fsm_debug()
        self.tabs.currentChanged.connect(self._on_tab_changed)

        right = QWidget()
        rv = QVBoxLayout(right); rv.setContentsMargins(0,0,0,0)

        robot_row = QHBoxLayout()
        self.cmb_robot = QComboBox()
        self.cmb_robot.setEditable(True)
        self._set_robot_combo_items(self._known_robot_namespaces)
        self.cmb_role = QComboBox()
        self.cmb_role.setEditable(True)
        self.cmb_role.addItems(ROBOT_ROLE_LABELS)
        self.cmb_role.setCurrentText(self._robot_roles[self._current_namespace])
        self.cmb_robot.setCurrentText(self._current_namespace)
        self.ed_relay_url = QLineEdit(self._relay_url_for_namespace(self._current_namespace))
        self.ed_relay_url.setPlaceholderText('http://10.142.47.160:8080')
        self.btn_apply_robot = QPushButton('Aplicar robot')
        self.btn_apply_robot.clicked.connect(self._on_apply_robot_namespace)
        self.cmb_robot.editTextChanged.connect(self._on_robot_namespace_text_changed)
        robot_row.addWidget(QLabel('Robot:'))
        robot_row.addWidget(self.cmb_robot, 1)
        robot_row.addWidget(QLabel('Rol:'))
        robot_row.addWidget(self.cmb_role, 1)
        self.lbl_path_control_mode = QLabel()
        robot_row.addWidget(QLabel('Control path:'))
        robot_row.addWidget(self.lbl_path_control_mode)
        robot_row.addWidget(QLabel('Relay video:'))
        robot_row.addWidget(self.ed_relay_url, 2)
        robot_row.addWidget(self.btn_apply_robot)
        rv.addLayout(robot_row)

        # Map controls
        self.chk_follow_robot = QCheckBox('Seguir robot')
        self.chk_follow_robot.setChecked(True)
        rv.addWidget(self.chk_follow_robot)

        self.web = QWebEngineView()
        rv.addWidget(self.web)

        sp = QSplitter(Qt.Horizontal)
        sp.addWidget(self.tabs)
        sp.addWidget(right)
        sp.setSizes([420, 1000])
        sp.setStretchFactor(0, 0)
        sp.setStretchFactor(1, 1)
        self.setCentralWidget(sp)
        self.resize(1400, 900)
        self._update_window_title()

        self._init_map()

    def _build_tab_mission(self):
        # Tab principal (estado + misión) vive en state_widget.py
        self.mission_tab = MissionWidget()
        self.tabs.addTab(self.mission_tab, 'Misión')

    # def _build_tab_control(self):
    #     w = QWidget(); lay = QVBoxLayout(w)
    #     frm = QFormLayout()
    #     self.ed_vlin = QLineEdit('1.0')
    #     self.ed_vang = QLineEdit('0.6')
    #     self.ed_alin = QLineEdit('0.5')
    #     self.ed_aang = QLineEdit('0.8')
    #     frm.addRow('Vel. lineal máx [m/s]:', self.ed_vlin)
    #     frm.addRow('Vel. angular máx [rad/s]:', self.ed_vang)
    #     frm.addRow('Acel. lineal [m/s²]:', self.ed_alin)
    #     frm.addRow('Acel. angular [rad/s²]:', self.ed_aang)
    #     lay.addLayout(frm)

    #     self.cmb_follow = QComboBox(); self.cmb_follow.addItems(['follow1', 'follow2'])
    #     lay.addWidget(QLabel('Modo Follow:')); lay.addWidget(self.cmb_follow)

    #     b_apply = QPushButton('Aplicar configuración'); b_apply.clicked.connect(self._on_apply_cfg)
    #     lay.addWidget(b_apply)
    #     lay.addStretch(1)
    #     self.tabs.addTab(w, 'Control')
    def _build_tab_control(self):
        self.control_tab = ControlWidget(self._ros)
        self.control_tab.pathControlModeChanged.connect(self._on_path_control_mode_changed)
        self.tabs.addTab(self.control_tab, 'Control')

    def _build_tab_persons(self):
        self.persons_tab = PersonsWidget()
        self.persons_tab.assignmentRequested.connect(self._on_person_assignment_change)
        self.persons_tab.set_current_robot(self._current_namespace)
        self.persons_tab.set_current_role(self._robot_roles.get(self._current_namespace, 'explorador'))
        self.tabs.addTab(self.persons_tab, 'Personas')

    def _build_tab_video(self):
        self.video_tab = VideoTabWidget(
            relay_base_url=self._relay_url_for_namespace(self._current_namespace)
        )
        self.tabs.addTab(self.video_tab, 'Video')

    def _build_tab_fsm_debug(self):
        self.fsm_debug_tab = FsmDebugWidget(namespace=self._current_namespace)
        self.tabs.addTab(self.fsm_debug_tab, 'FSM Debug')

    def _default_relay_base_url(self) -> str:
        return os.environ.get('IMAGE_RELAY_BASE_URL', 'http://127.0.0.1:8080').strip() or 'http://127.0.0.1:8080'

    def _load_robot_relay_urls(self):
        mapping = {}
        raw_value = os.environ.get('IMAGE_RELAY_BASE_URLS', '').strip()
        if raw_value:
            for item in raw_value.replace(';', ',').split(','):
                key, sep, value = item.partition('=')
                if not sep:
                    continue
                namespace = self._normalize_detected_namespace(key)
                relay_url = str(value or '').strip()
                if namespace and relay_url:
                    mapping[namespace] = relay_url

        default_url = self._default_relay_base_url()
        raw_namespace = os.environ.get('ROBOT_NAMESPACE', '').strip()
        namespace = self._normalize_detected_namespace(raw_namespace)
        if namespace and default_url:
            mapping.setdefault(namespace, default_url)

        return mapping

    def _relay_url_for_namespace(self, namespace: str) -> str:
        ns = self._normalized_namespace(namespace)
        relay_url = str(self._robot_relay_urls.get(ns, '') or '').strip()
        if relay_url:
            return relay_url
        return self._default_relay_base_url()

    def _default_namespace(self) -> str:
        value = self._normalize_detected_namespace(os.environ.get('ROBOT_NAMESPACE', ''))
        return value or DEFAULT_PRIMARY_NAMESPACE

    def _initial_known_robot_namespaces(self, current_namespace: str):
        namespaces = []
        for namespace in (current_namespace, *DEFAULT_KNOWN_ROBOT_NAMESPACES):
            ns = self._normalize_detected_namespace(namespace)
            if ns and ns not in namespaces:
                namespaces.append(ns)
        return namespaces

    def _normalized_namespace(self, namespace: str) -> str:
        value = str(namespace or '').strip().strip('/')
        return value or self._default_namespace()

    def _normalize_detected_namespace(self, namespace: str):
        value = str(namespace or '').strip().strip('/')
        return value or None

    def _extract_namespace_from_graph_name(self, graph_name: str, suffix: str):
        name = str(graph_name or '').strip()
        if not name.startswith('/') or not name.endswith(suffix):
            return None

        namespace = name[:-len(suffix)]
        return self._normalize_detected_namespace(namespace)

    def _discover_active_robot_namespaces(self):
        graph_node = self._robot_pose_node or self._ros
        if graph_node is None:
            return []

        namespaces = set()

        try:
            topic_names = graph_node.get_topic_names_and_types()
        except Exception:
            topic_names = []

        for topic_name, _topic_types in topic_names:
            for suffix in ROBOT_DISCOVERY_TOPIC_SUFFIXES:
                namespace = self._extract_namespace_from_graph_name(topic_name, suffix)
                if namespace:
                    namespaces.add(namespace)

        try:
            service_names = graph_node.get_service_names_and_types()
        except Exception:
            service_names = []

        for service_name, _service_types in service_names:
            for suffix in ROBOT_DISCOVERY_SERVICE_SUFFIXES:
                namespace = self._extract_namespace_from_graph_name(service_name, suffix)
                if namespace:
                    namespaces.add(namespace)

        return sorted(namespaces)

    def _discover_robot_pose_topics(self):
        graph_node = self._robot_pose_node or self._ros
        if graph_node is None:
            return {}

        try:
            topic_names = graph_node.get_topic_names_and_types()
        except Exception:
            topic_names = []
        topic_name_set = {name for name, _types in topic_names}

        topics_by_namespace = {}

        for topic_name, _topic_types in topic_names:
            namespace = None
            key = None
            if topic_name.endswith(ROBOT_POSE_FIX_TOPIC_SUFFIX):
                namespace = self._extract_namespace_from_graph_name(topic_name, ROBOT_POSE_FIX_TOPIC_SUFFIX)
                key = 'fix'
            elif topic_name.endswith(ROBOT_POSE_ODOM_TOPIC_SUFFIX):
                namespace = self._extract_namespace_from_graph_name(topic_name, ROBOT_POSE_ODOM_TOPIC_SUFFIX)
                key = 'odom'

            if not namespace or not key:
                continue
            topics_by_namespace.setdefault(namespace, {})[key] = topic_name

        global_fix = ROBOT_POSE_FIX_TOPIC_SUFFIX
        global_odom = ROBOT_POSE_ODOM_TOPIC_SUFFIX
        if global_fix in topic_name_set or global_odom in topic_name_set:
            for namespace in GLOBAL_FIXPOSITION_FALLBACK_NAMESPACES:
                ns = self._normalized_namespace(namespace)
                namespaced_fix = f'/{ns}{ROBOT_POSE_FIX_TOPIC_SUFFIX}'
                namespaced_odom = f'/{ns}{ROBOT_POSE_ODOM_TOPIC_SUFFIX}'
                if namespaced_fix in topic_name_set or namespaced_odom in topic_name_set:
                    continue
                entry = topics_by_namespace.setdefault(ns, {})
                if global_fix in topic_name_set:
                    entry.setdefault('fix', global_fix)
                if global_odom in topic_name_set:
                    entry.setdefault('odom', global_odom)

        return topics_by_namespace

    def _resolve_fixposition_topics(self, namespace: str):
        graph_node = self._robot_pose_node or self._ros
        ns = self._normalized_namespace(namespace)
        topics = {
            'odom': ROBOT_POSE_ODOM_TOPIC_SUFFIX,
            'fix': ROBOT_POSE_FIX_TOPIC_SUFFIX,
            'gps_vel': ROBOT_POSE_GPS_VEL_TOPIC_SUFFIX,
        }
        if graph_node is None:
            if ns in GLOBAL_FIXPOSITION_FALLBACK_NAMESPACES:
                return topics
            return {
                'odom': f'/{ns}{ROBOT_POSE_ODOM_TOPIC_SUFFIX}',
                'fix': f'/{ns}{ROBOT_POSE_FIX_TOPIC_SUFFIX}',
                'gps_vel': f'/{ns}{ROBOT_POSE_GPS_VEL_TOPIC_SUFFIX}',
            }

        try:
            topic_names = graph_node.get_topic_names_and_types()
        except Exception:
            topic_names = []
        topic_name_set = {name for name, _types in topic_names}

        namespaced_topics = {
            'odom': f'/{ns}{ROBOT_POSE_ODOM_TOPIC_SUFFIX}',
            'fix': f'/{ns}{ROBOT_POSE_FIX_TOPIC_SUFFIX}',
            'gps_vel': f'/{ns}{ROBOT_POSE_GPS_VEL_TOPIC_SUFFIX}',
        }
        if namespaced_topics['fix'] in topic_name_set or namespaced_topics['odom'] in topic_name_set:
            return {
                'odom': namespaced_topics['odom'],
                'fix': namespaced_topics['fix'],
                'gps_vel': namespaced_topics['gps_vel'],
            }

        if ns in GLOBAL_FIXPOSITION_FALLBACK_NAMESPACES:
            return topics

        return {
            'odom': namespaced_topics['odom'],
            'fix': namespaced_topics['fix'],
            'gps_vel': namespaced_topics['gps_vel'],
        }

    def _discover_remote_person_topics(self):
        graph_node = self._robot_pose_node or self._ros
        if graph_node is None:
            return {}

        topics_by_namespace = {}
        try:
            topic_names = graph_node.get_topic_names_and_types()
        except Exception:
            topic_names = []

        for topic_name, _topic_types in topic_names:
            namespace = None
            key = None
            if topic_name.endswith(REMOTE_PERSONS_LATLON_TOPIC_SUFFIX):
                namespace = self._extract_namespace_from_graph_name(topic_name, REMOTE_PERSONS_LATLON_TOPIC_SUFFIX)
                key = 'latlon'
            elif topic_name.endswith(REMOTE_SARNET_POSITIONS_TOPIC_SUFFIX):
                namespace = self._extract_namespace_from_graph_name(topic_name, REMOTE_SARNET_POSITIONS_TOPIC_SUFFIX)
                key = 'positions'
            elif topic_name.endswith(REMOTE_SARNET_DISTANCES_TOPIC_SUFFIX):
                namespace = self._extract_namespace_from_graph_name(topic_name, REMOTE_SARNET_DISTANCES_TOPIC_SUFFIX)
                key = 'distances'

            if not namespace or not key:
                continue
            topics_by_namespace.setdefault(namespace, {})[key] = topic_name

        return topics_by_namespace

    def _marker_color_for_namespace(self, namespace: str) -> str:
        ns = self._normalized_namespace(namespace)
        color = self._robot_marker_color_by_namespace.get(ns)
        if color:
            return color

        used = set(self._robot_marker_color_by_namespace.values())
        for candidate in ROBOT_MARKER_COLORS:
            if candidate not in used:
                color = candidate
                break
        if not color:
            color = ROBOT_MARKER_COLORS[len(self._robot_marker_color_by_namespace) % len(ROBOT_MARKER_COLORS)]
        self._robot_marker_color_by_namespace[ns] = color
        return color

    def _destroy_robot_pose_subscription(self, subscription):
        if subscription is None:
            return
        target_node = self._robot_pose_node or self._ros
        if target_node is None:
            return
        try:
            target_node.destroy_subscription(subscription)
        except Exception:
            pass

    def _remove_robot_pose_subscription(self, namespace: str, drop_state: bool = True):
        subscriptions = self._robot_pose_subscriptions.pop(namespace, None)
        if subscriptions:
            self._destroy_robot_pose_subscription(subscriptions.get('fix'))
            self._destroy_robot_pose_subscription(subscriptions.get('odom'))
        if drop_state:
            with self._robot_pose_lock:
                self._robot_pose_states.pop(namespace, None)

    def _set_robot_pose_state(self, namespace: str, lat=None, lon=None, heading_deg=None, no_gps: bool = False):
        ns = self._normalized_namespace(namespace)
        self._marker_color_for_namespace(ns)
        with self._robot_pose_lock:
            state = dict(self._robot_pose_states.get(ns, {}))
            if lat is not None:
                state['lat'] = float(lat)
            if lon is not None:
                state['lon'] = float(lon)
            if heading_deg is not None:
                state['heading_deg'] = float(heading_deg) % 360.0
            state['no_gps'] = bool(no_gps)
            state['last_seen_monotonic'] = time.monotonic()
            self._robot_pose_states[ns] = state
        self._refresh_robot_markers()

    def _namespace_georef(self, namespace: str):
        ns = self._normalized_namespace(namespace)
        with self._robot_pose_lock:
            state = dict(self._robot_pose_states.get(ns, {}))

        lat = state.get('lat')
        lon = state.get('lon')
        heading_deg = state.get('heading_deg')
        if lat is None or lon is None or heading_deg is None:
            return None
        try:
            return float(lat), float(lon), float(heading_deg)
        except Exception:
            return None

    def _destroy_remote_person_subscription(self, subscription):
        if subscription is None:
            return
        target_node = self._robot_pose_node or self._ros
        if target_node is None:
            return
        try:
            target_node.destroy_subscription(subscription)
        except Exception:
            pass

    def _remove_remote_person_subscription(self, namespace: str, drop_state: bool = True):
        subscriptions = self._remote_person_subscriptions.pop(namespace, None)
        if subscriptions:
            self._destroy_remote_person_subscription(subscriptions.get('latlon'))
            self._destroy_remote_person_subscription(subscriptions.get('positions'))
            self._destroy_remote_person_subscription(subscriptions.get('distances'))
        if drop_state:
            with self._remote_persons_lock:
                self._remote_person_streams.pop(namespace, None)

    def _schedule_remote_person_flush(self):
        self._requestRemotePersonsFlush.emit()

    def _schedule_remote_person_flush_main_thread(self):
        if not self._remote_persons_flush_timer.isActive():
            self._remote_persons_flush_timer.start(80)

    def _on_remote_persons_latlon_msg(self, namespace: str, msg: PoseArray):
        with self._remote_persons_lock:
            stream = self._remote_person_streams.setdefault(namespace, {})
            stream['latlon'] = list(msg.poses)
        self._schedule_remote_person_flush()

    def _on_remote_persons_positions_msg(self, namespace: str, msg: PoseArray):
        with self._remote_persons_lock:
            stream = self._remote_person_streams.setdefault(namespace, {})
            stream['positions'] = list(msg.poses)
        self._schedule_remote_person_flush()

    def _on_remote_persons_distances_msg(self, namespace: str, msg: Float32MultiArray):
        with self._remote_persons_lock:
            stream = self._remote_person_streams.setdefault(namespace, {})
            stream['distances'] = [float(value) for value in msg.data]
        self._schedule_remote_person_flush()

    def _sync_remote_person_subscriptions(self):
        target_node = self._robot_pose_node or self._ros
        if target_node is None:
            return

        topics_by_namespace = self._discover_remote_person_topics()
        wanted_namespaces = set(topics_by_namespace.keys())
        existing_namespaces = set(self._remote_person_subscriptions.keys())

        for namespace in sorted(existing_namespaces - wanted_namespaces):
            self._remove_remote_person_subscription(namespace, drop_state=False)

        for namespace, topics in topics_by_namespace.items():
            previous = self._remote_person_subscriptions.get(namespace, {})
            previous_latlon = previous.get('latlon_topic')
            previous_positions = previous.get('positions_topic')
            previous_distances = previous.get('distances_topic')
            current_latlon = topics.get('latlon')
            current_positions = topics.get('positions')
            current_distances = topics.get('distances')
            if (
                previous
                and previous_latlon == current_latlon
                and previous_positions == current_positions
                and previous_distances == current_distances
            ):
                continue

            self._remove_remote_person_subscription(namespace, drop_state=False)

            subscriptions = {
                'latlon': None,
                'positions': None,
                'distances': None,
                'latlon_topic': current_latlon,
                'positions_topic': current_positions,
                'distances_topic': current_distances,
            }
            if current_latlon:
                subscriptions['latlon'] = target_node.create_subscription(
                    PoseArray,
                    current_latlon,
                    lambda msg, ns=namespace: self._on_remote_persons_latlon_msg(ns, msg),
                    10,
                )
            if current_positions:
                subscriptions['positions'] = target_node.create_subscription(
                    PoseArray,
                    current_positions,
                    lambda msg, ns=namespace: self._on_remote_persons_positions_msg(ns, msg),
                    10,
                )
            if current_distances:
                subscriptions['distances'] = target_node.create_subscription(
                    Float32MultiArray,
                    current_distances,
                    lambda msg, ns=namespace: self._on_remote_persons_distances_msg(ns, msg),
                    10,
                )
            self._remote_person_subscriptions[namespace] = subscriptions

    def _build_remote_person_detections(self, namespace: str, stream: dict):
        latlon_poses = list(stream.get('latlon', []) or [])
        local_poses = list(stream.get('positions', []) or [])
        distances = list(stream.get('distances', []) or [])
        total = max(len(latlon_poses), len(local_poses), len(distances))
        georef = self._namespace_georef(namespace)
        detections = []

        for index in range(total):
            latlon_pose = latlon_poses[index] if index < len(latlon_poses) else None
            local_pose = local_poses[index] if index < len(local_poses) else None
            distance = distances[index] if index < len(distances) else math.nan

            external_id = index + 1
            lat = None
            lon = None
            local_x = None
            local_y = None
            local_z = None

            if latlon_pose is not None:
                try:
                    lat = float(latlon_pose.position.x)
                    lon = float(latlon_pose.position.y)
                except Exception:
                    lat = lon = None
                try:
                    pose_id = int(round(float(latlon_pose.orientation.w)))
                    if pose_id > 0:
                        external_id = pose_id
                except Exception:
                    pass

            if local_pose is not None:
                try:
                    local_x = float(local_pose.position.x)
                    local_y = float(local_pose.position.y)
                    local_z = float(local_pose.position.z)
                except Exception:
                    local_x = local_y = local_z = None

            if (lat is None or lon is None) and georef is not None and local_x is not None and local_y is not None:
                robot_lat, robot_lon, heading_deg = georef
                heading_rad = math.radians(heading_deg)
                east_m = float(local_x) * math.sin(heading_rad) - float(local_y) * math.cos(heading_rad)
                north_m = float(local_x) * math.cos(heading_rad) + float(local_y) * math.sin(heading_rad)
                lat, lon = _offset_latlon(robot_lat, robot_lon, east_m, north_m)

            detections.append({
                'source_namespace': namespace,
                'external_id': int(external_id),
                'lat': lat,
                'lon': lon,
                'local_x': local_x,
                'local_y': local_y,
                'local_z': local_z,
                'distance_m': None if not math.isfinite(distance) else float(distance),
            })

        return detections

    def _merge_remote_person_detections(self, namespace: str, detections):
        seen_at = time.monotonic()
        namespace = self._normalized_namespace(namespace)
        origin_label = f'sarnet:{namespace}'
        previous_records_by_key = {
            (str(record.get('source_namespace', '')), int(record.get('source_external_id', 0) or 0)): dict(record)
            for record in self._person_records
            if record.get('origin') == 'sarnet'
        }
        new_records = [
            dict(record)
            for record in self._person_records
            if not (record.get('origin') == 'sarnet' and str(record.get('source_namespace', '')) == namespace)
        ]

        for index, detection in enumerate(detections, start=1):
            if not isinstance(detection, dict):
                continue
            external_id = int(detection.get('external_id', index) or index)
            previous_record = previous_records_by_key.get((namespace, external_id), {})
            record = {
                'id': int(previous_record.get('id', self._next_person_id)),
                'status': 'detected',
                'origin': 'sarnet',
                'origin_label': origin_label,
                'source_namespace': namespace,
                'source_external_id': external_id,
                'assigned_robot': str(previous_record.get('assigned_robot', '') or ''),
                'distance_m': detection.get('distance_m', None),
                'local_x': detection.get('local_x', None),
                'local_y': detection.get('local_y', None),
                'local_z': detection.get('local_z', None),
                'lat': detection.get('lat', None),
                'lon': detection.get('lon', None),
                'last_seen_monotonic': seen_at,
            }
            if 'id' not in previous_record:
                self._next_person_id += 1
            new_records.append(record)

        self._person_records = sorted(new_records, key=lambda record: (int(record['id']), str(record.get('origin', ''))))

    def _flush_remote_person_streams(self):
        with self._remote_persons_lock:
            streams = {key: dict(value) for key, value in self._remote_person_streams.items()}

        changed = False
        for namespace, stream in streams.items():
            detections = self._build_remote_person_detections(namespace, stream)
            if not detections:
                continue
            self._merge_remote_person_detections(namespace, detections)
            changed = True
        if changed:
            self._refresh_person_records_view()

    def _on_robot_pose_fix(self, namespace: str, msg: NavSatFix):
        try:
            lat = float(msg.latitude)
            lon = float(msg.longitude)
        except Exception:
            return
        no_gps = not ((msg.status.status is not None) and (msg.status.status >= 0))
        self._set_robot_pose_state(namespace, lat=lat, lon=lon, no_gps=no_gps)

    def _on_robot_pose_odom(self, namespace: str, msg: Odometry):
        try:
            q = msg.pose.pose.orientation
            heading_deg = _quat_to_bearing_deg(q.x, q.y, q.z, q.w)
        except Exception:
            return
        self._set_robot_pose_state(namespace, heading_deg=heading_deg)

    def _sync_robot_pose_subscriptions(self):
        target_node = self._robot_pose_node or self._ros
        if target_node is None:
            return

        topics_by_namespace = self._discover_robot_pose_topics()
        wanted_namespaces = set(topics_by_namespace.keys())
        existing_namespaces = set(self._robot_pose_subscriptions.keys())

        for namespace in sorted(existing_namespaces - wanted_namespaces):
            # Conservamos la última pose conocida aunque haya un bache en el grafo ROS.
            self._remove_robot_pose_subscription(namespace, drop_state=False)

        for namespace, topics in topics_by_namespace.items():
            previous = self._robot_pose_subscriptions.get(namespace, {})
            previous_fix = previous.get('fix_topic')
            previous_odom = previous.get('odom_topic')
            current_fix = topics.get('fix')
            current_odom = topics.get('odom')
            if previous and previous_fix == current_fix and previous_odom == current_odom:
                continue

            self._remove_robot_pose_subscription(namespace, drop_state=False)

            subscriptions = {
                'fix': None,
                'odom': None,
                'fix_topic': current_fix,
                'odom_topic': current_odom,
            }
            if current_fix:
                subscriptions['fix'] = target_node.create_subscription(
                    NavSatFix,
                    current_fix,
                    lambda msg, ns=namespace: self._on_robot_pose_fix(ns, msg),
                    10,
                )
            if current_odom:
                subscriptions['odom'] = target_node.create_subscription(
                    Odometry,
                    current_odom,
                    lambda msg, ns=namespace: self._on_robot_pose_odom(ns, msg),
                    10,
                )
            self._robot_pose_subscriptions[namespace] = subscriptions
            self._marker_color_for_namespace(namespace)

    def _build_robot_marker_payload(self):
        now = time.monotonic()
        with self._robot_pose_lock:
            states = {key: dict(value) for key, value in self._robot_pose_states.items()}

        payload = []
        for namespace, state in states.items():
            lat = state.get('lat')
            lon = state.get('lon')
            if lat is None or lon is None:
                continue
            last_seen = state.get('last_seen_monotonic')
            stale_age_s = None
            if last_seen is not None:
                stale_age_s = now - float(last_seen)
            if stale_age_s is not None and stale_age_s > self._robot_pose_drop_timeout_s:
                continue

            role = self._robot_roles.get(namespace, '')
            label = namespace if not role else f'{namespace} [{role}]'
            badge = ''
            if state.get('no_gps'):
                badge = 'NO GPS'
            elif stale_age_s is not None and stale_age_s > self._robot_pose_warn_timeout_s:
                badge = 'SIN DATOS'
            payload.append({
                'name': namespace,
                'label': label,
                'lat': float(lat),
                'lon': float(lon),
                'heading_deg': state.get('heading_deg', None),
                'color': self._marker_color_for_namespace(namespace),
                'is_active': namespace == self._current_namespace,
                'badge': badge,
            })

        payload.sort(key=lambda item: (0 if item.get('is_active') else 1, item.get('name', '')))
        return payload

    def _refresh_robot_markers(self):
        self._requestRobotMarkersFlush.emit()

    def _refresh_robot_markers_main_thread(self):
        if not self._robot_markers_timer.isActive():
            self._robot_markers_timer.start(80)

    def _js_set_robot_markers(self, robots):
        safe_robots = []
        for robot in robots or []:
            try:
                safe_robots.append({
                    'name': str(robot.get('name', '')),
                    'label': str(robot.get('label', robot.get('name', ''))),
                    'lat': float(robot['lat']),
                    'lon': float(robot['lon']),
                    'heading_deg': None if robot.get('heading_deg') is None else float(robot['heading_deg']),
                    'color': str(robot.get('color', '#e53e3e')),
                    'is_active': bool(robot.get('is_active', False)),
                    'badge': str(robot.get('badge', '')),
                })
            except Exception:
                continue
        self._js_call(f'window.setRobotMarkers({json.dumps(safe_robots)});')

    def _flush_robot_markers(self):
        self._js_set_robot_markers(self._build_robot_marker_payload())

    def _set_robot_combo_items(self, namespaces, preserve_text: str = None):
        items = []
        seen = set()
        for namespace in namespaces:
            ns = self._normalize_detected_namespace(namespace)
            if not ns or ns in seen:
                continue
            seen.add(ns)
            items.append(ns)

        current_text = preserve_text
        if current_text is None and hasattr(self, 'cmb_robot'):
            current_text = self.cmb_robot.currentText()
        current_text = self._normalize_detected_namespace(current_text) or self._current_namespace

        if current_text not in seen:
            items.insert(0, current_text)
            seen.add(current_text)

        self._known_robot_namespaces = items

        if not hasattr(self, 'cmb_robot'):
            return

        existing_items = [self.cmb_robot.itemText(i) for i in range(self.cmb_robot.count())]
        if existing_items == items and self.cmb_robot.currentText() == current_text:
            return

        self.cmb_robot.blockSignals(True)
        self.cmb_robot.clear()
        self.cmb_robot.addItems(items)
        self.cmb_robot.setCurrentText(current_text)
        self.cmb_robot.blockSignals(False)
        if hasattr(self, 'ed_relay_url'):
            self.ed_relay_url.setText(self._relay_url_for_namespace(current_text))

    def _remember_robot_namespace(self, namespace: str):
        ns = self._normalized_namespace(namespace)
        merged = list(self._known_robot_namespaces)
        if ns not in merged:
            merged.append(ns)
        self._set_robot_combo_items(merged, preserve_text=ns)
        return ns

    def _refresh_robot_namespace_choices(self):
        detected = self._discover_active_robot_namespaces()
        preferred = [self._current_namespace]
        if hasattr(self, 'cmb_robot'):
            preferred.append(self.cmb_robot.currentText())

        merged = []
        for namespace in preferred + list(self._known_robot_namespaces) + detected + list(DEFAULT_KNOWN_ROBOT_NAMESPACES):
            ns = self._normalize_detected_namespace(namespace)
            if ns and ns not in merged:
                merged.append(ns)

        self._set_robot_combo_items(merged)

        detected_set = {self._normalize_detected_namespace(namespace) for namespace in detected}
        detected_set.discard(None)
        if detected_set and self._current_namespace not in detected_set and len(detected_set) == 1:
            target_namespace = next(iter(detected_set))
            self._robot_roles.setdefault(target_namespace, self._robot_roles.get(self._current_namespace, 'explorador'))
            self._robot_relay_urls.setdefault(target_namespace, self._relay_url_for_namespace(self._current_namespace))
            if self._exec is None:
                self._current_namespace = target_namespace
                self.persons_tab.set_current_robot(target_namespace)
                self.persons_tab.set_current_role(self._robot_roles[target_namespace])
                self.cmb_robot.setCurrentText(target_namespace)
                self.cmb_role.setCurrentText(self._robot_roles[target_namespace])
                self.ed_relay_url.setText(self._relay_url_for_namespace(target_namespace))
                self._update_window_title()
            else:
                self._connect_robot_namespace(target_namespace)
                return

        self._sync_robot_pose_subscriptions()
        self._sync_remote_person_subscriptions()

    def _update_window_title(self):
        role = self._robot_roles.get(self._current_namespace, 'sin rol')
        mode_label = 'externo' if self._path_control_mode == 'external' else 'interno'
        if hasattr(self, 'lbl_path_control_mode'):
            self.lbl_path_control_mode.setText(mode_label)
        self.setWindowTitle(f'GUI — {self._current_namespace} [{role}] [{mode_label}] — Map + Control + FollowZED')

    def _on_path_control_mode_changed(self, mode: str):
        self._path_control_mode = 'external' if str(mode or '').strip().lower() == 'external' else 'internal'
        self._update_window_title()

    def _build_mission_topics(self, namespace: str):
        from gui.ros_api import namespaced

        ns = self._normalized_namespace(namespace)
        fixposition_topics = self._resolve_fixposition_topics(ns)
        return {
            'odom': fixposition_topics['odom'],
            'fix': fixposition_topics['fix'],
            'gps_vel': fixposition_topics['gps_vel'],
            'fsm_mode': namespaced(ns, 'fsm_mode'),
            'possible_transitions': namespaced(ns, 'possible_transitions'),
            'cte': namespaced(ns, 'cte'),
            'look_ahead_distance': namespaced(ns, 'look_ahead_distance'),
            'min_distance_to_path': namespaced(ns, 'min_distance_to_path'),
            'detected_persons_latlon': namespaced(ns, 'detected_persons_latlon'),
            'detected_persons_global_latlon': '/detected_persons_latlon_global',
        }

    def _connect_robot_namespace(self, namespace: str):
        ns = self._remember_robot_namespace(namespace)
        old_ros = self._ros

        if self._exec is not None:
            try:
                self.mission_tab.detach_ros(self._exec)
            except Exception:
                pass

        self._ros = RosSide(namespace=ns)
        self._exec.add_node(self._ros)

        self._current_namespace = ns
        self._person_records = []
        self._next_person_id = 1
        self._persons_fanet_centroids = []
        self._persons_fanet_positions = []
        self._persons_fanet_distances = []
        self._persons_fanet_frame_count = 0
        self._persons_fanet_subscriptions = []
        self._persons_fanet_pending_detections = None
        self.cmb_robot.setCurrentText(ns)
        self.cmb_role.setCurrentText(self._robot_roles.get(ns, 'explorador'))
        self.ed_relay_url.setText(self._relay_url_for_namespace(ns))
        self._update_window_title()

        self.mission_tab.set_ros(self._ros)
        self.mission_tab.attach_ros(self._exec, topics=self._build_mission_topics(ns))
        self._attach_persons_fanet_subscriptions(ns)
        self.control_tab.set_ros(self._ros)
        self.persons_tab.set_current_robot(ns)
        self.persons_tab.set_current_role(self._robot_roles.get(ns, 'explorador'))
        self.persons_tab.set_records([])
        if hasattr(self, 'fsm_debug_tab'):
            self.fsm_debug_tab.set_namespace(ns)
        self._js_set_detected_persons([])
        self._flush_remote_person_streams()
        if hasattr(self, 'video_tab'):
            self.video_tab.set_relay_url(self._relay_url_for_namespace(ns))

        try:
            self._ros.get_logger().info(f"Connected GUI ROS namespace -> {ns}")
            self._ros.request_fsm_mode()
            self._ros.request_possible_transitions()
        except Exception:
            pass

        if old_ros is not None:
            try:
                self._exec.remove_node(old_ros)
            except Exception:
                pass
            try:
                old_ros.destroy_node()
            except Exception:
                pass

        self._refresh_robot_namespace_choices()

    def _on_apply_robot_namespace(self):
        namespace = self._remember_robot_namespace(self.cmb_robot.currentText())
        role = self.cmb_role.currentText().strip() or 'explorador'
        relay_url = self.ed_relay_url.text().strip() or self._default_relay_base_url()
        self._robot_roles[namespace] = role
        self._robot_relay_urls[namespace] = relay_url
        if self._exec is None:
            self._current_namespace = namespace
            self.persons_tab.set_current_robot(namespace)
            self.persons_tab.set_current_role(role)
            self.ed_relay_url.setText(relay_url)
            self._update_window_title()
            return
        try:
            self._connect_robot_namespace(namespace)
            self.persons_tab.set_current_role(role)
        except Exception as e:
            print(f"[GUI] no se pudo conectar al namespace {namespace}: {e}")

    def _on_robot_namespace_text_changed(self, namespace: str):
        ns = self._normalize_detected_namespace(namespace)
        if not ns:
            return
        relay_url = self._robot_relay_urls.get(ns)
        if relay_url:
            self.ed_relay_url.setText(str(relay_url))

    def _on_tab_changed(self, index: int):
        # Al volver a "Personas" → forzar refresco inmediato de la tabla
        if hasattr(self, 'persons_tab') and self.tabs.widget(index) is self.persons_tab:
            self._apply_refresh_person_records_view()

    def _init_map(self):
        # settings
        self.web.settings().setAttribute(QWebEngineSettings.LocalContentCanAccessFileUrls, True)
        self.web.settings().setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls, True)

        # load logs
        self.web.loadStarted.connect(lambda: print("[WEB] loadStarted"))
        self.web.loadProgress.connect(lambda p: print(f"[WEB] loadProgress {p}%"))
        self.web.loadFinished.connect(lambda ok: print(f"[WEB] loadFinished ok={ok}"))

        # profile
        profile: QWebEngineProfile = self.web.page().profile()
        profile.setHttpUserAgent(
            "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 "
            "(KHTML, like Gecko) Chrome/122.0 Safari/537.36"
        )
        profile.setHttpAcceptLanguage("es-ES,es;q=0.9,en;q=0.8")
        self._interceptor = HeaderInjector()
        profile.setUrlRequestInterceptor(self._interceptor)

        # WebChannel
        self.bridge = MapBridge(self)
        self.bridge.removeWaypointIndex.connect(self.mission_tab.on_remove_index_from_map)
        # self.bridge.guidedTarget.connect(self._guided_from_map)
        # self.bridge.plannedWaypoint.connect(self._planned_from_map)
        self.bridge.guidedTarget.connect(self.mission_tab.on_guided_from_map)
        self.bridge.plannedWaypoint.connect(self.mission_tab.on_planned_from_map)
        channel = QWebChannel(self.web.page())
        channel.registerObject("bridge", self.bridge)
        self.web.page().setWebChannel(channel)
        self.mission_tab.set_js_call(self._js_call)

        # proxy + estático
        html_path = _resolve_html_path()
        static_root = os.path.dirname(html_path)
        self._tile_proxy = TileProxy(root_dir=static_root)
        port = self._tile_proxy.start()  # sirve /ui/* y /osm /esri
        print(f"[GUI] estático desde: {static_root}")

        # cargar por HTTP, no file://
        url = QUrl(f"http://127.0.0.1:{port}/ui/map.html")
        print(f"[GUI] cargando: {url.toString()}")
        self.web.setUrl(url)

        # Vehicle marker bootstrap: show something immediately so we can tell the JS bridge works
        # even when GPS hasn't arrived yet.
        self._last_vehicle = (0.0, 0.0, None)
        self._have_vehicle_gps = False
        self.web.loadFinished.connect(self._on_map_load_finished)

        # Feed Fixposition pose (already consumed by the State tab) into the web map.
        # We hook the signal here because main_window owns the web view and JS calls.
        try:
            self.mission_tab.state._signals.gps_pose.connect(self._on_gps_pose)
            self.mission_tab.state._signals.detected_persons.connect(self._on_detected_persons)
        except Exception:
            # If the widget structure changes, we fail gracefully (map just won't show vehicle).
            pass

    def _on_map_load_finished(self, ok: bool):
        self._map_ready = bool(ok)
        if not ok:
            return
        self._refresh_robot_markers()
        self._flush_js_queue()

    def _on_gps_pose(self, lat: float, lon: float, heading_deg):
        """Update the vehicle marker on the embedded Leaflet map.

        Expects `map.html` to expose `window.updateVehicle(lat, lon[, headingDeg])`.
        """

        try:
            lat_f = float(lat)
            lon_f = float(lon)
        except Exception:
            return

        heading_value = heading_deg
        try:
            if heading_value is not None and not math.isfinite(float(heading_value)):
                heading_value = None
        except Exception:
            heading_value = None

        # Record last known pose.
        self._have_vehicle_gps = True
        self._last_vehicle = (lat_f, lon_f, heading_value)
        self._set_robot_pose_state(self._current_namespace, lat=lat_f, lon=lon_f, heading_deg=heading_value, no_gps=False)

        pan = bool(self.chk_follow_robot.isChecked())
        self._js_update_vehicle(lat_f, lon_f, heading_value, pan=pan, no_gps=False)
        self._georef_fanet_records()
        self._auto_update_person_statuses(lat_f, lon_f)

    def _georef_fanet_records(self):
        changed = False
        for record in self._person_records:
            if record.get('origin') != 'fanet':
                continue

            georef = self._current_robot_georef()
            if georef is None:
                continue

            robot_lat, robot_lon, heading_deg = georef
            record_lat = record.get('lat')
            record_lon = record.get('lon')

            if record_lat is not None and record_lon is not None:
                forward_m, left_m = _latlon_to_local_offsets(robot_lat, robot_lon, record_lat, record_lon, heading_deg)
                previous_local_x = record.get('local_x')
                previous_local_y = record.get('local_y')
                if previous_local_x is None or abs(float(previous_local_x) - float(forward_m)) >= 0.05:
                    record['local_x'] = float(forward_m)
                    changed = True
                if previous_local_y is None or abs(float(previous_local_y) - float(left_m)) >= 0.05:
                    record['local_y'] = float(left_m)
                    changed = True
                continue

            if record.get('local_x') is None or record.get('local_y') is None:
                continue

            lat, lon = self._fanet_detection_to_latlon({
                'robot_x': record.get('local_x'),
                'robot_y': record.get('local_y'),
            })
            if lat is None or lon is None:
                continue
            if record.get('lat') != lat or record.get('lon') != lon:
                record['lat'] = float(lat)
                record['lon'] = float(lon)
                changed = True
        if changed:
            self._refresh_person_records_view()

    def _js_update_vehicle(self, lat: float, lon: float, heading_deg, pan: bool = False, no_gps: bool = False):
        """Update current robot pose on the map and optionally pan to it."""

        try:
            lat_f = float(lat)
            lon_f = float(lon)
        except Exception:
            return

        heading_value = None
        if heading_deg is not None:
            try:
                heading_value = float(heading_deg) % 360.0
            except Exception:
                heading_value = None

        self._set_robot_pose_state(self._current_namespace, lat=lat_f, lon=lon_f, heading_deg=heading_value, no_gps=no_gps)

        if pan:
            self._js_call(f'window.panTo({lat_f}, {lon_f});')

    def _on_detected_persons(self, points):
        if isinstance(points, str):
            try:
                points = json.loads(points)
            except Exception:
                return
        if not isinstance(points, list):
            return
        self._merge_detected_persons(points)
        self._refresh_person_records_view()

    def _on_fanet_detections(self, detections):
        if not isinstance(detections, list):
            return
        self._merge_fanet_detections(detections)
        self._refresh_person_records_view()

    def _attach_persons_fanet_subscriptions(self, namespace: str):
        if self._ros is None:
            return

        prefix = f'/{self._normalized_namespace(namespace)}/fanet'
        self._persons_fanet_subscriptions = [
            self._ros.create_subscription(PoseArray, f'{prefix}/person_centroids', self._on_persons_fanet_centroids_msg, 10),
            self._ros.create_subscription(PoseArray, f'{prefix}/person_positions_robot', self._on_persons_fanet_positions_msg, 10),
            self._ros.create_subscription(Float32MultiArray, f'{prefix}/person_distances', self._on_persons_fanet_distances_msg, 10),
        ]

    def _set_persons_fanet_frame_count(self, count: int):
        self._persons_fanet_frame_count = max(0, int(count))
        if len(self._persons_fanet_centroids) > self._persons_fanet_frame_count:
            self._persons_fanet_centroids = self._persons_fanet_centroids[:self._persons_fanet_frame_count]
        if len(self._persons_fanet_positions) > self._persons_fanet_frame_count:
            self._persons_fanet_positions = self._persons_fanet_positions[:self._persons_fanet_frame_count]
        if len(self._persons_fanet_distances) > self._persons_fanet_frame_count:
            self._persons_fanet_distances = self._persons_fanet_distances[:self._persons_fanet_frame_count]

    def _build_persons_fanet_detections(self):
        detections = []
        for index in range(self._persons_fanet_frame_count):
            centroid = self._persons_fanet_centroids[index] if index < len(self._persons_fanet_centroids) else None
            position = self._persons_fanet_positions[index] if index < len(self._persons_fanet_positions) else None
            distance = self._persons_fanet_distances[index] if index < len(self._persons_fanet_distances) else math.nan

            detection = {
                'id': index + 1,
                'pixel_x': None,
                'pixel_y': None,
                'area_px': None,
                'robot_x': None,
                'robot_y': None,
                'robot_z': None,
                'distance_m': None,
            }

            if centroid is not None:
                detection['pixel_x'] = float(centroid.position.x)
                detection['pixel_y'] = float(centroid.position.y)
                detection['area_px'] = float(centroid.position.z)

            if position is not None:
                detection['robot_x'] = float(position.position.x)
                detection['robot_y'] = float(position.position.y)
                detection['robot_z'] = float(position.position.z)

            if math.isfinite(distance):
                detection['distance_m'] = float(distance)

            detections.append(detection)

        return detections

    def _publish_persons_fanet_detections(self):
        detections = self._build_persons_fanet_detections()
        with self._persons_fanet_lock:
            self._persons_fanet_pending_detections = detections

    def _flush_persons_fanet_detections(self):
        with self._persons_fanet_lock:
            detections = self._persons_fanet_pending_detections
            self._persons_fanet_pending_detections = None
        if detections is None:
            return
        self._on_fanet_detections(detections)

    def _on_persons_fanet_centroids_msg(self, msg: PoseArray):
        self._persons_fanet_centroids = list(msg.poses)
        self._set_persons_fanet_frame_count(len(self._persons_fanet_centroids))
        self._publish_persons_fanet_detections()

    def _on_persons_fanet_positions_msg(self, msg: PoseArray):
        self._persons_fanet_positions = list(msg.poses)
        self._set_persons_fanet_frame_count(len(self._persons_fanet_positions))
        self._publish_persons_fanet_detections()

    def _on_persons_fanet_distances_msg(self, msg: Float32MultiArray):
        self._persons_fanet_distances = [float(value) for value in msg.data]
        self._set_persons_fanet_frame_count(len(self._persons_fanet_distances))
        self._publish_persons_fanet_detections()

    def _current_robot_georef(self):
        pose = getattr(self, '_last_vehicle', None)
        if not pose or len(pose) != 3:
            return None
        lat, lon, heading_deg = pose
        if not self._have_vehicle_gps:
            return None
        if heading_deg is None:
            return None
        try:
            return float(lat), float(lon), float(heading_deg)
        except Exception:
            return None

    def _fanet_detection_to_latlon(self, detection):
        georef = self._current_robot_georef()
        robot_x = detection.get('robot_x', None)
        robot_y = detection.get('robot_y', None)
        if georef is None or robot_x is None or robot_y is None:
            return None, None

        lat, lon, heading_deg = georef
        heading_rad = math.radians(heading_deg)
        forward_m = float(robot_x)
        left_m = float(robot_y)
        east_m = forward_m * math.sin(heading_rad) - left_m * math.cos(heading_rad)
        north_m = forward_m * math.cos(heading_rad) + left_m * math.sin(heading_rad)
        return _offset_latlon(lat, lon, east_m, north_m)

    def _smooth_fanet_latlon(self, record_id: int, lat, lon, previous_records_by_id: dict):
        if lat is None or lon is None:
            return None, None

        previous = previous_records_by_id.get(int(record_id))
        if previous is None:
            return float(lat), float(lon)

        previous_lat = previous.get('lat')
        previous_lon = previous.get('lon')
        if previous_lat is None or previous_lon is None:
            return float(lat), float(lon)

        alpha = float(self._fanet_geo_smoothing_alpha)
        smoothed_lat = (1.0 - alpha) * float(previous_lat) + alpha * float(lat)
        smoothed_lon = (1.0 - alpha) * float(previous_lon) + alpha * float(lon)
        return smoothed_lat, smoothed_lon

    def _merge_fanet_detections(self, detections):
        seen_at = time.monotonic()
        previous_records_by_id = {
            int(record['id']): dict(record)
            for record in self._person_records
            if record.get('origin') == 'fanet'
        }
        new_records = [
            dict(record)
            for record in self._person_records
            if record.get('origin') != 'fanet'
        ]

        for index, detection in enumerate(detections, start=1):
            if not isinstance(detection, dict):
                continue

            record_id = int(detection.get('id', index) or index)
            lat, lon = self._fanet_detection_to_latlon(detection)
            lat, lon = self._smooth_fanet_latlon(record_id, lat, lon, previous_records_by_id)
            previous_record = previous_records_by_id.get(record_id, {})
            local_x = detection.get('robot_x', None)
            local_y = detection.get('robot_y', None)
            record = {
                'id': record_id,
                'status': 'detected',
                'origin': 'fanet',
                'assigned_robot': str(previous_record.get('assigned_robot', '') or ''),
                'distance_m': None,
                'local_x': None,
                'local_y': None,
                'local_z': None,
                'lat': None,
                'lon': None,
            }

            if local_x is not None:
                record['local_x'] = float(local_x)
            if local_y is not None:
                record['local_y'] = float(local_y)
            if detection.get('robot_z') is not None:
                record['local_z'] = float(detection['robot_z'])
            if detection.get('distance_m') is not None:
                record['distance_m'] = float(detection['distance_m'])
            if lat is not None and lon is not None:
                record['lat'] = float(lat)
                record['lon'] = float(lon)
            record['last_seen_monotonic'] = seen_at

            new_records.append(record)

        self._person_records = sorted(new_records, key=lambda record: (int(record['id']), str(record.get('origin', ''))))
        self._next_person_id = max([1] + [int(record['id']) + 1 for record in self._person_records])

    def _merge_detected_persons(self, points):
        seen_at = time.monotonic()
        unmatched_records = {record['id'] for record in self._person_records}

        for point in points:
            person_id = 0
            if isinstance(point, dict):
                try:
                    lat = float(point['lat'])
                    lon = float(point['lon'])
                    person_id = int(point.get('id', 0) or 0)
                except Exception:
                    continue
            else:
                try:
                    lat = float(point[0])
                    lon = float(point[1])
                except Exception:
                    continue

            if person_id > 0:
                existing = next((record for record in self._person_records if int(record['id']) == person_id), None)
                if existing is not None:
                    existing['lat'] = lat
                    existing['lon'] = lon
                    existing.setdefault('assigned_robot', '')
                    existing['status'] = 'detected'
                    existing['last_seen_monotonic'] = seen_at
                    unmatched_records.discard(existing['id'])
                    continue

            best_record = None
            best_distance = None
            for record in self._person_records:
                if record['id'] not in unmatched_records:
                    continue
                distance = _haversine_m(lat, lon, record['lat'], record['lon'])
                if distance > self._person_match_threshold_m:
                    continue
                if best_distance is None or distance < best_distance:
                    best_distance = distance
                    best_record = record

            if best_record is not None:
                best_record['lat'] = lat
                best_record['lon'] = lon
                best_record.setdefault('assigned_robot', '')
                best_record['status'] = 'detected'
                best_record['last_seen_monotonic'] = seen_at
                unmatched_records.discard(best_record['id'])
                continue

            self._person_records.append({
                'id': person_id if person_id > 0 else self._next_person_id,
                'lat': lat,
                'lon': lon,
                'status': 'detected',
                'assigned_robot': '',
                'distance_m': None,
                'last_seen_monotonic': seen_at,
            })
            if person_id > 0:
                self._next_person_id = max(self._next_person_id, person_id + 1)
            else:
                self._next_person_id += 1

        self._person_records.sort(key=lambda record: int(record['id']))

    def _prune_stale_person_records(self):
        now = time.monotonic()
        kept_records = []
        changed = False
        for record in self._person_records:
            last_seen = record.get('last_seen_monotonic', None)
            if last_seen is None:
                kept_records.append(record)
                continue
            # Si la persona ya está georreferenciada, la mantenemos en el mapa
            # aunque deje de detectarse un rato. Su posición se actualizará en
            # cuanto reaparezca una detección compatible.
            if record.get('lat') is not None and record.get('lon') is not None:
                kept_records.append(record)
                continue
            if (now - float(last_seen)) <= self._person_record_stale_timeout_s:
                kept_records.append(record)
                continue
            changed = True

        if changed:
            self._person_records = kept_records
            self._next_person_id = max([1] + [int(record['id']) + 1 for record in self._person_records])

        return changed

    def _on_person_records_watchdog(self):
        if self._prune_stale_person_records():
            self._refresh_person_records_view()

    def _refresh_person_records_view(self):
        self._requestPersonsViewRefresh.emit()

    def _refresh_person_records_view_main_thread(self):
        if not self._persons_refresh_timer.isActive():
            self._persons_refresh_timer.start(120)

    def _apply_refresh_person_records_view(self):
        self._prune_stale_person_records()
        self.persons_tab.set_records(self._person_records)
        points = []
        for record in self._person_records:
            if record.get('lat') is None or record.get('lon') is None:
                continue
            points.append({
                'id': int(record['id']),
                'lat': float(record['lat']),
                'lon': float(record['lon']),
                'status': 'detected',
                'assigned_robot': str(record.get('assigned_robot', '') or ''),
                'distance_m': record.get('distance_m', None),
            })
        self._js_set_detected_persons(points)

    def _auto_update_person_statuses(self, robot_lat: float, robot_lon: float):
        changed = False
        for record in self._person_records:
            try:
                distance = _haversine_m(robot_lat, robot_lon, record['lat'], record['lon'])
            except Exception:
                continue

            previous_distance = record.get('distance_m', None)
            record['distance_m'] = float(distance)
            record['status'] = 'detected'
            if previous_distance is None or abs(float(previous_distance) - float(distance)) >= 1.0:
                changed = True

        if changed:
            self._refresh_person_records_view()

    def _on_person_status_change(self, person_id: int, status: str):
        return

    def _on_person_assignment_change(self, person_id: int, robot_name: str):
        robot_value = str(robot_name or '').strip()
        for record in self._person_records:
            if int(record['id']) == int(person_id):
                record['assigned_robot'] = robot_value
                break
        self._refresh_person_records_view()

    def _on_auto_person_tracking_changed(self, enabled: bool):
        return

    def _on_person_thresholds_changed(self, pending_m: float, rescued_m: float):
        return

    def _js_set_detected_persons(self, points):
        safe_points = []
        for point in points:
            if isinstance(point, dict):
                try:
                    safe_points.append({
                        'id': int(point.get('id', 0)),
                        'lat': float(point['lat']),
                        'lon': float(point['lon']),
                        'status': str(point.get('status', 'detected')),
                        'assigned_robot': str(point.get('assigned_robot', '') or ''),
                        'distance_m': point.get('distance_m', None),
                    })
                except Exception:
                    continue
                continue

            try:
                lat = float(point[0])
                lon = float(point[1])
            except Exception:
                continue
            safe_points.append({'id': 0, 'lat': lat, 'lon': lon, 'status': 'detected'})
        self._js_call(f'window.setDetectedPersons({json.dumps(safe_points)});')

    @staticmethod
    def _classify_js_call(js: str):
        if js.startswith('window.setMission('):
            return 'mission'
        if js.startswith('window.setDetectedPersons('):
            return 'persons'
        if js.startswith('window.setRobotMarkers('):
            return 'robots'
        if js.startswith('window.updateVehicle('):
            return 'vehicle'
        if js.startswith('window.panTo('):
            return 'pan'
        return None

    def _flush_js_queue(self):
        page = self.web.page() if hasattr(self, 'web') else None
        if page is None:
            self._pending_js_by_key.clear()
            self._pending_js_generic.clear()
            return

        if not self._map_ready:
            return

        ordered_keys = ['mission', 'persons', 'robots', 'vehicle', 'pan']
        for key in ordered_keys:
            js = self._pending_js_by_key.pop(key, None)
            if js:
                page.runJavaScript(js)

        generic_calls = self._pending_js_generic[:8]
        self._pending_js_generic.clear()
        for js in generic_calls:
            page.runJavaScript(js)

    def closeEvent(self, ev):
        for namespace in list(self._robot_pose_subscriptions.keys()):
            self._remove_robot_pose_subscription(namespace, drop_state=False)
        for namespace in list(self._remote_person_subscriptions.keys()):
            self._remove_remote_person_subscription(namespace, drop_state=False)
        if self._robot_pose_node is not None and self._exec is not None:
            try:
                self._exec.remove_node(self._robot_pose_node)
            except Exception:
                pass
            try:
                self._robot_pose_node.destroy_node()
            except Exception:
                pass
            self._robot_pose_node = None
        if hasattr(self, 'video_tab'):
            try:
                self.video_tab.shutdown()
            except Exception:
                pass
        super().closeEvent(ev)

    # ===== ROS =====
    def start_ros(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            if self._exec is None:
                self._exec = rclpy.executors.MultiThreadedExecutor(num_threads=2)
                self._th = threading.Thread(target=self._spin_executor, daemon=True)
                self._th.start()
            if self._robot_pose_node is None:
                self._robot_pose_node = Node('cuadriga_gui_robot_pose_monitor')
                self._exec.add_node(self._robot_pose_node)
            self._connect_robot_namespace(self._current_namespace)
            self._sync_robot_pose_subscriptions()
            self._sync_remote_person_subscriptions()
            if not self._namespace_refresh_timer.isActive():
                self._namespace_refresh_timer.start()
        except Exception as e:
            print(f"[GUI] no se pudo iniciar ROS: {e}")

    def _spin_executor(self):
        while rclpy.ok():
            try:
                self._exec.spin()
                return
            except RuntimeError as e:
                print(f"[GUI] executor runtime error: {e}")
                time.sleep(0.1)
            except Exception as e:
                print(f"[GUI] executor error: {e}")
                time.sleep(0.25)

    # ===== GUI slots =====
    def _on_change_state(self):
        if not self._ros:
            # Aunque no haya ROS, actualiza el HUD para ver el estado en pantalla
            label = self.cmb_state.currentText()
            self.hud.set_state(label)
            return

        key = self.cmb_state.currentData()
        label = self.cmb_state.currentText()
        self._ros.send_state(key)    # publicas a /gui/state_cmd
        self.hud.set_state(label)    # lo ves en el HUD


    def _on_add_wp(self):
        try:
            lat = float(self.ed_lat.text()); lon = float(self.ed_lon.text())
        except Exception:
            return
        self._planned.append((lat, lon))
        self._refresh_table()
        self._js_set_mission()

    def _on_pan(self):
        try:
            lat = float(self.ed_lat.text()); lon = float(self.ed_lon.text())
        except Exception:
            return
        self._js_call(f'window.panTo({lat}, {lon});')

    def _on_send_path(self):
        if not self._ros: return
        self._ros.send_path(self._planned)

    def _on_clear(self):
        self._planned.clear()
        self._refresh_table()
        self._js_call('window.clearMission();')

    def _on_load_csv(self):
        fn, _ = QFileDialog.getOpenFileName(self, 'CSV de waypoints', filter='CSV (*.csv)')
        if not fn: return
        pts = []
        try:
            with open(fn, 'r') as f:
                for line in f:
                    parts = [p.strip() for p in line.split(',')]
                    if len(parts) < 2: continue
                    lat = float(parts[0]); lon = float(parts[1])
                    pts.append((lat, lon))
        except Exception:
            return
        self._planned = pts
        self._refresh_table()
        self._js_set_mission()

    def _on_apply_cfg(self):
        if not self._ros: return
        try:
            cfg = dict(
                lin_max=float(self.ed_vlin.text()),
                ang_max=float(self.ed_vang.text()),
                acc_lin=float(self.ed_alin.text()),
                acc_ang=float(self.ed_aang.text()),
                follow_mode=self.cmb_follow.currentText(),
            )
        except Exception:
            return
        self._ros.send_cfg(cfg)

    # ===== Bridge callbacks =====
    def _guided_from_map(self, lat, lon):
        if self._ros:
            self._ros.send_guided(lat, lon)
        self._js_call(f'window.panTo({lat}, {lon});')

    def _planned_from_map(self, lat, lon):
        self._planned.append((lat, lon))
        self._refresh_table()
        self._js_set_mission()

    # ===== util =====
    def _refresh_table(self):
        self.tbl.setRowCount(len(self._planned))
        for i, (lat, lon) in enumerate(self._planned):
            self.tbl.setItem(i, 0, QTableWidgetItem(str(i)))
            self.tbl.setItem(i, 1, QTableWidgetItem(f"{lat:.7f}"))
            self.tbl.setItem(i, 2, QTableWidgetItem(f"{lon:.7f}"))

    def _js_call(self, js: str):
        page = self.web.page() if hasattr(self, 'web') else None
        if page is None:
            return

        key = self._classify_js_call(js)
        if key is None:
            self._pending_js_generic.append(js)
            if len(self._pending_js_generic) > 8:
                self._pending_js_generic = self._pending_js_generic[-8:]
        else:
            self._pending_js_by_key[key] = js

        if self._map_ready and not self._js_flush_timer.isActive():
            self._js_flush_timer.start(33)

    def _js_set_mission(self):
        js_array = json.dumps(self._planned)
        self._js_call(f'window.setMission({js_array});')
