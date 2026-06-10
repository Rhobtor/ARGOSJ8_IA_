import sys
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
from gui.app.main_window import MainWindow
import os

DEFAULT_CYCLONEDDS_URI = 'file:///etc/cyclonedds/local_cyclonedds.xml'

# 1) limpia proxies heredados que están mal formados
for k in ['http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY',
          'all_proxy','ALL_PROXY','no_proxy','NO_PROXY']:
    os.environ.pop(k, None)

# 2) fuerza a Chromium (Qt WebEngine) a NO usar proxy
os.environ['QTWEBENGINE_CHROMIUM_FLAGS'] = (
    os.environ.get('QTWEBENGINE_CHROMIUM_FLAGS','') + ' --no-proxy-server'
)


def _sanitize_dds_environment():
    requested_rmw = os.environ.get('RMW_IMPLEMENTATION', '').strip()
    requested_uri = os.environ.get('CYCLONEDDS_URI', '').strip()
    default_path = DEFAULT_CYCLONEDDS_URI.removeprefix('file://')
    has_default_config = os.path.isfile(default_path)

    # Local desktop runs should not force CycloneDDS unless the user or the
    # container entrypoint already configured it. In this workspace, forcing
    # rmw_cyclonedds_cpp makes topic reads work but leaves ROS service futures
    # pending forever, which is exactly what breaks the FSM buttons.
    if not requested_rmw and not requested_uri and not has_default_config:
        os.environ.pop('RMW_IMPLEMENTATION', None)
        os.environ.pop('CYCLONEDDS_URI', None)
        print('[GUI] DDS runtime: using system default RMW')
        return

    if requested_uri.startswith('file://'):
        requested_path = requested_uri.removeprefix('file://')
        if os.path.isfile(requested_path):
            if not requested_rmw:
                os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
            print(
                f"[GUI] DDS runtime: RMW={os.environ.get('RMW_IMPLEMENTATION', '<default>')} URI={requested_uri}"
            )
            return

        if has_default_config:
            print(
                f'[GUI] DDS config no disponible en {requested_path}; '
                f'se usara {DEFAULT_CYCLONEDDS_URI}'
            )
            os.environ['CYCLONEDDS_URI'] = DEFAULT_CYCLONEDDS_URI
            if not requested_rmw:
                os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
            print(
                f"[GUI] DDS runtime: RMW={os.environ.get('RMW_IMPLEMENTATION', '<default>')} URI={os.environ['CYCLONEDDS_URI']}"
            )
            return

        print(f'[GUI] DDS config no disponible en {requested_path}; usando RMW por defecto')
        os.environ.pop('CYCLONEDDS_URI', None)
        if not requested_rmw:
            os.environ.pop('RMW_IMPLEMENTATION', None)
        return

    if not requested_uri and has_default_config:
        os.environ['CYCLONEDDS_URI'] = DEFAULT_CYCLONEDDS_URI
        if not requested_rmw:
            os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        print(
            f"[GUI] DDS runtime: RMW={os.environ.get('RMW_IMPLEMENTATION', '<default>')} URI={os.environ['CYCLONEDDS_URI']}"
        )
        return

    print(f"[GUI] DDS runtime: RMW={requested_rmw or '<default>'} URI={requested_uri or '<default>'}")



def main():
    _sanitize_dds_environment()
    app = QApplication(sys.argv)


    # Creamos la ventana, diferimos el arranque ROS para no bloquear la GUI
    w = MainWindow(defer_ros_start=True)
    w.show()


    # Arranca ROS tras pintar la ventana
    QTimer.singleShot(0, w.start_ros)


    sys.exit(app.exec())




if __name__ == '__main__':
    main()