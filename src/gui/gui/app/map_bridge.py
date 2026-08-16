# cuadriga_gui/app/map_bridge.py
from PySide6.QtCore import QObject, Signal, Slot

class MapBridge(QObject):
    # Señales expuestas a Python (para que MainWindow .connect(...))
    guidedTarget = Signal(float, float)      # lat, lon
    plannedWaypoint = Signal(float, float)   # lat, lon
    clearPlanned = Signal()
    removeWaypointIndex = Signal(int)
    mapLayerChanged = Signal(str)
    demCoverageStatus = Signal(str, bool)

    def __init__(self, parent=None):
        super().__init__(parent)

    # Slots invocables desde JS (bridge.onGuidedTarget(...))
    @Slot(float, float)
    def onGuidedTarget(self, lat: float, lon: float):
        self.guidedTarget.emit(lat, lon)

    @Slot(float, float)
    def onPlannedWaypoint(self, lat: float, lon: float):
        self.plannedWaypoint.emit(lat, lon)

    @Slot()
    def onClearPlanned(self):
        self.clearPlanned.emit()

    @Slot(int)
    def onRemoveWaypointIndex(self, idx: int):
        try:
            i = int(idx)
        except Exception:
            return
        self.removeWaypointIndex.emit(i)   # <-- reemite a Qt/Python

    @Slot(str)
    def onMapLayerChanged(self, layer_name: str):
        self.mapLayerChanged.emit(str(layer_name))

    @Slot(str, bool)
    def onDemCoverageStatus(self, message: str, valid: bool):
        self.demCoverageStatus.emit(str(message), bool(valid))