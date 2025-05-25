
##################################################################
##################################################################
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flexible_planner_train.py
─────────────────────────
Nodo ROS 2 que entrena online con PPO:
  • Selección robusta del target (goal visible / frontier válido)
  • Génesis de minipaths flexibles (policy CNN + LSTM + búsqueda abanico)
  • Recompensas por progreso, llegada, colisión, seguridad, longitud de ruta
  • Publicación de Path + Markers y topics auxiliares
  • Guardado de pesos por episodio y métricas en TensorBoard
"""

# ==============  IMPORTS  =================================================
import math, random, time, os, pathlib, copy, numpy as np, tensorflow as tf
import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg      import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import (PoseArray, PoseStamped,
                               Point, Vector3, Twist)
from visualization_msgs.msg import Marker
from std_msgs.msg       import Header, Bool
from scipy.ndimage      import (binary_dilation,
                                generate_binary_structure,
                                distance_transform_edt)

import datetime
from argj801_ctl_platform_interfaces.msg import CmdThrottleMsg
from grid_map_msgs.msg import GridMap

# ==============  PARÁMETROS GLOBALES  =====================================
PATCH           = 128                       # lado del parche (celdas)
CLEAR_MIN       = 3.4                       # m (holgura waypoint)
GOAL_VIS_OK     = 4                         # ciclos “goal visible” → OK
GOAL_RADIUS     = 3.0                       # m para “goal reached”
RADII           = [2.3, 3.6,5.9]           # radios candidatos
ANGLES          = np.linspace(-math.pi/3,
                               math.pi/3, 11)   # ±60° (10 pasos)
MAX_WPS_EP      = 60
# Velocidades límite
MIN_VEL = 1.0          # m/s  (velocidad mínima deseada)
MAX_VEL = 6.0          # m/s  (velocidad máxima permitida)
LOOK_A  = 2.0    # m (aceptación)
# PPO
ROLLOUT_STEPS   = 1024
BATCH_SZ        = 256
EPOCHS          = 500
MAX_UPDATES     = 10
GAMMA           = 0.99
GAE_LAMBDA      = 0.95
CLIP_EPS        = 0.2
LR_ACTOR        = 3e-4
LR_CRITIC       = 1e-3
STD_START       = 0.3
STD_MIN         = 0.05
STD_DECAY       = 0.995
MAX_EPISODES = 300 
MAX_TILT = 1.0
MAX_STEERING = 70.0
MAX_THROTTLE = 70.0
WHEEL_BASE = 1.35        # m – distancia ejes
DELTA_MAX  = math.radians(32)   # giro máx. ruedas
K_STEER_LP = 0.7         # filtro exp. en steering
K_THROTTLE_LP = 0.4      # filtro exp. en throttle
KP_SPEED   = 10.0       # PID (solo P)  → throttle
MAX_DTHR   = 2.0        # rampa throttle / ciclo
V_MAX      = 3.0         # m/s (nuevo techo)
V_MIN      = 0.6         # m/s (no stall)
THR_IDLE = 2.0
DECEL_STOP = 0.20
HEIGHT_LAYER = "elevation"               # nombre de la capa de altura
MAX_HEIGHT   =  5.0       
# Terreno / pendiente ───────────────────────────────────────────────
MAX_SLOPE_TAN   = math.tan(math.radians(15.0))   # ≃ 15°  → pendiente límite
SLOPE_COST_GAIN = 20.0                            # peso en la métrica de coste

# ─────────────────────────── stuck / reverse ────────────────────────────
STUCK_DIST_THR   = 0.01     # m – avance mínimo para no considerarlo atascado
STUCK_CYCLES_MAX = 120       # nº de ciclos consecutivos sin progreso
REV_STEPS        = 30       # ciclos de marcha atrás
REV_THROTTLE     = -0.4 * MAX_THROTTLE
REV_STEER_SIGN   = +1      # cambia a −1 si quieres salir girando al otro lado
# ─────────────────────────── scan extra ─────────────────────────────────
UNKNOWN_FRAC_THR = 0.30     # % de celdas -1 en el frontal que dispara giro




DTYPE = np.float32


RUN_DIR = pathlib.Path.home() /  "PHD" / "ARGOJ8_IA" / "weights"
RUN_DIR.mkdir(exist_ok=True)

# ==============  UTILS GEOMÉTRICOS  =======================================
def l2(a, b):  # distancia euclídea 2-D
    return math.hypot(b[0]-a[0], b[1]-a[1])

def idx_from_world(info, pt):
    res = info.resolution
    return (int((pt[0]-info.origin.position.x)/res),
            int((pt[1]-info.origin.position.y)/res))

def distance(a,b): return math.hypot(b[0]-a[0], b[1]-a[1])


def height_at(arr_h, info_h, pt):
    """
    Altura en ‘pt’ para un GridMap:
      * info_h.pose.position  = centro del mapa
      * longitudes = info_h.length_x/y
    """
    res = info_h.resolution
    ox = info_h.pose.position.x - info_h.length_x / 2.0
    oy = info_h.pose.position.y - info_h.length_y / 2.0
    i = int((pt[0] - ox) / res)
    j = int((pt[1] - oy) / res)
    if 0 <= j < arr_h.shape[0] and 0 <= i < arr_h.shape[1]:
        return float(arr_h[j, i])
    return 0.0

def slope_ok(arr_h, info, a, b):
    """
    Comprueba la pendiente media entre ‘a’ y ‘b’.
    Devuelve (ok:bool, slope:float).
    """
    dz = abs(height_at(arr_h, info, b) - height_at(arr_h, info, a))
    dx = distance(a, b)
    if dx < 1e-6:
        return True, 0.0
    s  = dz / dx
    return s <= MAX_SLOPE_TAN, s

def fuse_dynamic_layer(grid_static, obstacles, info, decay_steps=5):
    grid = grid_static.copy()
    res  = info.resolution
    H, W = grid.shape
    for obs in obstacles:                       # PoseArray con vel.z = radius
        x, y, r = obs.position.x, obs.position.y, obs.orientation.w
        i0 = int((x - info.origin.position.x) / res)
        j0 = int((y - info.origin.position.y) / res)
        rad = int(r / res)
        for dj in range(-rad, rad+1):
            for di in range(-rad, rad+1):
                if di*di+dj*dj <= rad*rad:
                    j, i = j0+dj, i0+di
                    if 0 <= j < H and 0 <= i < W:
                        grid[j, i] = min(100, grid[j, i]+100)  # marca ocupado
    return grid




def gridmap_to_numpy(msg: GridMap, layer=HEIGHT_LAYER):
    """
    Devuelve un np.ndarray (H×W) con la capa pedida.
    Celdas NaN → 0.0   (puedes cambiar la política si prefieres).
    """
    if layer not in msg.layers:
        return None
    idx_layer = msg.layers.index(layer)
    h  = msg.data[idx_layer].layout.dim[1].size
    w  = msg.data[idx_layer].layout.dim[0].size
    buf = np.asarray(msg.data[idx_layer].data, dtype=np.float32)
    arr = buf.reshape((h, w))
    arr = np.nan_to_num(arr, nan=0.0)

    return arr , msg.info


def bres_free(grid, info, a, b):
    """Bresenham + bloqueo: (-1) desconocido ó >=100 obstáculo."""
    i0,j0 = idx_from_world(info,a)
    i1,j1 = idx_from_world(info,b)
    di,dj = abs(i1-i0), abs(j1-j0)
    si = 1 if i0<i1 else -1
    sj = 1 if j0<j1 else -1
    err=di-dj
    H,W = grid.shape
    while True:
        if not (0<=i0<W and 0<=j0<H): return False
        v = grid[j0,i0]
        if v==-1 or v>=100:          return False
        if (i0,j0)==(i1,j1):         return True
        e2=2*err
        if e2>-dj: err-=dj; i0+=si
        if e2< di: err+=di; j0+=sj

def clearance_ok(grid, info, pt, r_m):
    i,j = idx_from_world(info,pt)
    r = int(r_m/info.resolution)
    H,W = grid.shape
    for dj in range(-r, r+1):
        for di in range(-r, r+1):
            x,y = i+di, j+dj
            if 0<=x<W and 0<=y<H and (grid[y,x]==-1 or grid[y,x]>=100):
                return False
    return True

# ==============  RED CNN + LSTM  ==========================================
def build_policy():
    g   = tf.keras.Input(shape=(PATCH,PATCH,2), name="grid")
    st  = tf.keras.Input(shape=(6,),            name="state")
    w0  = tf.keras.Input(shape=(2,),            name="wp0")
    # CNN
    x = tf.keras.layers.Conv2D(16,3,padding="same",activation="relu")(g)
    x = tf.keras.layers.MaxPooling2D()(x)
    x = tf.keras.layers.Conv2D(32,3,padding="same",activation="relu")(x)
    x = tf.keras.layers.GlobalAveragePooling2D()(x)
    z = tf.keras.layers.Concatenate()([x,st])
    # LSTM un paso
    h0 = tf.keras.layers.Dense(128,activation="tanh")(z)
    c0 = tf.keras.layers.Dense(128,activation="tanh")(z)
    lstm = tf.keras.layers.LSTMCell(128)
    h1,_ = lstm(w0,[h0,c0])
    delta = tf.keras.layers.Dense(3,activation="tanh")(h1)
    return tf.keras.Model([g,st,w0], delta, name="policy")

# ==============  NODO PRINCIPAL  ==========================================
class FlexPlanner(Node):
    def __init__(self):
        super().__init__("flexible_trainer")

        # --- Subscripciones
        qos=10
        self.create_subscription(Odometry,      "/ARGJ801/odom_demo",            self.cb_odom,     qos)
        self.create_subscription(PoseArray,     "/goal",            self.cb_goal,     qos)
        self.create_subscription(OccupancyGrid, "/occupancy_grid",  self.cb_grid,     50)
        self.create_subscription(PoseArray,     "/safe_frontier_points_centroid",
                                 self.cb_frontier, qos)
        self.create_subscription(Bool,"/virtual_collision", self.cb_collision, qos)
        self.create_subscription(Bool,"/reset_confirmation",self.cb_reset_conf,qos)
        self.create_subscription(Bool,"/goal_reached",self.cb_goal_reached,qos)
        #self.create_subscription(PoseArray, "/obstacle_navigation_nodes_lidar",self.cb_obstacles, 10)
        self.create_subscription(GridMap,"/terrain_grid",self.cb_heightmap, 10)

        # --- Publicadores
        self.path_pub  = self.create_publisher(Path,  "/global_path_predicted", qos)
        self.wps_pub   = self.create_publisher(Marker,"/path_waypoints_marker", qos)
        self.coll_pub  = self.create_publisher(Bool,  "/virtual_collision", qos)
        #self.goal_pub  = self.create_publisher(Bool,  "/goal_reached", qos)
        latched=QoSProfile(depth=1,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reset_pub = self.create_publisher(Bool,  "/reset_request", latched)
        self.cmd_pub   = self.create_publisher(CmdThrottleMsg, "/ARGJ801/cmd_throttle_msg", qos)

        # --- Estado ROS
        self.waiting_reset=False
        self.current_path = []     # lista de waypoints activos
        self.wp_index     = 1      # índice del wp que se está siguiendo
        self.pose=self.twist=None
        self.goal=None
        self.grid_msg=None
        self.frontiers=[]
        self.obstacles = []  
        self.collided=False
        self.ready = True 
        self.reset_t0=None
        self.episode_done=False
        self.ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.update_done=0
        self.max_seg  =0.6
        self.max_steps=60
        self.dev_thr  =0.8
        self.clear_min=0.4     # m
        self.max_seg_length      = 1.0
        self.reach_thr=0.4
        self.goal_reached_flag=False
        self.scanning     = False      # ← NUEVO
        self.scan_t0      = None
        self.SCAN_SECS    = 1.5        # duración de un barrido 360°
        self.height_map_msg = None
        #  —  estado marcha atrás
        self.stuck_counter     = 0
        self.reversing         = False
        self.rev_steps_left    = 0
        self.training = False

        self.last_cmd = CmdThrottleMsg()

        # --- Red y PPO
        self.policy = build_policy()
        self.log_std = tf.Variable(np.log(STD_START*np.ones(3,np.float32)),
                                   trainable=True)
        self.opt_actor  = tf.keras.optimizers.Adam(LR_ACTOR)
        self.opt_critic = tf.keras.optimizers.Adam(LR_CRITIC)
        self.value_net  = tf.keras.Sequential([
            tf.keras.layers.Input(shape=(38,)),     # 32 (patch emb.) + 5 (state)
            tf.keras.layers.Dense(128,activation="tanh"),
            tf.keras.layers.Dense(1)
        ])

        # TensorBoard
        self.writer=tf.summary.create_file_writer(str(RUN_DIR/
                                                      time.strftime("run_%Y%m%d_%H%M%S")))
        # Buffers PPO
        self.reset_buffers()

        # Episodios
        self.episode=0
        self.goal_counter=0
        self.goals_in_world=random.randint(5,7)
        self.goal_vis=0

        self.create_timer(0.1, self.step)
        self.get_logger().info("Flexible planner + PPO listo")

    # ---------- Callbacks ROS ----------
    def cb_goal_reached(self, msg: Bool):
        if not msg.data:
            return
        # descarta si acabamos de reiniciar (<2 s) o hay colisión activa
        if (self.reset_t0 and
            (self.get_clock().now() - self.reset_t0).nanoseconds < 2e9):
            return
        if self.collided:
            return
        self.goal_reached_flag = True
    def cb_odom(self,m):
        self.pose=m.pose.pose; self.twist=m.twist.twist
    # def cb_obstacles(self, msg: PoseArray):
    #     self.obstacles = msg.poses

    def cb_goal(self,m):
        self.goal=m.poses[0] if m.poses else None
    # def cb_grid(self,m):
    #     self.grid_msg=m

    def cb_heightmap(self, msg: GridMap):
        """Guarda el GridMap de terreno más reciente."""
        self.height_map_msg = msg



    def cb_grid(self, m: OccupancyGrid):
        # mezclamos capa estática + móviles
        static_arr = np.array(m.data, dtype=np.int8).reshape((m.info.height,
                                                            m.info.width))
        if self.obstacles:
            dyn_arr = fuse_dynamic_layer(static_arr, self.obstacles, m.info)
            self.grid_dyn = dyn_arr                    # guarda dinámico
        else:
            self.grid_dyn = static_arr
        self.grid_msg = m                              # conserva original

    def cb_frontier(self,m):
        self.frontiers=[(p.position.x,p.position.y) for p in m.poses]
    def cb_collision(self,m):
        self.collided = bool(m.data)
    # def cb_reset_conf(self, msg:Bool):
    #     if msg.data:
    #         self.waiting_reset = False
    #         self.get_logger().info("[Reset] confirmado por el supervisor")
    #         self.goal_counter = 0
    #         self.goals_in_world = random.randint(5,7)
    #         self.collided = False
    #         self.reset_buffers()

    def cb_reset_conf(self, msg: Bool):
        if msg.data:
            self.waiting_reset = False
            self.ready         = False          # ← PAUSA hasta que el mundo cargue
            self.reset_t0      = self.get_clock().now()
            self.goal_counter  = 0
            self.goals_in_world = random.randint(5, 7)
            self.collided      = False
            self.reset_buffers()
            self.get_logger().info("[Reset] confirmado por el supervisor")


    # --------------------------------------------------------------
    #  Mantiene el vehículo quieto compensando la pendiente
    # --------------------------------------------------------------
    def _hold_position(self):
        """
        Publica repetidamente un CmdThrottleMsg para contrarrestar
        cualquier velocidad residual cuando el robot *debe* estar parado
        (p.ej. durante el entrenamiento PPO tras llegar al goal).
        · Lee la velocidad lineal real            -> v_real  (m/s)
        · Aplica un control P  (con limitación)   -> throttle
        · Steering a 0     (para no cambiar rumbo)
        """
        v_real = math.hypot(self.twist.linear.x, self.twist.linear.y)

        KP_HOLD = 80.0                 # afinado empírico
        thr_cmd = -KP_HOLD * v_real    # signo opuesto a la velocidad
        thr_cmd = np.clip(thr_cmd,
                          -0.40 * MAX_THROTTLE,   # freno motor hacia atrás
                           0.40 * MAX_THROTTLE)   # freno motor hacia delante

        msg = CmdThrottleMsg()
        msg.throttle = thr_cmd
        msg.steering = 0.0
        self.cmd_pub.publish(msg)
        self.last_cmd = msg



    def _brake(self):
        msg = CmdThrottleMsg(); msg.throttle = 0.0; msg.steering = 0.0
        return msg
    
    def _reverse_cmd(self):
        msg = CmdThrottleMsg()
        msg.throttle  = REV_THROTTLE
        msg.steering  = REV_STEER_SIGN * 0.35 * MAX_STEERING

        return msg

    def _front_unknown_fraction(self, patch_occ):
        """
        Devuelve fracción de celdas desconocidas (-1) en la franja frontal
        (filas [R-4:R+4], columnas [R:PATCH]).
        """
        R = PATCH//2
        front = patch_occ[R-4:R+4, R:]
        total = front.size
        unknown = np.count_nonzero(front == -1)
        return unknown / total if total else 0.0


    def _spin_in_place(self, sign=+1):
        msg = CmdThrottleMsg(); msg.throttle = 0.0
        msg.steering = sign * 0.85 * MAX_STEERING
        self.get_logger().info(f"[SPIN] {sign:+d}")
        return msg

    @staticmethod
    def _wrap_angle(a):
        return (a + math.pi) % (2*math.pi) - math.pi


    def _yaw_from_quaternion(self, q):
        """Devuelve yaw (rad) desde geometry_msgs.msg.Quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _roll_pitch_from_quaternion(self, q):
        """Devuelve (roll, pitch) en radianes."""
        sinr_cosp = 2*(q.w*q.x + q.y*q.z)
        cosr_cosp = 1 - 2*(q.x*q.x + q.y*q.y)
        roll  = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2*(q.w*q.y - q.z*q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))   # clamp numérico
        return roll, pitch



    def _global_to_local(self, dx, dy, yaw):
        """Convierte ΔX,ΔY de frame 'map' a 'base_link'."""
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        return dx*cos_y - dy*sin_y, dx*sin_y + dy*cos_y


    def entry_point_in_grid(self,cp, fp, info):
        """
        Dado cp=(x0,y0) y fp=(x1,y1), devuelve el primer punto
        dentro del bbox del grid [x0,x1]×[y0,y1] por donde pasa
        el segmento cp→fp, o None si no intersecta.
        Usa el algoritmo de Liang–Barsky.
        """
        x0 = info.origin.position.x
        y0 = info.origin.position.y
        x1 = x0 + info.width  * info.resolution
        y1 = y0 + info.height * info.resolution

        dx = fp[0] - cp[0]
        dy = fp[1] - cp[1]

        p = [-dx,  dx, -dy,  dy]
        q = [ cp[0]-x0, x1-cp[0], cp[1]-y0, y1-cp[1]]

        u0, u1 = 0.0, 1.0
        for pi, qi in zip(p, q):
            if pi == 0:
                if qi < 0:
                    return None  # paralelo y completamente fuera
            else:
                t = qi / pi
                if pi < 0:
                    u0 = max(u0, t)
                else:
                    u1 = min(u1, t)
        if u0 > u1:
            return None

        # punto de entrada al grid:
        return (cp[0] + u0*dx, cp[1] + u0*dy)

    # ---------- Parche local -----------
    # def extract_patch(self):
    #     info=self.grid_msg.info
    #     H,W=info.height,info.width
    #     # arr=np.array(self.grid_msg.data,dtype=np.int8).reshape((H,W))
    #     arr = self.grid_dyn.copy() # capa dinámica #nueva
    #     cp=(self.pose.position.x,self.pose.position.y)
    #     ci=int((cp[0]-info.origin.position.x)/info.resolution)
    #     cj=int((cp[1]-info.origin.position.y)/info.resolution)
    #     i_lo,i_hi=ci-PATCH//2,ci+PATCH//2
    #     j_lo,j_hi=cj-PATCH//2,cj+PATCH//2
    #     i0,i1=max(i_lo,0),min(i_hi,W)
    #     j0,j1=max(j_lo,0),min(j_hi,H)
    #     patch=arr[j0:j1,i0:i1]
    #     pad=((j0-j_lo,j_hi-j1),(i0-i_lo,i_hi-i1))
    #     patch=np.pad(patch,pad,'constant',constant_values=-1)
    #     norm=((patch+1)/101.0).astype(np.float32)   # [-1,100] → [0,1]
    #     return np.expand_dims(norm,-1), arr, info



    def extract_patch(self):
        """Devuelve (patch_2ch, full_occ, info). patch shape = PATCH×PATCH×2."""
        # -------- occupancy (canal 0) ----------
        info = self.grid_msg.info
        H, W = info.height, info.width
        occ_full = self.grid_dyn.copy()                 # (H×W) int8
        cp = (self.pose.position.x, self.pose.position.y)
        ci = int((cp[0] - info.origin.position.x) / info.resolution)
        cj = int((cp[1] - info.origin.position.y) / info.resolution)
        i_lo, i_hi = ci - PATCH//2, ci + PATCH//2
        j_lo, j_hi = cj - PATCH//2, cj + PATCH//2
        i0, i1 = max(i_lo, 0), min(i_hi, W)
        j0, j1 = max(j_lo, 0), min(j_hi, H)
        occ_patch = occ_full[j0:j1, i0:i1]
        pad = ((j0 - j_lo, j_hi - j1), (i0 - i_lo, i_hi - i1))
        occ_patch = np.pad(occ_patch, pad, 'constant', constant_values=-1)
        norm_occ = ((occ_patch + 1) / 101.0).astype(np.float32)      #  [-1,100]→[0,1]

        # -------- height (canal 1) ------------
        # lienzo vacío del mismo tamaño que occ_patch
        norm_h = np.zeros_like(norm_occ, dtype=np.float32)

        if self.height_map_msg is not None:
            # ① convierto todo el GridMap a np.array
            h_full,hm_info = gridmap_to_numpy(self.height_map_msg)
            if h_full is not None:
                
                res_h   = hm_info.resolution
                ox_h    = hm_info.pose.position.x - hm_info.length_x / 2.0
                oy_h    = hm_info.pose.position.y - hm_info.length_y / 2.0
                Hh, Wh  = h_full.shape

                # ③ recorremos cada celda del lienzo y copiamos valor si existe
                for jj in range(PATCH):
                    y_world = cp[1] + (jj - PATCH//2) * info.resolution
                    jh = int((y_world - oy_h) / res_h)
                    if not (0 <= jh < Hh):       # fuera -> queda 0
                        continue
                    for ii in range(PATCH):
                        x_world = cp[0] + (ii - PATCH//2) * info.resolution
                        ih = int((x_world - ox_h) / res_h)
                        if 0 <= ih < Wh:
                            norm_h[jj, ii] = np.clip(
                                h_full[jh, ih] / MAX_HEIGHT, -1.0, 1.0
                            )

        patch_2ch = np.stack([norm_occ, norm_h], axis=-1)            # (PATCH,PATCH,2)
        return patch_2ch, occ_full, info


    # ---------- Target robusto ----------
    def choose_target(self, cp, grid, info):
        # 1) goal directo si está definido y visible
        if self.goal:
            gp = (self.goal.position.x, self.goal.position.y)
            if bres_free(grid, info, cp, gp):
                return gp, "GOAL"

        # 2) cualquier frontier
        if self.frontiers:
            best = min(self.frontiers, key=lambda f: l2(f, gp if self.goal else cp))
            return best, "FRONTIER"

        # 3) nada de nada
        return None, "NONE"




    def bres_line_free(self,grid, info, a, b):
        def idx(p):
            return (int((p[0]-info.origin.position.x)/info.resolution),
                    int((p[1]-info.origin.position.y)/info.resolution))
        i0,j0 = idx(a); i1,j1 = idx(b)
        di,dj = abs(i1-i0), abs(j1-j0); si = 1 if i0<i1 else -1; sj = 1 if j0<j1 else -1
        err = di-dj; H,W = grid.shape
        while True:
            if not (0<=i0<W and 0<=j0<H) or grid[j0,i0] == -1 or grid[j0,i0] >= 100:
                return False
            if (i0,j0)==(i1,j1): return True
            e2=2*err
            if e2>-dj: err-=dj; i0+=si
            if e2< di: err+=di; j0+=sj


    def densify(self, path):
        out = [path[0]]
        for a, b in zip(path, path[1:]):
            d = distance(a, b)
            if d > self.max_seg_length:
                steps = int(math.ceil(d / self.max_seg_length))
                for i in range(1, steps):
                    t = i / steps
                    out.append((a[0]*(1-t)+b[0]*t, a[1]*(1-t)+b[1]*t))
            out.append(b)
        return out
    
    # def generate_flexible_path(self, start, target, grid, info):
    #     path=[start]
    #     cp=start
    #     step_len=self.max_seg
    #     angles=np.linspace(-math.pi/3, math.pi/3, 10)   # ±60°
    #     radii=[step_len*0.5, step_len, step_len*1.5]

    #     for _ in range(self.max_steps):
    #         best=None; best_cost=float("inf")
    #         vec_t=(target[0]-cp[0], target[1]-cp[1])
    #         ang0=math.atan2(vec_t[1], vec_t[0])

    #         for r in radii:
    #             for a_off in angles:
    #                 ang=ang0 + a_off
    #                 cand=(cp[0]+r*math.cos(ang), cp[1]+r*math.sin(ang))

    #                 # ---------- filtro ③  “celda debe ser conocida y libre” ----------
    #                 i = int((cand[0] - info.origin.position.x) / info.resolution)
    #                 j = int((cand[1] - info.origin.position.y) / info.resolution)
    #                 if not (0 <= i < grid.shape[1] and 0 <= j < grid.shape[0]):
    #                     continue                             # fuera de mapa
    #                 if grid[j, i] == -1 or grid[j, i] >= 100:
    #                     continue                             # desconocida u obstáculo
    #                 # -----------------------------------------------------------------

    #                 if not self.bres_line_free(grid, info, cp, cand):
    #                     continue
    #                 if not clearance_ok(grid, info, cand, self.clear_min):
    #                     continue

    #                 cost = distance(cand, target) - 0.5*self.clear_min
    #                 if cost < best_cost:
    #                     best_cost, best = cost, cand

    #         if best is None:
    #             break
    #         path.append(best)
    #         cp = best
    #         if distance(cp, target) < self.reach_thr:
    #             break

    #     path.append(target)
    #     return self.densify(path)       



    def generate_flexible_path(self, start, target, grid, info):
        """Como antes, pero descarta y penaliza pendientes > MAX_SLOPE_TAN."""
        h_arr, hm_info= gridmap_to_numpy(self.height_map_msg) if self.height_map_msg else (None,None)

        path = [start]
        cp   = start
        step_len = self.max_seg
        angles = np.linspace(-math.pi/8,  math.pi/8, 10)     # ±22.5°
        radii  = [step_len*0.5, step_len, step_len*1.5]

        for _ in range(self.max_steps):
            best = None
            best_metric = (float("inf"), float("inf"))   # (slope, dist)


            vec_t = (target[0]-cp[0], target[1]-cp[1])
            ang0  = math.atan2(vec_t[1], vec_t[0])

            for r in radii:
                for a_off in angles:
                    ang  = ang0 + a_off
                    cand = (cp[0] + r*math.cos(ang),
                            cp[1] + r*math.sin(ang))

                    i, j = idx_from_world(info, cand)
                    if not (0 <= i < grid.shape[1] and 0 <= j < grid.shape[0]):
                        continue
                    if grid[j, i] == -1 or grid[j, i] >= 100:
                        continue
                    if not self.bres_line_free(grid, info, cp, cand):
                        continue
                    if not clearance_ok(grid, info, cand, self.clear_min):
                        continue

                    # ---------- pendiente ----------
                    if h_arr is not None:
                        ok_slope, slope = slope_ok(h_arr, hm_info, cp, cand)
                        if not ok_slope:
                            continue                     # demasiado empinado
                    else:
                        slope = 0.0
                    # clear = distance(cand,cp)
                    # C_OBS = 4.0
                    # # cost = (distance(cand, target)
                    # #         + SLOPE_COST_GAIN * s        # penaliza subir/bajar
                    # #         - 0.5 * self.clear_min)
                    # cost = (distance(cand, target)
                    #          + SLOPE_COST_GAIN * s
                    #          + C_OBS / (clear + 1e-3)) 
                    # if cost < best_cost:
                    #     best_cost, best = cost, cand
                    dist    = distance(cand, target)
                    metric  = (slope, dist)          # primero slope, luego dist
                    if metric < best_metric:
                        best_metric, best = metric, cand

            if best is None:
                break
            path.append(best)
            cp = best
            if distance(cp, target) < self.reach_thr:
                break

        path.append(target)
        return self.densify(path)



####################################################antiguo
    # ---------- Siguiente waypoint ----------
    def next_waypoint(self, cp, tgt, grid, info, patch):
        # 1) Δ “preferido” de la policy
        state=np.array([0,0,tgt[0]-cp[0],tgt[1]-cp[1]],np.float32)
        patch_b = patch[None,...]
        state_b = state[None,:]
        delta=self.policy([patch_b, state_b, np.zeros((1,2),np.float32)],
                          training=False)[0].numpy()
        ang0=math.atan2(delta[1],delta[0]) if np.linalg.norm(delta)>1e-3 \
             else math.atan2(tgt[1]-cp[1], tgt[0]-cp[0])

        # 2) abanico de candidatos
        best=None; best_cost=float("inf")
        h_arr = gridmap_to_numpy(self.height_map_msg) if self.height_map_msg else None

        for r in RADII:
            for off in ANGLES:
                ang=ang0+off
                cand=(cp[0]+r*math.cos(ang), cp[1]+r*math.sin(ang))
                i,j=idx_from_world(info,cand)
                if not (0<=i<grid.shape[1] and 0<=j<grid.shape[0]): continue
                if grid[j,i]==-1 or grid[j,i]>=100: continue
                if not bres_free(grid,info,cp,cand):          continue
                if not clearance_ok(grid,info,cand,CLEAR_MIN):continue

                # ----- pendiente -----
                if h_arr is not None:
                    ok_slope, s = slope_ok(h_arr, info, cp, cand)
                    if not ok_slope:
                        continue
                else:
                    s = 0.0

                cost = l2(cand, tgt) + SLOPE_COST_GAIN * s - 0.5 * CLEAR_MIN
                if cost < best_cost:
                    best_cost, best = cost, cand
        return best, delta
    ##########################################
    def compute_obstacle_distances(self,patch, info):
        """
        patch: array PATCH×PATCH con valores [-1..100]
        info.resolution en m/celda
        Devuelve d_front, d_left, d_right en metros (∞ si libre).
        """
        R = PATCH//2
        # frontal: filas [R-4:R+4], columnas [R:PATCH]
        front = patch[R-4:R+4, R:]
        # izquierda: filas [R:PATCH], cols [0:8]
        left  = patch[R:, 0:8]
        # derecha: rows [R:PATCH], cols [PATCH-8:PATCH]
        right = patch[R:, PATCH-8:]
        def min_dist(slice_):
            # índice de la primera celda ≥0 que sea obstáculo (valor ≥0)
            occ = np.where(slice_>=0, slice_, np.inf)
            idx = np.unravel_index(np.argmin(occ), occ.shape)
            # distancia en células = columna del idx o fila según slice
            # adaptamos: para front, medimos el eje x; para left/right, eje y
            return np.inf if occ[idx]==np.inf else (idx[1] if slice_ is front else idx[0])
        d_front = min_dist(front) * info.resolution
        d_left  = min_dist(left)  * info.resolution
        d_right = min_dist(right) * info.resolution
        return d_front, d_left, d_right



    ##########################################



    # def follow_path(self, cp):
    #     # si no hay ruta o ya terminamos → quieto
    #     if len(self.current_path) <= self.wp_index:
    #         self.cmd_pub.publish(Twist())
    #         return

    #     wp = self.current_path[self.wp_index]

    #     # ¿hemos llegado a este wp?
    #     if l2(cp, wp) < LOOK_A:
    #         self.wp_index += 1
    #         if self.wp_index >= len(self.current_path):
    #             self.cmd_pub.publish(Twist())
    #             return
    #         wp = self.current_path[self.wp_index]

    #     dx, dy = wp[0] - cp[0], wp[1] - cp[1]
    #     d      = math.hypot(dx, dy)
    #     vx     = min(MAX_VEL, 1.5 * d) * dx / d
    #     vy     = min(MAX_VEL, 1.5 * d) * dy / d

    #     cmd = Twist()
    #     cmd.linear.x = vx
    #     cmd.linear.y = vy
    #     self.cmd_pub.publish(cmd)

    #     self.get_logger().info(
    #         f"[FOLLOW] wp {self.wp_index}/{len(self.current_path)-1} "
    #         f"→ v=({vx:.2f},{vy:.2f})")
###nuevo
    def _next_target_index(self, cp, look_ahead):
        """Devuelve el índice del primer waypoint a ≥ look_ahead del robot."""
        idx = self.wp_index
        while (idx + 1 < len(self.current_path)
            and l2(cp, self.current_path[idx]) < look_ahead):
            idx += 1
        return idx
######### antiguo###########################################
    # def follow_path(self, cp):
    #     """
    #     Sigue self.current_path publicando CmdThrottleMsg en lugar de Twist.
    #     cp: (x,y) posición actual del robot en world frame.
    #     """
    #     # 1) Ruta terminada → frena
    #     if self.wp_index >= len(self.current_path):
    #         stop = CmdThrottleMsg()
    #         stop.throttle = 0.0
    #         stop.steering = 0.0
    #         self.cmd_pub.publish(stop)
    #         return

    #     # 2) Look‐ahead adaptativo
    #     v_nom = MAX_VEL
    #     Ld = max(0.25, 0.4 * v_nom)
    #     self.wp_index = self._next_target_index(cp, Ld)
    #     tgt = self.current_path[self.wp_index]

    #     # 3) Cálculo del vector en world
    #     dx_g = tgt[0] - cp[0]
    #     dy_g = tgt[1] - cp[1]
    #     dist = math.hypot(dx_g, dy_g)
    #     if dist < 1e-3:
    #         stop = CmdThrottleMsg()
    #         stop.throttle = 0.0
    #         stop.steering = 0.0
    #         self.cmd_pub.publish(stop)
    #         return

    #     # 4) Pasa a coordenadas del robot
    #     yaw = self._yaw_from_quaternion(self.pose.orientation)
    #     dx, dy = self._global_to_local(dx_g, dy_g, yaw)

    #     # 5) Pure-Pursuit
    #     alpha = math.atan2(dy, dx)
    #     kappa = 2.0 * math.sin(alpha) / Ld

    #     # 6) Velocidades deseadas
    #     v_lin = max(MIN_VEL, min(MAX_VEL, 1.8 * dist))
    #     omega = kappa * v_lin

    #     # 7) Mapea a unidades de throttle/steering
    #     MAX_THROTTLE = 500.0   # unidad máxima de tu Ctrl
    #     MAX_STEERING = 500.0   # idem
    
    #     thr_cmd = (v_lin / MAX_VEL) * MAX_THROTTLE
    #     str_cmd = -(omega / (2.0)) * MAX_STEERING  # si 2.0 rad/s fuera tu ω máx.

    #     cmd = CmdThrottleMsg()
    #     cmd.throttle = max(-MAX_THROTTLE, min(MAX_THROTTLE, thr_cmd))
    #     cmd.steering = max(-MAX_STEERING, min(MAX_STEERING, str_cmd))

    #     self.cmd_pub.publish(cmd)
    #     self.get_logger().info(
    #         f"[FOLLOW-TH] wp={self.wp_index}/{len(self.current_path)-1}  "
    #         f"thr={cmd.throttle:.1f}  steer={cmd.steering:.1f}  "
    #         f"α={alpha*180/math.pi:+.1f}°"
    #     )
   
   ###################################
    # def follow_path(self, cp):
    #     """
    #     Sigue self.current_path publicando CmdThrottleMsg.
    #     Gira en sitio si α > 60°, curva mientras avanza si α ≤ 60°.
    #     """
    #     if self.wp_index >= len(self.current_path):
    #         stop = CmdThrottleMsg()
    #         stop.throttle = 0.0
    #         stop.steering = 0.0
    #         self.cmd_pub.publish(stop)
    #         return

    #     v_nom = MAX_VEL
    #     Ld = max(0.25, 0.4 * v_nom)
    #     self.wp_index = self._next_target_index(cp, Ld)
    #     tgt = self.current_path[self.wp_index]

    #     dx_g = tgt[0] - cp[0]
    #     dy_g = tgt[1] - cp[1]
    #     dist = math.hypot(dx_g, dy_g)
    #     if dist < 1e-3:
    #         stop = CmdThrottleMsg()
    #         stop.throttle = 0.0
    #         stop.steering = 0.0
    #         self.cmd_pub.publish(stop)
    #         return

    #     yaw = self._yaw_from_quaternion(self.pose.orientation)
    #     dx, dy = self._global_to_local(dx_g, dy_g, yaw)

    #     alpha = math.atan2(dy, dx)
    #     kappa = 2.0 * math.sin(alpha) / Ld

    #     if abs(alpha) > math.radians(60):
    #         thr_cmd = 0.0
    #         str_cmd = math.copysign(0.8 * MAX_STEERING, alpha)
    #     else:
    #         v_lin  = max(MIN_VEL, min(MAX_VEL, 1.8 * dist))
    #         omega  = kappa * v_lin
    #         thr_cmd = (v_lin / MAX_VEL) * MAX_THROTTLE
    #         str_cmd = -(omega / 2.0) * MAX_STEERING

    #     cmd = CmdThrottleMsg()
    #     cmd.throttle = max(-MAX_THROTTLE, min(MAX_THROTTLE, thr_cmd))
    #     cmd.steering = max(-MAX_STEERING, min(MAX_STEERING, str_cmd))
    #     self.cmd_pub.publish(cmd)
    #     self.last_cmd = cmd
    #     self.get_logger().info(
    #         f"[FOLLOW-TH] wp={self.wp_index}/{len(self.current_path)-1}  "
    #         f"α={alpha*57.3:+.1f}°  thr={cmd.throttle:.1f}  steer={cmd.steering:.1f}"
    #     )

####################################### bueno anterior
    def follow_path(self, cp):
        """
        Pure-Pursuit mejorado:
        · v_des depende de curvatura y pendiente
        · steering via Ackermann
        · throttle por control P en velocidad
        · filtros de 1er orden para suavizar inercia
        """
        if self.wp_index >= len(self.current_path):
            self.cmd_pub.publish(self._brake())
            return

        # 1) Look-ahead dependiente de velocidad
        v_curr = math.hypot(self.twist.linear.x, self.twist.linear.y)
        Ld     = np.clip(1.6 * v_curr, 0.8, 4.0)      # m
        self.wp_index = self._next_target_index(cp, Ld)
        tgt = self.current_path[self.wp_index]

        # 2) Transformación a frame del vehículo
        dx_g, dy_g = tgt[0] - cp[0], tgt[1] - cp[1]
        yaw = self._yaw_from_quaternion(self.pose.orientation)
        dx, dy = self._global_to_local(dx_g, dy_g, yaw)

        dist   = math.hypot(dx, dy)
        # if dist < 1e-2:
        #     self.cmd_pub.publish(self._brake()); return
        at_last_wp = (self.wp_index == len(self.current_path)-1)
        if at_last_wp and dist < 0.15:        # parada real
            self.cmd_pub.publish(self._brake())
            self.last_cmd = self._brake()
            return

        alpha  = math.atan2(dy, dx)
        kappa  = 2.0 * math.sin(alpha) / max(dist, 1e-3)   # curvatura





        # 3) Desired steering (Ackermann)
        delta_des = math.atan(WHEEL_BASE * kappa)
        delta_des = np.clip(delta_des, -DELTA_MAX, DELTA_MAX)
        str_cmd   = -(delta_des / DELTA_MAX) * MAX_STEERING   # map to units

        # 4) Desired speed (curvatura + pendiente)
        slope = 0.0
        if self.height_map_msg is not None:
            h_arr,hm_info = gridmap_to_numpy(self.height_map_msg)
            slope = abs(height_at(h_arr, hm_info, tgt) -
                        height_at(h_arr, hm_info, cp)) / dist
        # v_des = V_MAX * (1 - 0.7*abs(kappa)) * (1 - 0.5*slope)
        # v_des = np.clip(v_des, V_MIN, V_MAX)
        ##v_des = V_MAX * (1 - 0.7*abs(kappa)) * (1 - 0.5*slope)
        v_des = V_MAX * (1 - 0.7*abs(kappa))
        v_des *= 1.0 - 0.5*max(slope, 0)         # cuesta arriba
        v_des *= 1.0 - 0.2*max(-slope, 0)        # cuesta abajo (menos severo)
        v_des = np.clip(v_des, V_MIN, V_MAX)

        # desaceleración progresiva en los últimos 0.6 m
        if at_last_wp and dist < 0.6:
            v_des *= dist / 0.6

        # 5) Longitudinal control (P)
        # thr_raw = KP_SPEED * (v_des - v_curr)          # N-units
        err_v   = v_des - v_curr
        # integral mínima para vencer rozamiento
        self.int_err = getattr(self, 'int_err', 0.0) + err_v*0.01
        self.int_err = np.clip(self.int_err, -2.0, 2.0)
        ## thr_raw = KP_SPEED * err_v + self.int_err + THR_IDLE*np.sign(v_des)
        # añade “boost” +10 % si estamos subiendo más de 5 %
        boost = 0.10*MAX_THROTTLE if slope > 0.05 else 0.0
        thr_raw = KP_SPEED * err_v + self.int_err + THR_IDLE*np.sign(v_des) + boost
        thr_cmd = np.clip(thr_raw, -MAX_THROTTLE, MAX_THROTTLE)

        # 6) Filtros y rampa
        ##str_cmd = (1-K_STEER_LP) * str_cmd + K_STEER_LP * self.last_cmd.steering
        # thr_cmd = (1-K_THROTTLE_LP) * thr_cmd + K_THROTTLE_LP * self.last_cmd.throttle
        # # thr_cmd = (1-K_THROTTLE_LP)*thr_cmd + K_THROTTLE_LP*self.last_cmd.throttle
        # # d_thr   = np.clip(thr_cmd - self.last_cmd.throttle,
        # #                 -MAX_DTHR, MAX_DTHR)
        str_filt = (1-K_STEER_LP)*str_cmd + K_STEER_LP*self.last_cmd.steering
        thr_filt = (1-K_THROTTLE_LP)*thr_cmd + K_THROTTLE_LP*self.last_cmd.throttle

        d_thr   = np.clip(thr_filt - self.last_cmd.throttle,
                          -MAX_DTHR, MAX_DTHR)
        thr_cmd = self.last_cmd.throttle + d_thr
        str_cmd = str_filt

        # 7) Publica
        cmd = CmdThrottleMsg()
        cmd.throttle = thr_cmd
        cmd.steering = str_cmd
        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd

        self.get_logger().info(
            f"[CTRL] wp={self.wp_index}/{len(self.current_path)-1} "
            f"α={alpha*57.3:+.1f}° k={kappa:.3f} v={v_curr:.2f}->{v_des:.2f} "
            f"thr={thr_cmd:.0f} str={str_cmd:.0f}"
        )





############################################
 
###nuevo
    # ---------- Recompensa --------------
    def compute_reward(self, old_d, new_d, collided, reached, step_len,
                    overturned, min_obs_dist, is_spinning,slope_curr):
        """
        old_d, new_d: distancias a target
        min_obs_dist: mínimo entre d_front/d_left/d_right
        is_spinning: bool, si estamos girando in situ
        """
        r = 2.0*(old_d - new_d)      # progreso
        r -= 0.05                    # coste por paso
        r -= 0.1 * step_len          # coste ruta larga
        if reached:    r += 200
        if collided:   r -= 200
        if overturned: r -= 200
        # penaliza proximidad a obstáculo
        r -= 1.0 / (min_obs_dist + 1e-3)
        r -= 15.0 * slope_curr          # slope_curr = pendiente media del último segmento

        # bonifica giros in situ
        if is_spinning:
            r += 10.0
        return r

    # ---------- Buffers PPO -------------
    def reset_buffers(self):
        self.patch_buf=[]; self.state_buf=[]
        self.act_buf=[];   self.logp_buf=[]
        self.rew_buf=[];   self.val_buf=[]
        self.done_buf=[]
        self.vel_buf = []

    def step(self):

        if getattr(self, "training", True):
            self._hold_position()
            return                         # no hacemos nada más


        # 0 · Espera a que el mundo nuevo esté listo ------------------
        if not self.ready:
            ok_time = self.reset_t0 and \
                    (self.get_clock().now() - self.reset_t0).nanoseconds > 2e9
            ok_grid = self.grid_msg and \
                    np.any(np.array(self.grid_msg.data) != -1)
            if ok_time and self.pose and ok_grid:
                self.ready = True
                self.get_logger().info("[Reset] mundo cargado; reanudando")
            else:
                return                                   # sigue en pausa

        # 1 · Validaciones mínimas ------------------------------------
        if None in (self.pose, self.goal, self.grid_msg):
            return
        cp = (self.pose.position.x, self.pose.position.y)

        # ——— detector de atasco ——————————————————————————
        dist_progress = l2(cp, getattr(self, 'prev_pos', cp))
        if dist_progress < STUCK_DIST_THR:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        self.prev_pos = cp

        # #  ▶ si ya estamos en modo marcha atrás
        # if self.reversing:
        #     if self.rev_steps_left > 0:
        #         self.cmd_pub.publish(self._reverse_cmd())
        #         self.rev_steps_left -= 1
        #         return
        #     else:
        #         self.reversing = False          # fin de la maniobra
        #  ▶ entrar en marcha atrás si acumulamos ciclos atascados
        if self.stuck_counter >= STUCK_CYCLES_MAX:
            self.get_logger().warning("🚗  Atasco detectado → marcha atrás")
            self.reversing       = True
            self.rev_steps_left  = REV_STEPS

            if self.reversing:
                if self.rev_steps_left > 0:
                    self.cmd_pub.publish(self._reverse_cmd())
                    self.rev_steps_left -= 1
                    return
                else:
                    self.reversing = False          # fin de la maniobra
                self.stuck_counter   = 0
            return





    #---------- 2 · ¿estamos escaneando? ----------
        need_extra_scan = False
        if self.scanning:
            # ←--- PRIMERO comprobamos si **ya** hay un target
            patch, grid, info = self.extract_patch()
            tgt, mode = self.choose_target(cp, grid, info)
            self.get_logger().info(f"scanning: {self.scanning} {tgt}")
            if tgt is not None:               # ¡ya vemos algo!
                self.scanning = False         # cancela giro y sigue
            elif (self.get_clock().now() - self.scan_t0).nanoseconds < self.SCAN_SECS*1e9:
                self.cmd_pub.publish(self._spin_in_place(+1))
                return
            else:                             # terminó el giro completo
                self.scan_t0 = self.get_clock().now()   # arranca otro barrido
                return


        # ---------- 3 · parche + target ----------
        patch, grid, info = self.extract_patch()
        tgt, mode = self.choose_target(cp, grid, info)

        # 3.a  Si sigue sin target ⇒ iniciar un único barrido
        need_extra_scan = False
        if tgt is None:
            need_extra_scan = True
        else:
            #  nuevo: demasiado desconocido en frente aunque haya objetivo
            unknown_frac = self._front_unknown_fraction(patch[:, :, 0])
            if unknown_frac > UNKNOWN_FRAC_THR:
                need_extra_scan = True
                self.get_logger().info(f"🌑  {unknown_frac:.0%} frontal desconocido → escaneo")

        if need_extra_scan:
            self.scanning = True
            self.scan_t0  = self.get_clock().now()
            self.get_logger().info("🌪  Giro exploratorio")
            return

        # ---------- 4 · alinearse al objetivo antes de avanzar ----------
        # yaw_goal  = math.atan2(tgt[1]-cp[1], tgt[0]-cp[0])
        # yaw_robot = self._yaw_from_quaternion(self.pose.orientation)
        # yaw_err   = self._wrap_angle(yaw_goal - yaw_robot)

        # if abs(yaw_err) > math.radians(12):       # ±12°
        #     self.cmd_pub.publish(self._spin_in_place(math.copysign(1, yaw_err)))
        #     return                                # aún girando para encarar

        # 1.b Detector de vuelco --------------------------------------
        roll, pitch = self._roll_pitch_from_quaternion(self.pose.orientation)
        overturned  = abs(roll) > MAX_TILT or abs(pitch) > MAX_TILT
        if overturned:
            self.get_logger().warning("🚨  Robot volcado")
            self.reset_pub.publish(Bool(data=True))
            return                        # espera a que el mundo se reinicie


        # 2 · Parche local y target robusto ---------------------------
        patch, grid, info = self.extract_patch()
        d_front, d_left, d_right = self.compute_obstacle_distances(patch[:,:,0], info)
        min_obs_dist = min(d_front, d_left, d_right)
        tgt, mode = self.choose_target(cp, grid, info)
        if tgt is None:
            self.get_logger().warn("Sin target válido")
            return

        # 3 · ¿Replanificamos? ----------------------------------------
        need_replan = (
            not self.current_path or
            self.collided or
            l2(cp, self.current_path[min(2, len(self.current_path)-1)]) > 0.8
        )
        if need_replan:
            self.current_path = self.generate_flexible_path(cp, tgt, grid, info)
            self.wp_index = 1
            self.get_logger().info(f"[PATH] len={len(self.current_path)} wps")

        # 4 · Seguimiento del path ------------------------------------
        self.follow_path(cp)
        self.publish_path(self.current_path)



        # 5 · Recompensa + buffers PPO --------------------------------
        wp = self.current_path[self.wp_index]
        old_d = l2(cp, tgt)
        new_d = l2(wp, tgt)

        # ——— pendiente del tramo actual ———————————————
        if self.height_map_msg is not None:
            h_arr, hm_info = gridmap_to_numpy(self.height_map_msg)
            if h_arr is not None:
                dz = abs(height_at(h_arr, hm_info, wp) -
                         height_at(h_arr, hm_info, cp))
                slope_curr = dz / max(distance(cp, wp), 1e-6)
            else:
                slope_curr = 0.0
        else:
            slope_curr = 0.0



        reached=self.goal_reached_flag
        collided=self.collided
        # detecta si está girando en sitio:
        # por ejemplo: si throttle casi 0 y steering alto
        is_spinning = (abs(self.last_cmd.throttle) < 1e-2 and 
                    abs(self.last_cmd.steering) > 0.5 * MAX_STEERING)

        reward = self.compute_reward(
            old_d, new_d,
            collided=self.collided,
            reached=self.goal_reached_flag,
            step_len=l2(cp, wp),
            overturned=abs(self._roll_pitch_from_quaternion(self.pose.orientation)[0]) > MAX_TILT or
                    abs(self._roll_pitch_from_quaternion(self.pose.orientation)[1]) > MAX_TILT,
            min_obs_dist=min_obs_dist,
            is_spinning=is_spinning,
            slope_curr=slope_curr
        )

        state_vec = np.concatenate([patch.flatten()[:32],
                                    np.array([0, 0,
                                            tgt[0]-cp[0],
                                            tgt[1]-cp[1]], DTYPE)])


                # ---------------- velocidad experta (la que acabamos de mandar) ----------
        dx, dy = wp[0]-cp[0], wp[1]-cp[1]
        dist   = math.hypot(dx, dy) + 1e-6
        v_lin  = min(MAX_VEL, 1.5*dist)          # misma fórmula que follow_path
        v_norm = np.clip(2*v_lin/MAX_VEL - 1, -1, 1)   # normaliza 0..MAX → -1..+1

        self.vel_buf.append(np.float32(v_norm))         # ← guarda


        # acción ficticia (Δx,Δy de la policy, opcional)
        act_delta = np.zeros(3, DTYPE)
        slope_tgt = slope_curr
        state5 = np.array([
                d_front,
                d_left,
                d_right,
                slope_tgt,
                tgt[0] - cp[0],
                tgt[1] - cp[1]
            ], dtype=DTYPE)
        self.state_buf.append(state5)
        self.patch_buf.append(patch.astype(DTYPE))
        # self.state_buf.append(state_vec)
        self.act_buf.append(act_delta)
        self.logp_buf.append(np.float32(0.0))
        self.rew_buf.append(reward)
        # self.val_buf.append(self.value_net(state_vec[None, ...])[0, 0])
        self.done_buf.append(reached)
        # state5 ya lo tienes definido como np.array([d_front,d_left,d_right, dx, dy])
        patch_emb32 = patch.flatten()[:32]
        critic_input = np.concatenate([patch_emb32, state5])   # tamaño 37
        self.val_buf.append(self.value_net(critic_input[None, ...])[0, 0])

        # 6 · Fin de episodio -----------------------------------------
        if reached:
            self.cmd_pub.publish(CmdThrottleMsg())                  # v=0, ω=0

            if reached:    self.get_logger().info("⛳  Goal alcanzado")
            if collided:   self.get_logger().info("💥  Colisión detectada")
            if overturned: self.get_logger().warning("🚨  Robot volcado")
            self.training = True
            self.update_ppo()                              # entrenamiento
            self.training = False
            self.goal_reached_flag = False
            self.episode  += 1
            self.goal_counter += int(reached and mode == "GOAL")


            with self.writer.as_default():
                tf.summary.scalar("episode_reward",  sum(self.rew_buf),
                                step=self.episode)
                tf.summary.scalar("collided", int(collided), step=self.episode)




        # 7 · Reset de mapa si procede -------------------------------
        if (self.goal_counter >= self.goals_in_world):

            self.reset_pub.publish(Bool(data=True))
            self.ready         = False
            self.waiting_reset = True
            self.reset_t0      = self.get_clock().now()
            self.goal_counter  = 0
            self.goals_in_world = random.randint(5, 7)
            self.get_logger().info("[RESET] petición enviada; esperando confirmación")

        # 8 · Límite global de episodios ------------------------------
        if self.episode >= MAX_EPISODES:
            self.get_logger().info(f"{MAX_EPISODES} episodios completados. "
                                "Finalizando nodo.")
            self.cmd_pub.publish(CmdThrottleMsg())
            rclpy.shutdown()



    # ---------- Publicación RViz ----------
    def publish_path(self,pts):
        hdr=Header(frame_id="map",
                   stamp=self.get_clock().now().to_msg())
        path=Path(header=hdr)
        for x,y in pts:
            ps=PoseStamped(header=hdr)
            ps.pose.position.x=x; ps.pose.position.y=y
            ps.pose.orientation.w=1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

        mk=Marker(header=hdr,ns="wps",id=0,
                  type=Marker.POINTS,action=Marker.ADD)
        mk.scale=Vector3(x=0.15,y=0.15,z=0.0)
        mk.color.r=mk.color.g=1.0; mk.color.a=1.0
        mk.points=[Point(x=x,y=y) for x,y in pts[1:]]
        self.wps_pub.publish(mk)

    def update_ppo(self):
        # 1) Returns y ventajas ---------------------------------------
        returns, advs = [], []
        gae = 0.0
        next_val = 0.0
        for r, v, d in zip(reversed(self.rew_buf),
                        reversed(self.val_buf),
                        reversed(self.done_buf)):
            delta = r + GAMMA * next_val * (1 - d) - v
            gae   = delta + GAMMA * GAE_LAMBDA * (1 - d) * gae
            advs.insert(0, gae)
            next_val = v
        returns = np.array(advs) + np.array(self.val_buf)
        advs    = (np.array(advs) - np.mean(advs)) / (np.std(advs) + 1e-8)

        # 2) DataSet ---------------------------------------------------
        ds = tf.data.Dataset.from_tensor_slices(
            (np.stack(self.patch_buf).astype(np.float32),
            np.stack(self.state_buf).astype(np.float32),
            np.stack(self.act_buf).astype(np.float32),     # (N,2)
            np.array(self.logp_buf, np.float32),
            advs.astype(np.float32),
            returns.astype(np.float32))
        ).shuffle(4096).batch(BATCH_SZ)

        # 3) Optimización ---------------------------------------------
        for _ in range(EPOCHS):
            for g, st, act, lp_old, adv, ret in ds:
                with tf.GradientTape() as tpi, tf.GradientTape() as tpv:
                    w0_dummy = tf.zeros((tf.shape(act)[0], 2), tf.float32)
                    mu  = self.policy([g, st, w0_dummy], training=True)
                    std = tf.exp(self.log_std)              # (2,)

                    lp  = -0.5 * tf.reduce_sum(
                            ((act - mu) / std) ** 2 +
                            2 * tf.math.log(std) +
                            tf.math.log(2 * np.pi), axis=-1)

                    ratio    = tf.exp(lp - lp_old)
                    pg_loss  = -tf.reduce_mean(
                                tf.minimum(ratio * adv,
                                            tf.clip_by_value(ratio,
                                                            1-CLIP_EPS, 1+CLIP_EPS) * adv))

                    # state_vec = tf.concat(
                    #      [tf.reshape(g, (-1, PATCH*PATCH))[:, :32], st], axis=-1)
                    patch_flat = tf.reshape(g, (tf.shape(g)[0], -1))
                    state_vec  = tf.concat([patch_flat[:, :32], st], axis=-1)
                    v      = tf.squeeze(self.value_net(state_vec, training=True), axis=-1)
                    v_loss = tf.reduce_mean((ret - v) ** 2)

                self.opt_actor.apply_gradients(
                    zip(tpi.gradient(pg_loss,
                                    self.policy.trainable_variables + [self.log_std]),
                        self.policy.trainable_variables + [self.log_std]))
                self.opt_critic.apply_gradients(
                    zip(tpv.gradient(v_loss, self.value_net.trainable_variables),
                        self.value_net.trainable_variables))

        # 4) Annealing de σ -------------------------------------------
        new_std = tf.maximum(tf.exp(self.log_std) * STD_DECAY, STD_MIN)
        self.log_std.assign(tf.math.log(new_std))

        # 5) Logs ------------------------------------------------------
        with self.writer.as_default():
            tf.summary.scalar("loss_actor",  pg_loss,      step=self.episode)
            tf.summary.scalar("loss_critic", v_loss,       step=self.episode)
            tf.summary.scalar("policy_std",  float(new_std[0]), step=self.episode)

        # 6) Limpieza y guardado --------------------------------------------------
        
            self.policy.save_weights(RUN_DIR / f"policy_latest_{self.ts}.weights.h5")
            demo_fname = RUN_DIR / f"demo_ep{self.ts}.npz"
            np.savez_compressed(
                demo_fname,
                patches = np.stack(self.patch_buf, 0),
                states  = np.stack(self.state_buf, 0),
                actions = np.stack(self.act_buf,  0),   # Δx Δy que ya tenías
                v_norm  = np.stack(self.vel_buf,  0)    # ← NUEVO
            )
            self.get_logger().info(f"[DEMO] guardada en {demo_fname.name}")


        self.reset_buffers()
        self.get_logger().info(f"[PPO] update  π={pg_loss.numpy():.3f}  "
                            f"V={v_loss.numpy():.3f}  σ={float(new_std[0]):.2f}")

# ==============  MAIN  ====================================================
def main(args=None):
    rclpy.init(args=args)
    node=FlexPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__":
    main()
