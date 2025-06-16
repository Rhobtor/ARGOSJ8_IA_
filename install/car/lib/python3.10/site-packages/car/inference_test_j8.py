#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
inference_test_j8.py  –  Nodo ROS 2 **solo-inferencia** para tu arquitectura
jerárquica (meta-policy + low-policy).

• Carga el *último* checkpoint *.weights.h5 del directorio dado.
• Publica los mismos tópicos de depuración que el trainer:
    /global_path_predicted  (verde)
    /meta_wp                (flecha azul)
    /goal_vec               (flecha roja)
• Reutiliza el mismo controlador de seguimiento   follow_path()
  importándolo desde TerrainPPOTrainer.
"""

# ─────────────────────── imports genéricos ──────────────────────────────
import argparse, pathlib, re, math, time, copy
import numpy as np
import rclpy
from rclpy.node import Node

# ROS-msgs
from std_msgs.msg        import Header, ColorRGBA
from geometry_msgs.msg   import Point, Vector3, PoseStamped, PoseArray
from nav_msgs.msg        import Path, Odometry, OccupancyGrid
from visualization_msgs.msg import Marker
from grid_map_msgs.msg   import GridMap
from argj801_ctl_platform_interfaces.msg import CmdThrottleMsg

# TensorFlow / Keras
import tensorflow as tf
from tensorflow.keras import layers as L

# ─────────────────────── reutilizamos utilidades ────────────────────────
from car.j8_test_ppo import (
    TerrainPPOTrainer,                                 # para follow_path()
    PATCH, BIG_PATCH, MIN_WP_DIST, R_META, CLEAR_MIN,  # const
    WAYPOINT_RADIUS, l2, clearance_ok, densify,        # helpers
    rrt_plan_cost, smooth_bspline, _filter_curv, gridmap_to_numpy,
    slope_ok                                           # follow_path lo usa
)

# ───────────────────────── modelos idénticos ────────────────────────────
def build_low_policy():
    """
    Política local (bajo nivel) que recibe parche 128×128×3 + estado [dx_wp, dy_wp, d_front, d_left].
    Produce 2 valores tanh(-1,1) = (Δ throttle, Δ steering).
    """
    g = tf.keras.Input(shape=(PATCH, PATCH, 3), name="grid")
    st = tf.keras.Input(shape=(4,), name="state")
    x = tf.keras.layers.Conv2D(16, 3, padding="same", activation="relu")(g)
    x = tf.keras.layers.MaxPooling2D()(x)
    x = tf.keras.layers.Conv2D(32, 3, padding="same", activation="relu")(x)
    x = tf.keras.layers.GlobalAveragePooling2D()(x)
    z = tf.keras.layers.Concatenate()([x, st])
    h0 = tf.keras.layers.Dense(128, activation="tanh")(z)
    c0 = tf.keras.layers.Dense(128, activation="tanh")(z)
    lstm = tf.keras.layers.LSTMCell(128)
    w0 = tf.keras.layers.Input(shape=(2,), name="w0")  # estado oculto inicial
    h1, _ = lstm(w0, [h0, c0])
    delta = tf.keras.layers.Dense(2, activation="tanh")(h1)
    return tf.keras.Model([g, st, w0], delta, name="policy")

def build_meta_policy():
    """
    Política de alto nivel que recibe parche 64×64×3 (global) + vector [dx_goal, dy_goal].
    Produce (Δx, Δy) en [-1,1], se escala por R_META.
    """
    # Ahora input shape=(BIG_PATCH,BIG_PATCH,5) (3 occ+1 goal+1 entropy)
    G = tf.keras.Input(shape=(BIG_PATCH, BIG_PATCH, 6), name="big_patch")
    v = tf.keras.Input(shape=(2,), name="vec_goal")
    x = tf.keras.layers.Conv2D(32, 3, activation="relu")(G)
    x = tf.keras.layers.MaxPooling2D()(x)
    x = tf.keras.layers.Conv2D(64, 3, activation="relu")(x)
    x = tf.keras.layers.GlobalAveragePooling2D()(x)
    z = tf.keras.layers.Concatenate()([x, v])
    z = tf.keras.layers.Dense(128, activation="relu")(z)
    out = tf.keras.layers.Dense(2, activation="tanh")(z)  # rango [-1,1]
    return tf.keras.Model([G, v], out, name="meta_policy")

# ───────────────────────── nodo de inferencia ───────────────────────────
class TerrainPPOInference(Node):
    def __init__(self, weights_dir: pathlib.Path):
        super().__init__("terrain_ppo_inference")

        # ─── suscripciones ──────────────────────────────────────────────
        self.create_subscription(Odometry,      "/ARGJ801/odom_demo", self.cb_odom, 10)
        self.create_subscription(PoseArray,     "/goal",              self.cb_goal, 10)
        self.create_subscription(OccupancyGrid, "/occupancy_grid",    self.cb_grid, 10)
        self.create_subscription(GridMap,       "/terrain_grid",      self.cb_hmap, 10)

        # ─── publishers ────────────────────────────────────────────────
        self.path_pub   = self.create_publisher(Path,   "/global_path_predicted", 10)
        self.arrow_pub  = self.create_publisher(Marker, "/meta_wp",               10)
        self.goal_vec_pub = self.create_publisher(Marker, "/goal_vec",            10)
        self.cmd_pub    = self.create_publisher(CmdThrottleMsg, "/ARGJ801/cmd_throttle_msg", 10)

        # ─── redes y pesos ─────────────────────────────────────────────
        self.low  = build_low_policy()
        self.meta = build_meta_policy()
        self._load_latest(weights_dir)

        # ─── estado mínimo para follow_path() ──────────────────────────
        self.pose=None; self.twist=None; self.goal=None
        self.grid=None; self.grid_msg=None; self.height_map_msg=None
        self.grid_dyn=None
        self.hmap=None                                 # alias cómodo

        self.current_path=[]; self.wp_index=0
        self.last_cmd = CmdThrottleMsg()               # para el filtro
        self.last_cmd.throttle = 0.0
        self.last_cmd.steering = 0.0
        self.meta_tick = 0                             # 1.5 s

        # ─── enganchar follow_path() y helpers del trainer ─────────────
        self.follow_path           = TerrainPPOTrainer.follow_path.__get__(self)
        self._yaw_from_quaternion  = TerrainPPOTrainer._yaw_from_quaternion.__get__(self)
        self._global_to_local      = TerrainPPOTrainer._global_to_local.__get__(self)
        # (slope_ok y demás ya son funciones libres)

        # ─── timer 10 Hz ───────────────────────────────────────────────
        self.create_timer(0.1, self.step)
        self.get_logger().info("Nodo de inferencia listo ✓")

    # ──────────────────── carga de checkpoints ─────────────────────────
    def _load_latest(self, wdir: pathlib.Path):
        def _latest(pattern):
            files = sorted(wdir.glob(pattern))
            if not files:
                return None
            # intenta _ep<n>; si no, coge el último por timestamp
            with_ep = [f for f in files if re.search(r'_ep(\d+)', f.stem)]
            return max(with_ep, key=lambda p:int(re.search(r'_ep(\d+)',p.stem).group(1))) \
                   if with_ep else max(files, key=lambda p:p.stat().st_mtime)

        low_ckpt  = _latest("policy_low*.weights.h5")
        meta_ckpt = _latest("policy_meta*.weights.h5")
        if not low_ckpt or not meta_ckpt:
            raise RuntimeError(f"No se encontraron pesos en {wdir}")

        # Keras 3 → sólo `skip_mismatch`
        self.low .load_weights(low_ckpt,  skip_mismatch=True)
        self.meta.load_weights(meta_ckpt, skip_mismatch=True)
        self.get_logger().info(f"Cargados pesos:\n   LOW : {low_ckpt.name}\n   META: {meta_ckpt.name}")

    # ──────────────────── callbacks ROS2 ───────────────────────────────
    def cb_odom(self,msg):
        self.pose  = msg.pose.pose
        self.twist = msg.twist.twist
    def cb_goal(self,msg):
        self.goal = (msg.poses[0].position.x, msg.poses[0].position.y) if msg.poses else None
    def cb_grid(self,msg):
        self.grid_msg = msg
        self.grid_dyn = np.asarray(msg.data,dtype=np.int8)
        self.grid     = self.grid_dyn.reshape((msg.info.height,msg.info.width))
    def cb_hmap(self,msg):
        self.height_map_msg = msg
        self.hmap = msg

    # ──────────────────── patches (idénticos al trainer) ───────────────
    def extract_patch(self):
        if self.pose is None or self.goal is None or self.grid_msg is None or self.grid is None:return
        info = self.grid_msg.info
        cp   = (self.pose.position.x, self.pose.position.y)
        ci   = int((cp[0]-info.origin.position.x)/info.resolution)
        cj   = int((cp[1]-info.origin.position.y)/info.resolution)
        i0,i1 = ci-PATCH//2, ci+PATCH//2
        j0,j1 = cj-PATCH//2, cj+PATCH//2
        occ = self.grid[max(0,j0):min(j1,info.height),
                        max(0,i0):min(i1,info.width)]
        pad = ((max(0,-j0),max(0,j1-info.height)),
               (max(0,-i0),max(0,i1-info.width)))
        occ = np.pad(occ,pad,'constant',constant_values=-1)
        norm_occ = ((occ+1)/101.0).astype(np.float32)

        # altura normalizada
        h_norm = np.zeros_like(norm_occ,dtype=np.float32)
        if self.hmap:
            h_arr,hi = gridmap_to_numpy(self.hmap)
            if h_arr is not None:
                res = hi.resolution
                ox  = hi.pose.position.x - hi.length_x/2
                oy  = hi.pose.position.y - hi.length_y/2
                for jj in range(PATCH):
                    wy = cp[1] + (jj-PATCH//2)*info.resolution
                    jh = int((wy-oy)/res)
                    if 0<=jh<h_arr.shape[0]:
                        for ii in range(PATCH):
                            wx = cp[0] + (ii-PATCH//2)*info.resolution
                            ih = int((wx-ox)/res)
                            if 0<=ih<h_arr.shape[1]:
                                h_norm[jj,ii] = h_arr[jh,ih]
                mu = np.nanmean(h_norm)
                h_norm = (h_norm-mu)/5.0      # ≈ -1..+1

        gx,gy = np.gradient(h_norm)
        return np.stack([norm_occ,gx,gy],axis=-1).astype(np.float32)

    def extract_big_patch(self):
        local = self.extract_patch()
        if local is None: return None
        big = tf.image.resize(local,(BIG_PATCH,BIG_PATCH),method='bilinear').numpy()
        goal_layer = np.zeros((BIG_PATCH,BIG_PATCH),np.float32)
        if self.goal:
            scale = BIG_PATCH/PATCH
            dx = (self.goal[0]-self.pose.position.x)/self.grid_msg.info.resolution
            dy = (self.goal[1]-self.pose.position.y)/self.grid_msg.info.resolution
            gi = int(BIG_PATCH/2 + dx*scale)
            gj = int(BIG_PATCH/2 + dy*scale)
            goal_layer[max(0,gj-2):gj+3, max(0,gi-2):gi+3] = 5.0
        ent = np.zeros_like(goal_layer)
        frs = np.zeros_like(goal_layer)
        return np.dstack([big,goal_layer,ent,frs]).astype(np.float32)

    # ──────────────────── path local RRT* coste ─────────────────────────
    def short_path(self,start,wp):
        arr,info = self.grid, self.grid_msg.info
        h,hinfo  = gridmap_to_numpy(self.hmap)
        path = rrt_plan_cost(start,wp,arr,info,h,hinfo,max_iter=300,step=0.5,goal_tol=0.3)
        if len(path)<2: path=[start,wp]
        return densify(smooth_bspline(_filter_curv(path)))

    # ──────────────────── helpers de visualización ─────────────────────
    def arrow(self,p_from,p_to,color,ns,id=0):
        hdr = Header(frame_id="map",stamp=self.get_clock().now().to_msg())
        m   = Marker(header=hdr,ns=ns,id=id,type=Marker.ARROW,action=Marker.ADD)
        m.scale = Vector3(x=0.05,y=0.1,z=0.1)
        m.color = ColorRGBA(r=float(color[0]),g=float(color[1]),b=float(color[2]),a=1.0)
        m.points=[Point(x=p_from[0],y=p_from[1],z=0.2),
                  Point(x=p_to  [0],y=p_to  [1],z=0.2)]
        (self.goal_vec_pub if ns=="goal_vec" else self.arrow_pub).publish(m)

    def publish_path(self,pts):
        hdr=Header(frame_id="map",stamp=self.get_clock().now().to_msg())
        msg=Path(header=hdr)
        for x,y in pts:
            ps=PoseStamped(header=hdr); ps.pose.position.x=x; ps.pose.position.y=y; ps.pose.orientation.w=1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    # ──────────────────── bucle de control (10 Hz) ──────────────────────
    def step(self):
        # espera a tener todos los datos
        if self.pose is None or self.goal is None or self.grid_msg is None or self.grid is None:return
        cp=(self.pose.position.x,self.pose.position.y)

        # ─── META cada 1.5 s ───────────────────────────────────────────
        if self.meta_tick==0 or not self.current_path:
            big = self.extract_big_patch()
            v   = np.array([(self.goal[0]-cp[0])/R_META,
                            (self.goal[1]-cp[1])/R_META],np.float32)
            mu      = self.meta([big[None,...],v[None,...]])[0].numpy()
            wp_rel  = np.clip(mu,-1,1)*R_META
            if np.linalg.norm(wp_rel)<MIN_WP_DIST:
                wp_rel = wp_rel/np.linalg.norm(wp_rel)*MIN_WP_DIST
            waypoint = (cp[0]+wp_rel[0],cp[1]+wp_rel[1])

            # seguridad mínima
            if not clearance_ok(self.grid,self.grid_msg.info,waypoint,CLEAR_MIN):
                waypoint = cp   # sin movimiento si cae en obstáculo

            # debug flechas
            self.arrow(cp,self.goal,(1,0,0),"goal_vec")
            self.arrow(cp,waypoint,(0,0,1),"meta_wp")

            # plan local
            self.current_path = self.short_path(cp,waypoint)
            self.publish_path(self.current_path)
            self.wp_index = 1
            self.meta_tick = 15           # 1.5 s
        else:
            self.meta_tick -= 1

        # ─── llamada al mismo Pure-Pursuit del trainer ─────────────────
        if self.current_path:
            # alias para compatibilidad con follow_path()
            self.height_map_msg = self.hmap
            self.grid_dyn       = self.grid
            self.follow_path(cp)           # ← del trainer

# ─────────────────────────────  main  ───────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights_dir", required=True,
                        help="Directorio con *.weights.h5")
    args = parser.parse_args()
    rclpy.init()
    node = TerrainPPOInference(pathlib.Path(args.weights_dir).expanduser())
    rclpy.spin(node)

if __name__ == "__main__":
    main()
