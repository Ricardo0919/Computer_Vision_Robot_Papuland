#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - PathFollower (auto-reset odom)
# Materia: Implementación de Robótica Inteligente
# Fecha: 14 de junio de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

import rclpy, os, yaml, numpy as np
from rclpy.node import Node
from rclpy.qos  import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Empty

DEG = lambda r: np.degrees(r)

class PathFollower(Node):
    def __init__(self):
        super().__init__('PathFollower')

        # --- Cargar archivo paths.yaml con rutas predefinidas ---
        self.declare_parameter('path_file', '')
        pkg = get_package_share_directory('puzzlebot_finalchallenge')
        default_file = os.path.join(pkg,'config','paths.yaml')
        path_file = self.get_parameter('path_file').value or default_file
        with open(path_file) as f:
            self.paths = yaml.safe_load(f).get('paths',{})
        if not self.paths:
            self.get_logger().error("❌ paths.yaml vacío"); return

        # --- Subscripciones y publicadores ---
        self.create_subscription(Odometry,'/odom',self.odom_cb,qos_profile_sensor_data)
        self.create_subscription(String,'/go_direction',self.dir_cb,10)
        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.done_pub = self.create_publisher(String, '/path_done', 10)

        # --- Cliente para resetear la odometría ---
        self.reset_cli = self.create_client(Empty,'reset_odometry')
        while not self.reset_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Esperando /reset_odometry…')

        # --- Estado interno del robot ---
        self.x = self.y = self.theta = 0.0
        self.active_path = []  # Lista de puntos (x, y)
        self.idx = 0           # Punto actual a alcanzar
        self.goal_th = 0.08    # Umbral de distancia para cambio de punto
        self.done = True       # Indica si ya terminó la ruta

        # --- PID para control lineal y angular ---
        self.kp_lin=1.0; self.kd_lin=0.1
        self.kp_ang=4.0; self.kd_ang=0.3
        self.prev_lin=self.prev_ang=0.0
        self.prev_t=self.get_clock().now()

        self.create_timer(0.1,self.control)
        self.get_logger().info("🧭 PathFollower listo.")

    # ---------- Callback para recibir la dirección a seguir ----------
    def dir_cb(self,msg):
        if not self.done:   # ignora si aún va en una ruta
            return
        d=msg.data.strip().lower()
        if d not in self.paths:
            self.get_logger().warn(f"❗ Dirección no válida: {d}"); return

        # Solicita reinicio de odometría antes de comenzar
        self.reset_cli.call_async(Empty.Request())

        # Carga la ruta correspondiente y reinicia el seguimiento
        self.active_path = self.paths[d]
        self.idx=0; self.done=False
        self.get_logger().info(f"📍 Ruta '{d}' con {len(self.active_path)} puntos.")

    # ---------- Callback para actualizar la posición del robot ----------
    def odom_cb(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _,_,self.theta = self.quat2euler(q.x,q.y,q.z,q.w)

    # ---------- Bucle principal de control ----------
    def control(self):
        if self.done or not self.active_path: 
            return
        
        # Obtener punto objetivo actual
        gx,gy = self.active_path[self.idx].values()
        dx,dy = gx-self.x, gy-self.y
        dist  = np.hypot(dx,dy)
        tgt   = np.arctan2(dy,dx)
        err   = self.norm_ang(tgt-self.theta)

        # Calcular PID angular
        now=self.get_clock().now()
        dt=(now-self.prev_t).nanoseconds*1e-9; 
        self.prev_t=now

        d_ang=(err-self.prev_ang)/dt if dt else 0; 
        self.prev_ang=err
        w=np.clip(self.kp_ang*err + self.kd_ang*d_ang,-1.5,1.5)

        # Calcular PID lineal solo si orientado correctamente
        if abs(err) < np.deg2rad(15):
            lin_er=dist
            d_lin=(lin_er-self.prev_lin)/dt if dt else 0; self.prev_lin=lin_er
            v=np.clip(self.kp_lin*lin_er + self.kd_lin*d_lin,-0.15,0.15)
        else:
            v=0.0

        # Publicar velocidades de control
        self.cmd_pub.publish(Twist(linear=Twist().linear.__class__(x=v),
                                   angular=Twist().angular.__class__(z=w)))

        # Verificar si se alcanzó el punto objetivo
        if dist < self.goal_th:
            self.idx += 1
            if self.idx >= len(self.active_path):
                self.done=True
                self.cmd_pub.publish(Twist()) # Detener robot
                #Termino de la ruta, publicar mensaje
                msg = String()
                msg.data = "done"
                self.done_pub.publish(msg)

    # ---------- utilidades ----------
    def norm_ang(self,a): return np.arctan2(np.sin(a),np.cos(a))
    def quat2euler(self,x,y,z,w):
        t0=+2*(w*x+y*z); t1=+1-2*(x*x+y*y); roll=np.arctan2(t0,t1)
        t2=+2*(w*y-z*x); t2=np.clip(t2,-1,1); pitch=np.arcsin(t2)
        t3=+2*(w*z+x*y); t4=+1-2*(y*y+z*z); yaw=np.arctan2(t3,t4)
        return roll,pitch,yaw

# ---------------- main ----------------
def main(args=None):
    rclpy.init(args=args)
    node=PathFollower()
    rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()