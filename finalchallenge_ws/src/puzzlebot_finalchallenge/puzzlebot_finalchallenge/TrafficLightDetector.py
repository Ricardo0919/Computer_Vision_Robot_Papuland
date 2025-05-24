#!/usr/bin/env python3
# ---------------------------------------------------------------
# TrafficLightDetector · filtros max_pixels, max_area y roi_right
# ---------------------------------------------------------------
import rclpy, cv2, numpy as np
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from cv_bridge  import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg    import String

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        # ---------- parámetros ----------
        self.declare_parameter('mode',        'real')   # sim | real
        self.declare_parameter('max_pixels',    50)     # máx. píxeles blancos
        self.declare_parameter('max_area',     200)     # máx. área de contorno
        self.declare_parameter('roi_right',    130)     # columna inicial del ROI derecho (1-160)

        p            = self.get_parameter
        mode         = p('mode').value
        self.max_px  = p('max_pixels').value
        self.max_ar  = p('max_area').value
        self.roi_r   = int(p('roi_right').value)

        # El estado previo empieza en "none", pero NO se publicará nunca ese valor
        self.prev_state = 'none'

        # ---------- actualización dinámica ----------
        self.add_on_set_parameters_callback(self.parameter_callback)

        # ---------- QoS / tópicos ----------
        cam_topic = 'camera' if mode == 'sim' else 'video_source/raw'
        qos_color = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            durability  = DurabilityPolicy.VOLATILE,
            depth       = 10)

        self.sub_img    = self.create_subscription(Image, cam_topic, self.cb_img, 10)
        self.pub_img    = self.create_publisher(Image, 'processed_img', 10)
        self.pub_col    = self.create_publisher(String, 'color_detector', qos_color)
        self.pub_mask_r = self.create_publisher(Image, 'mask_r', 10)
        self.pub_mask_y = self.create_publisher(Image, 'mask_y', 10)
        self.pub_mask_g = self.create_publisher(Image, 'mask_g', 10)

        self.bridge, self.frame, self.new_img = CvBridge(), None, False
        self.create_timer(0.1, self.timer_cb)        # 10 Hz

        # Rangos HSV
        self.hsv_ranges = {
            "Rojo": [
                {"lower": np.array([  0,  80, 120]), "upper": np.array([ 10, 255, 255])},
                {"lower": np.array([170,  80, 120]), "upper": np.array([179, 255, 255])}
            ],
            "Amarillo": [
                {"lower": np.array([ 15,  35, 110]), "upper": np.array([ 35, 255, 255])},
                {"lower": np.array([ 13,  32,   0]), "upper": np.array([ 70, 255, 255])}
            ],
            "Verde": [
                {"lower": np.array([ 40,  70, 140]), "upper": np.array([ 90, 255, 255])}
            ]
        }

        self.get_logger().info('🚦 TrafficLightDetector iniciado.')

    # ---------- callback de parámetros dinámicos ----------
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_pixels':
                self.max_px = param.value
                self.get_logger().info(f'🟠 max_pixels → {self.max_px}')
            elif param.name == 'max_area':
                self.max_ar = param.value
                self.get_logger().info(f'🟡 max_area → {self.max_ar}')
            elif param.name == 'roi_right':
                self.roi_r = param.value
                self.get_logger().info(f'🔵 roi_right → {self.roi_r}')
        return SetParametersResult(successful=True)

    # ---------- callbacks de imagen y temporizador ----------
    def cb_img(self, msg):
        try:
            self.frame  = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.new_img = True
        except Exception as e:
            self.get_logger().warn(f'Conversión falló: {e}')

    def timer_cb(self):
        if not self.new_img:
            return
        self.new_img = False

        vis, state, masks = self.detect(self.frame.copy())

        self.pub_img.publish(self.bridge.cv2_to_imgmsg(vis, 'bgr8'))
        self.pub_mask_r.publish(self.bridge.cv2_to_imgmsg(masks['Rojo'],     'mono8'))
        self.pub_mask_y.publish(self.bridge.cv2_to_imgmsg(masks['Amarillo'], 'mono8'))
        self.pub_mask_g.publish(self.bridge.cv2_to_imgmsg(masks['Verde'],    'mono8'))

        # --- PUBLICAR SOLO SI state ≠ "" y cambió respecto a prev_state ---
        if state and state != self.prev_state:
            self.prev_state = state
            self.pub_col.publish(String(data=state))
            self.get_logger().info(f'ESTADO: {state.upper()}')
        # Si state == "" no se publica nada y prev_state sigue intacto

    # ---------- núcleo de detección ----------
    def detect(self, img):
        img_r = cv2.resize(img, (160, 120))

        # Línea que marca el inicio del ROI derecho
        cv2.line(img_r, (self.roi_r, 0), (self.roi_r, img_r.shape[0] - 1), (255, 0, 0), 1)
        img_r = img_r[:, self.roi_r:]  # conservar solo la derecha

        hsv  = cv2.cvtColor(img_r, cv2.COLOR_BGR2HSV)
        vis  = img_r.copy()
        k    = np.ones((3, 3), np.uint8)
        masks, good = {}, {}

        for color, ranges in self.hsv_ranges.items():
            m = None
            for r in ranges:
                part = cv2.inRange(hsv, r['lower'], r['upper'])
                m = part if m is None else cv2.bitwise_or(m, part)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  k)
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
            masks[color] = m

            pix  = cv2.countNonZero(m)
            cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            area = max(cv2.contourArea(c) for c in cnts) if cnts else 0

            if 0 < pix <= self.max_px and area <= self.max_ar:
                good[color] = (pix, area, cnts)

        # -------------------- resultado --------------------
        if not good:
            return vis, "", masks           # ← cadena vacía = no publicar
        chosen = max(good, key=lambda c: good[c][0])
        state  = {'Rojo':'stop', 'Amarillo':'slow', 'Verde':'continue'}[chosen]

        # Dibujar anotaciones
        cv2.putText(vis, chosen, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cnts = good[chosen][2]
        if cnts:
            x, y, w, h = cv2.boundingRect(max(cnts, key=cv2.contourArea))
            cv2.rectangle(vis, (x, y), (x+w, y+h), (0, 255, 0), 2)

        return vis, state, masks


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
