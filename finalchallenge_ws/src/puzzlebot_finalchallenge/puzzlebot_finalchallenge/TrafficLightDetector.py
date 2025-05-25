#!/usr/bin/env python3
# ---------------------------------------------------------------
# TrafficLightDetector · processed_img único + HSV por cada lado
# ---------------------------------------------------------------
import rclpy, cv2, numpy as np
from rclpy.node  import Node
from rclpy.qos   import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from cv_bridge   import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg    import String
import copy

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        # ---------- parámetros ----------
        self.declare_parameter('mode',        'real')   # sim | real
        self.declare_parameter('max_pixels',    50)     # máx. píxeles blancos
        self.declare_parameter('max_area',     200)     # máx. área de contorno
        self.declare_parameter('roi_left',      30)     # última col. ROI-izq  (0-159)
        self.declare_parameter('roi_right',    130)     # primera col. ROI-der (1-160)

        p             = self.get_parameter
        mode          = p('mode').value
        self.max_px   = p('max_pixels').value
        self.max_ar   = p('max_area').value
        self.roi_l    = int(p('roi_left').value)
        self.roi_r    = int(p('roi_right').value)

        self.prev_state = 'none'          # no se publica “none”

        # ---------- actualización dinámica ----------
        self.add_on_set_parameters_callback(self.parameter_callback)

        # ---------- QoS / tópicos ----------
        cam_topic = 'camera' if mode == 'sim' else 'video_source/raw'
        qos_color = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            durability  = DurabilityPolicy.VOLATILE,
            depth       = 10)

        self.sub_img  = self.create_subscription(Image, cam_topic, self.cb_img, 10)
        self.pub_img  = self.create_publisher(Image, 'processed_img', 10)

        pub = self.create_publisher
        self.pub_mask_r_l = pub(Image, 'mask_r_l', 10)
        self.pub_mask_r_r = pub(Image, 'mask_r_r', 10)
        self.pub_mask_y_l = pub(Image, 'mask_y_l', 10)
        self.pub_mask_y_r = pub(Image, 'mask_y_r', 10)
        self.pub_mask_g_l = pub(Image, 'mask_g_l', 10)
        self.pub_mask_g_r = pub(Image, 'mask_g_r', 10)

        self.pub_col  = pub(String, 'color_detector', qos_color)

        self.bridge, self.frame, self.new_img = CvBridge(), None, False
        self.create_timer(0.1, self.timer_cb)          # 10 Hz

        # ---------- rangos HSV (derecha e izquierda) ----------
        self.hsv_ranges_right = {
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
        # Copia profunda para poder ajustar un lado sin afectar el otro
        self.hsv_ranges_left = {
            "Rojo": [
                {"lower": np.array([  150,  15, 105]), "upper": np.array([ 175, 255, 255])},
            ],
            "Amarillo": [
                {"lower": np.array([ 15,  35, 110]), "upper": np.array([ 35, 255, 255])},
            ],
            "Verde": [
                {"lower": np.array([ 40,  43, 000]), "upper": np.array([ 90, 255, 255])}
            ]
        }

        self.get_logger().info('🚦 TrafficLightDetector – HSV por lado iniciado.')

    # ---------- callback de parámetros dinámicos ----------
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_pixels':
                self.max_px = param.value
            elif param.name == 'max_area':
                self.max_ar = param.value
            elif param.name == 'roi_left':
                self.roi_l = int(param.value)
            elif param.name == 'roi_right':
                self.roi_r = int(param.value)
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

        vis = cv2.resize(self.frame.copy(), (160, 120))

        # Líneas divisorias visuales
        cv2.line(vis, (self.roi_l, 0), (self.roi_l, 119), (0, 255,   0), 1)  # verde (izq)
        cv2.line(vis, (self.roi_r, 0), (self.roi_r, 119), (255,   0,  0), 1)  # azul  (der)

        # Procesar ROI-izq  [0 : roi_l]  con rangos *left*
        state_l, masks_l = self.detect_side(vis, 0, self.roi_l,
                                            self.hsv_ranges_left)
        # Procesar ROI-der  [roi_r : 160] con rangos *right*
        state_r, masks_r = self.detect_side(vis, self.roi_r, 160,
                                            self.hsv_ranges_right,
                                            x_offset=self.roi_r)

        # Publicar imagen única
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(vis, 'bgr8'))

        # Publicar máscaras
        br = self.bridge
        self.pub_mask_r_l.publish(br.cv2_to_imgmsg(masks_l['Rojo'],     'mono8'))
        self.pub_mask_r_r.publish(br.cv2_to_imgmsg(masks_r['Rojo'],     'mono8'))
        self.pub_mask_y_l.publish(br.cv2_to_imgmsg(masks_l['Amarillo'], 'mono8'))
        self.pub_mask_y_r.publish(br.cv2_to_imgmsg(masks_r['Amarillo'], 'mono8'))
        self.pub_mask_g_l.publish(br.cv2_to_imgmsg(masks_l['Verde'],    'mono8'))
        self.pub_mask_g_r.publish(br.cv2_to_imgmsg(masks_r['Verde'],    'mono8'))

        # Estado final (prioridad: izquierda, luego derecha)
        state = state_l or state_r
        if state and state != self.prev_state:
            self.prev_state = state
            self.pub_col.publish(String(data=state))
            self.get_logger().info(f'ESTADO: {state.upper()}')

    # ---------- detección en un ROI ----------
    def detect_side(self, vis, x0, x1, hsv_ranges, *, x_offset=0):
        """
        vis        : imagen 160×120 donde se dibujan las anotaciones
        x0, x1     : columnas [inicio, fin) del ROI
        hsv_ranges : diccionario de rangos HSV a usar para este lado
        x_offset   : corrige bounding-box si el ROI no inicia en 0
        """
        roi   = vis[:, x0:x1]
        hsv   = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        k     = np.ones((3, 3), np.uint8)
        masks, good = {}, {}

        for color, ranges in hsv_ranges.items():
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

        if not good:
            return "", masks

        chosen = max(good, key=lambda c: good[c][0])
        state  = {'Rojo':'stop', 'Amarillo':'slow', 'Verde':'continue'}[chosen]

        # Anotaciones
        cv2.putText(vis, chosen,
                    (x0 + 5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        cnts = good[chosen][2]
        if cnts:
            x, y, w, h = cv2.boundingRect(max(cnts, key=cv2.contourArea))
            cv2.rectangle(vis,
                          (x_offset + x, y),
                          (x_offset + x + w, y + h),
                          (0, 255, 0), 2)

        return state, masks

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
