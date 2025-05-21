#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 5 – Nodo de detección de línea
# Materia: Implementación de Robótica Inteligente
# ------------------------------------------------------------------------------
import rclpy, cv2, numpy as np
from rclpy.node        import Node
from cv_bridge         import CvBridge
from sensor_msgs.msg   import Image
from std_msgs.msg      import Float32

class LineFollower(Node):
    def __init__(self):
        super().__init__('LineFollower')
        self.bridge, self.image_ok = CvBridge(), False

        # ───────── Parámetros ROS tunables ─────────
        self.declare_parameter('mode',          'real')
        self.declare_parameter('use_hsv',        False)
        self.declare_parameter('gray_thresh',     95)
        self.declare_parameter('lower_h',          0)
        self.declare_parameter('upper_h',        180)
        self.declare_parameter('upper_s',        255)
        self.declare_parameter('upper_v',        140)

        # Filtros de forma
        self.declare_parameter('min_area',         30)
        self.declare_parameter('min_ratio',       1.00)
        self.declare_parameter('max_width_frac',  0.60)

        # NUEVO: proporción desde la que empieza el ROI (0.6)
        self.declare_parameter('roi_ratio',       0.6)
        # -------------------------------------------

        mode      = self.get_parameter('mode').value
        topic_cam = 'video_source/raw' if mode == 'real' else 'camera'

        self.sub = self.create_subscription(Image, topic_cam, self.cb_img, 10)
        self.pub_error = self.create_publisher(Float32, '/line_follower_data', 10)
        self.pub_img   = self.create_publisher(Image,  '/processed_line_image', 10)
        self.pub_mask  = self.create_publisher(Image,  '/line_mask', 10)

        self.create_timer(0.1, self.cb_timer)
        self.get_logger().info(f'📸 LineFollower ON (ROI desde {self.get_parameter("roi_ratio").value*100:.0f} %)')

    # ---------- Callbacks ----------
    def cb_img(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_ok = True
        except Exception as e:
            self.get_logger().warning(f'⚠️ Img error: {e}')

    def cb_timer(self):
        if self.image_ok:
            self.process()

    # ---------- Procesado principal ----------
    def process(self):
        h, w = self.cv_img.shape[:2]
        roi_ratio = self.get_parameter('roi_ratio').value  # 0-1
        roi_y0    = int(roi_ratio * h)
        roi       = self.cv_img[roi_y0:, :]

        # 1) Máscara (gris u HSV)
        if not self.get_parameter('use_hsv').value:
            thr  = self.get_parameter('gray_thresh').value
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, thr, 255, cv2.THRESH_BINARY_INV)
        else:
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lo  = np.array([self.get_parameter('lower_h').value, 0, 0])
            hi  = np.array([self.get_parameter('upper_h').value,
                            self.get_parameter('upper_s').value,
                            self.get_parameter('upper_v').value])
            mask = cv2.inRange(hsv, lo, hi)

        # 2) Morfología
        k3 = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k3, 1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k3, 1)
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, 'mono8'))

        # 3) Contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        min_ar   = self.get_parameter('min_area').value
        min_rat  = self.get_parameter('min_ratio').value
        max_w_fr = self.get_parameter('max_width_frac').value

        def is_line(cnt):
            area = cv2.contourArea(cnt)
            if area < min_ar: return False
            x, y, ww, hh = cv2.boundingRect(cnt)
            if ww > max_w_fr * w: return False
            return max(ww, hh) / max(1, min(ww, hh)) >= min_rat

        contours = [c for c in contours if is_line(c)]
        contours.sort(key=cv2.contourArea, reverse=True)
        contours = contours[:3]

        # 4) Dibujos y centroides
        out = self.cv_img.copy()
        centers = []
        for c in contours:
            M = cv2.moments(c)
            if M['m00'] == 0: continue
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00']) + roi_y0  # → offset
            centers.append((cx, cy))
            cv2.drawContours(out, [c + np.array([0, roi_y0])], -1, (0,255,0), 2)

        centers.sort(key=lambda p: p[0])   # ordenar por X
        img_cx = w // 2

        if len(centers) == 1:
            cx, cy = centers[0]

        elif len(centers) == 2:
            left, right = centers            # ya ordenados

            if right[0] < img_cx:            # ambos a la IZQUIERDA
                cx, cy = left                # toma el más a la izquierda
            elif left[0] > img_cx:           # ambos a la DERECHA
                cx, cy = right               # toma el más a la derecha
            else:                            # uno a cada lado
                cx, cy = min(centers, key=lambda p: abs(p[0] - img_cx))

        else:  # 3 o más contornos
            cx, cy = min(centers, key=lambda p: abs(p[0] - img_cx))

        # 5) Error y publicación
        error = float(cx - img_cx)
        self.pub_error.publish(Float32(data=error))

        cv2.line(out,(img_cx,0),(img_cx,h),(0,255,255),1)
        cv2.circle(out,(cx,cy),4,(0,0,255), -1)
        cv2.line(out, (cx, cy), (img_cx, cy), (0, 255, 255), 1)  # Línea amarilla (error)
        cv2.putText(out,f'Error:{error:.1f}',(10,10), cv2.FONT_HERSHEY_SIMPLEX,0.3,(0,255,0),1)
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(out,'bgr8'))

# ---------------- main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
