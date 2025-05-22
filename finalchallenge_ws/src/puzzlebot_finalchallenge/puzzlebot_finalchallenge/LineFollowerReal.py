#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Histogram Line Follower – con filtros antifalso-positivo
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
        self.prev_cx = None

        # ───────── Parámetros tunables ─────────
        self.declare_parameter('mode',         'real')
        self.declare_parameter('gray_thresh',     95)
        self.declare_parameter('roi_ratio',     0.75)  # 80 % inferior
        self.declare_parameter('min_pixels',     600)  # pico mínimo
        self.declare_parameter('smooth_cols',      5)
        # ─ filtros anti-salto
        self.declare_parameter('win_px',        120)   # anchura de ventana de búsqueda
        self.declare_parameter('jump_px',        80)   # salto máximo permitido
        self.declare_parameter('min_run',        15)   # alto mínimo de la “barra” negra
        # ---------------------------------------

        topic_cam = 'video_source/raw' if self.get_parameter('mode').value == 'real' else 'camera'
        self.sub = self.create_subscription(Image, topic_cam, self.cb_img, 10)
        self.pub_err  = self.create_publisher(Float32, '/line_follower_data', 10)
        self.pub_img  = self.create_publisher(Image,  '/processed_line_image', 10)
        self.pub_mask = self.create_publisher(Image,  '/line_mask', 10)

        self.create_timer(0.10, self.cb_timer)
        self.get_logger().info('📸 Histogram LineFollower (robust) ON')

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
        img  = self.cv_img
        h, w = img.shape[:2]
        img_cx = w // 2

        # 1) ROI
        roi_y0 = int(self.get_parameter('roi_ratio').value * h)
        roi    = img[roi_y0:, :]

        # 2) Máscara
        thr  = self.get_parameter('gray_thresh').value
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, thr, 255, cv2.THRESH_BINARY_INV)
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, 'mono8'))

        # 3) Histograma vertical (suavizado)
        col_sum = mask.sum(axis=0).astype(np.float32)
        k = int(self.get_parameter('smooth_cols').value)
        if k > 1:
            col_sum = np.convolve(col_sum, np.ones(k)/k, mode='same')

        # 4) Ventana de búsqueda alrededor de prev_cx
        win_px   = self.get_parameter('win_px').value
        if self.prev_cx is not None:
            x_min = max(0, self.prev_cx - win_px)
            x_max = min(w-1, self.prev_cx + win_px)
        else:
            x_min, x_max = 0, w-1

        # 5) Seleccionar columna candidata con condición min_run
        min_run = self.get_parameter('min_run').value
        best_cx, best_val = None, 0
        for x in range(x_min, x_max+1):
            if col_sum[x] > best_val:
                # alto vertical de la barra
                run = np.count_nonzero(mask[:, x])
                if run >= min_run:
                    best_val, best_cx = col_sum[x], x

        # ¿hay pista?
        if best_cx is None or best_val < self.get_parameter('min_pixels').value:
            self.pub_err.publish(Float32(data=float('nan')))
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))
            return

        cx = best_cx

        # 6) Rechazo de salto brusco
        jump_px = self.get_parameter('jump_px').value
        if self.prev_cx is not None and abs(cx - self.prev_cx) > jump_px:
            # si el pico antiguo sigue “fuerte”, mantenemos prev_cx
            if col_sum[self.prev_cx] > 0.5 * best_val:
                cx = self.prev_cx

        self.prev_cx = cx  # actualizar memoria

        # 7) Obtener cy para dibujar
        rows = np.where(mask[:, cx] > 0)[0]
        cy = roi_y0 + (int(np.median(rows)) if len(rows) else mask.shape[0]//2)

        # 8) Dibujos
        out = img.copy()
        cv2.line(out, (img_cx, 0), (img_cx, h), (0, 255, 255), 1)
        cv2.circle(out, (cx, cy), 4, (0, 0, 255), -1)
        cv2.line(out, (cx, cy), (img_cx, cy), (0, 255, 255), 1)

        # 9) Publicar error
        error = float(cx - img_cx)
        self.pub_err.publish(Float32(data=error))
        cv2.putText(out, f'Err:{error:.1f}', (10, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        self.pub_img.publish(self.bridge.cv2_to_imgmsg(out, 'bgr8'))

# ---------------- main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
