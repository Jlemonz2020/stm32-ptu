# [MaixCAM] 二维云台 - 目标追踪
# 功能：检测黑色边框 → 返回中心坐标 → 串口输出
# 适用：黑色边框 + 白色内部的矩形目标
# 兼容：MaixCAM Pro + MaixVision IDE (MaixPy v4)

from maix import camera, display, app, time, uart, image

# === 参数配置 ===
IMG_WIDTH = 240
IMG_HEIGHT = 240

UART_PORT = "/dev/ttyS0"
UART_BAUD = 115200
UART_LINE_ENDING = "\n"
ENABLE_CONSOLE_LOG = True  # 改为True方便调试

# 检测黑色边框的阈值范围（二值化后矩形框是白色的）
BLACK_THRESHOLD = (0, 35)  # 检测黑色边框(灰度值越小越接近黑色)
MIN_AREA = 100             # 最小有效面积
MAX_AREA = 15000           # 最大面积限制

# 误识别抑制：按尺寸/长宽比/填充率过滤，并优先选"更大且更靠近画面中心"的候选
MIN_W = 25
MIN_H = 10
MAX_W_RATIO = 0.85
MAX_H_RATIO = 0.85
MIN_ASPECT = 0.3
MAX_ASPECT = 3.5
MIN_DENSITY = 0.01
MAX_DENSITY = 0.45
RELAX_MAX_DENSITY = 0.95
CENTER_WEIGHT = 0.4
TRACK_WEIGHT = 1.2
EDGE_MARGIN = 0

def init_uart():
    """初始化串口通信"""
    try:
        return uart.UART(UART_PORT, UART_BAUD)
    except Exception as e:
        print(f"UART failed: {e}")
        return None

def find_white_frame_center(gray_img, black_binary_img, last_cx=0, last_cy=0):
    """检测黑色边框(二值化为白色)并返回框的中心坐标"""
    blobs = black_binary_img.find_blobs([(128, 255)],
                                        pixels_threshold=MIN_AREA,
                                        area_threshold=MIN_AREA,
                                        merge=True)
    if not blobs:
        return 0, 0, 0, 0

    img_w = gray_img.width()
    img_h = gray_img.height()
    img_cx = img_w / 2
    img_cy = img_h / 2

    best = None
    best_score = None
    relaxed = None
    relaxed_score = None
    fallback = None
    fallback_score = None
    
    for b in blobs:
        if b.area() > MAX_AREA:
            continue
        try:
            _, _, w, h = b.rect()

            if w < MIN_W or h < MIN_H:
                continue
            if w > gray_img.width() * MAX_W_RATIO or h > gray_img.height() * MAX_H_RATIO:
                continue

            bc_x = b.cx()
            bc_y = b.cy()

            dx0 = bc_x - img_cx
            dy0 = bc_y - img_cy
            score = b.area() - CENTER_WEIGHT * (dx0 * dx0 + dy0 * dy0)

            if last_cx or last_cy:
                dx1 = bc_x - last_cx
                dy1 = bc_y - last_cy
                score = score - TRACK_WEIGHT * (dx1 * dx1 + dy1 * dy1)

            if fallback is None or score > fallback_score:
                fallback = b
                fallback_score = score

            aspect = w / h
            if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
                continue

            density = b.area() / (w * h)
            if density < MIN_DENSITY or density > RELAX_MAX_DENSITY:
                continue

            if density <= MAX_DENSITY:
                if EDGE_MARGIN:
                    if bc_x < EDGE_MARGIN or bc_x > img_w - EDGE_MARGIN:
                        raise Exception("edge")
                    if bc_y < EDGE_MARGIN or bc_y > img_h - EDGE_MARGIN:
                        raise Exception("edge")

                if best is None or score > best_score:
                    best = b
                    best_score = score

            if relaxed is None or score > relaxed_score:
                relaxed = b
                relaxed_score = score
        except Exception:
            continue

    if best is not None:
        return best.cx(), best.cy(), len(blobs), 1

    if relaxed is not None:
        return relaxed.cx(), relaxed.cy(), len(blobs), 3

    if fallback is not None:
        return fallback.cx(), fallback.cy(), len(blobs), 4

    return 0, 0, len(blobs), 0

def send_coord(serial, cx, cy):
    """通过串口发送坐标数据"""
    if not serial:
        return None, 0
    try:
        serial.write(f"{cx},{cy}{UART_LINE_ENDING}".encode())
        return serial, 1
    except Exception as e:
        print(f"UART write failed: {e}")
        return None, 0

if __name__ == "__main__":
    print("=" * 50)
    print("Starting MaixCAM Gimbal Tracker...")
    print("=" * 50)

    # 初始化硬件
    cam = camera.Camera(IMG_WIDTH, IMG_HEIGHT)
    disp = display.Display()
    serial = init_uart()

    print(f"Camera: {IMG_WIDTH}x{IMG_HEIGHT}")
    print(f"UART: {UART_PORT} @ {UART_BAUD} baud")
    print("System ready. Press Ctrl+C to stop.")
    print("=" * 50)

    last_cx = 0
    last_cy = 0
    tx_count = 0
    last_tx_ok = 0

    while not app.need_exit():
        # 获取图像并转换为灰度
        img = cam.read()
        gray = img.to_format(image.Format.FMT_GRAYSCALE)

        # 二值化：突出黑色边框(黑色 -> 白色)
        black_binary = gray.binary([BLACK_THRESHOLD])

        # 检测白色矩形框中心坐标
        cx, cy, blob_cnt, valid = find_white_frame_center(gray, black_binary, last_cx, last_cy)

        out_valid = valid
        if not valid:
            cx, cy = 0, 0
            out_valid = 0

        if cx and cy and out_valid:
            last_cx, last_cy = cx, cy

        # 每帧都发送坐标到串口（与旧代码一致）
        serial, last_tx_ok = send_coord(serial, cx, cy)
        if last_tx_ok:
            tx_count += 1
        if serial is None:
            serial = init_uart()

        # 显示检测结果
        if cx and cy:
            black_binary.draw_cross(cx, cy, image.Color.from_rgb(255, 255, 255), size=10)
            black_binary.draw_string(cx+15, cy-10, f"({cx},{cy})", image.Color.from_rgb(255, 255, 255))

        # 显示状态信息
        uart_state = 1 if serial else 0
        status = f"Target: ({cx},{cy}) | PORT:{UART_PORT} UART_OK:{uart_state} TX:{tx_count} OK:{last_tx_ok} | Blobs:{blob_cnt} | ok:{out_valid}"
        black_binary.draw_string(5, 5, status, image.Color.from_rgb(255, 255, 255))

        # 显示二值化图像
        disp.show(black_binary)

        # 控制台输出
        if ENABLE_CONSOLE_LOG:
            print(f"Target: ({cx}, {cy}) | TX:{tx_count} | FPS: {time.fps():.1f}")

    print("\nMaixCAM Gimbal Tracker stopped.")
