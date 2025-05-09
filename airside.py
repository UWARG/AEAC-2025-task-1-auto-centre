from picamera2 import Picamera2
import cv2
import numpy as np
import pyautogui
from pymavlink import mavutil
import math
import sys

# === CONSTANTS === #
LOCKED_RADIUS = 40  # max radius to lock
GUIDE_RADIUS = 120  # max radius to be in guide mode
DETECT_SLEEP_TIME = 10
PX_TO_MS = 0.00002  # pixel-to-velocity scale factor

# === Initialize camera ===
pyautogui.FAILSAFE = False
pyautogui.moveTo(0, pyautogui.size()[1])
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())
picam2.start()
picam2.set_controls({"AeEnable": False, "ExposureTime": 25, "AnalogueGain": 16.0})

# === OpenCV fullscreen window ===
cv2.namedWindow("Camera Preview", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("Camera Preview", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

# === Connect to MAVLink ===
print("Connecting to MAVLink...")
mav = mavutil.mavlink_connection(
    "/dev/ttyAMA0", baud=57600, source_component=191, source_system=1
)
mav.wait_heartbeat()
print(
    f"Heartbeat received from system {mav.target_system}, component {mav.target_component}"
)

# === Request GPS and RC channel data ===
mav.mav.request_data_stream_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    1,
    1,
)
mav.mav.request_data_stream_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
    5,
    1,
)
print("Requested GLOBAL_POSITION_INT and RC_CHANNELS streams\n")

# === mavlink mode trolling ===
current_mode = 5  # 5 is loiter, 4 is guided


def change_mode(mode) -> bool:  # noqa: ANN001
    """
    Request the flight controller to send a non-existant message to the Rpi
    """

    if mode == current_mode:
        return True

    message = mav.mav.command_long_encode(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,  # Confirmation
        1,  # Random message ID that doesn't exist
        mode,
        0,
        0,
        0,
        0,
        0,
    )

    mav.mav.send(message)

    response = mav.recv_match(
        type="COMMAND_ACK", blocking=True, timeout=10
    )  # check with manasva if blocking is chill
    if (
        response
        and response.result == 0
        and response.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE
    ):
        # print(response)
        # print(response.command)  # Seems to return 512 when the command doesn't exist
        current_mode = mode
        return True

    return False


def find_beacon(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        beacon_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(beacon_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return cx, cy
    return None


# === Main loop ===
try:
    pending_lat, pending_lon = None, None
    lat, lon = None, None
    gps_text = "Waiting for GPS..."
    ir_error_text = ""
    rc_chan_7 = 0
    last_rc_state = False
    coords_sent = False

    undetect_time = 0
    wait_for_undetect = False

    while True:
        frame = picam2.capture_array()

        # Read all available MAVLink messages
        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break

            if msg.get_type() == "RC_CHANNELS":
                current_rc_state = (
                    msg.chan7_raw >= 1200 if msg.chan7_raw is not None else False
                )

                if current_rc_state != last_rc_state:
                    coords_sent = False
                    last_rc_state = current_rc_state

                rc_chan_7 = msg.chan7_raw if msg.chan7_raw is not None else 0

            elif msg.get_type() == "GLOBAL_POSITION_INT":
                pending_lat = msg.lat / 1e7
                pending_lon = msg.lon / 1e7

        # === Beacon detection ===
        beacon_center = find_beacon(frame)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_thickness = 2
        color = (0, 255, 0)
        text_x = 10
        text_y = 50
        img_center_x = frame.shape[1] // 2
        img_center_y = frame.shape[0] // 2

        # == Update gps coords == #
        if pending_lat is not None and pending_lon is not None:
            lat = pending_lat
            lon = pending_lon
            gps_text = f"Lat: {lat:.7f}, Lon: {lon:.7f}"

        if beacon_center and not wait_for_undetect:
            cx, cy = beacon_center
            offset_x = cx - img_center_x
            offset_y = cy - img_center_y
            error = math.hypot(offset_x, offset_y)
            ir_error_text = f"IR Error: {error:.2f} px"

            text = "TARGET AQUIRED"

            if rc_chan_7 >= 1200:
                text = "TARGET LOCKING"

                vel_x = -offset_y * PX_TO_MS
                vel_y = offset_x * PX_TO_MS

                if error < GUIDE_RADIUS:
                    change_mode(4)  # Change to guided mode

                # Send velocity command to MAVLink in NED frame
                if current_mode == 4:
                    mav.mav.set_position_target_local_ned_send(
                        int(mav.time_since("SYSTEM_TIME") * 1e6),
                        mav.target_system,
                        mav.target_component,
                        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                        0b110111100111,  # Use only vel_X, vel_Y
                        0,
                        0,
                        0,  # Position
                        vel_x,
                        vel_y,
                        0,  # Velocity
                        0,
                        0,
                        0,  # Acceleration
                        0,
                        0,  # Yaw and yaw rate
                    )

                # If the target is locked, send the coordinates to GCS
                if error < LOCKED_RADIUS:
                    text = "TARGET LOCKED"

                    # Draw a circle around the target
                    color = (0, 0, 255)
                    cv2.circle(frame, (cx, cy), 20, color, 2)

                    # Send coordinates to GCS if not already sent
                    if current_mode == 4:
                        statustext = f"Target: {lat:.7f}, {lon:.7f}"
                        mav.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO, statustext.encode()
                        )
                    if not change_mode(5):
                        sys.exit(1)
                    undetect_time = (
                        time.time() + DETECT_SLEEP_TIME
                    )  # 10 seconds of undetect time
                    wait_for_undetect = True
            else:
                wait_for_undetect = False
                if not change_mode(5):
                    sys.exit(1)

            cv2.line(frame, (cx, 0), (cx, frame.shape[0]), color, 2)
            cv2.line(frame, (0, cy), (frame.shape[1], cy), color, 2)
            offset_text = f"{offset_x} px horiz, {-1 * offset_y} px vert"
        elif wait_for_undetect:
            text = "SLEEPING"
            offset_text = ""
            ir_error_text = ""
            wait_for_undetect = time.time() < undetect_time
        else:
            text = "NO TARGET"
            offset_text = ""
            ir_error_text = ""
            if not change_mode(5):
                sys.exit(1)

        # Draw overlay text
        cv2.putText(
            frame,
            text,
            (text_x, text_y),
            font,
            font_scale,
            color,
            font_thickness,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            offset_text,
            (text_x, text_y + 30),
            font,
            font_scale,
            color,
            font_thickness,
            cv2.LINE_AA,
        )

        # WARG branding (bottom-right)
        text_warg = "WARG Precision Targeting"
        font_scale_warg = 0.5
        text_warg_size = cv2.getTextSize(text_warg, font, font_scale_warg, 1)[0]
        warg_text_x = frame.shape[1] - text_warg_size[0] - 10
        warg_text_y = frame.shape[0] - 10
        cv2.putText(
            frame,
            text_warg,
            (warg_text_x, warg_text_y),
            font,
            font_scale_warg,
            color,
            1,
            cv2.LINE_AA,
        )

        # GPS and IR error text (bottom-left)
        gps_text_x = 10
        gps_text_y = frame.shape[0] - 10
        ir_error_text_y = gps_text_y - 20

        cv2.putText(
            frame,
            ir_error_text,
            (gps_text_x, ir_error_text_y),
            font,
            font_scale_warg,
            color,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            gps_text,
            (gps_text_x, gps_text_y),
            font,
            font_scale_warg,
            color,
            1,
            cv2.LINE_AA,
        )

        # Show final frame
        cv2.imshow("Camera Preview", frame)

        # Exit on space bar
        if cv2.waitKey(1) & 0xFF == 32:
            break

except KeyboardInterrupt:
    print("\nInterrupted by user.")
finally:
    cv2.destroyAllWindows()
    picam2.stop()
