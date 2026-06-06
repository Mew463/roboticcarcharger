import cv2
import math
import numpy as np
import logging
import time
from pupil_apriltags import Detector
from settings.config import CameraConfig
from algorithms.moving_avg import MovingAverage
class CharucoTracking():
    def __init__(self):
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        # # Set the resolution
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        # Check the actual result
        w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.logger = logging.getLogger(f"{__name__}")
        self.logger.info(f"{w} x {h} fps : {fps}")
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.board_wall = cv2.aruco.CharucoBoard(
            (6, 8),        # squaresX, squaresY
            0.04,          # square length (meters)
            0.025,         # marker length
            self.aruco_dict
        )
        
        self.board_tesla = cv2.aruco.CharucoBoard(
            (11, 14),        # squaresX, squaresY
            0.014,          # square length (meters)
            0.009,         # marker length
            self.aruco_dict
        )

        self.last_frame_time = time.time()
        self.the_image = None
        self.brightness_avg = MovingAverage(50)
        
    def close(self):
        self.cap.release()
        cv2.destroyAllWindows()
    
    def rotation_matrix_to_euler_angles(self, R):
        """
        Convert the output rotation_matrix from opencv to euler angles. 
        Euler angles are easier to interpret
        """
        sy = math.sqrt(R[0,0]**2 + R[1,0]**2)

        singular = sy < 1e-6

        if not singular:
            roll  = math.atan2(R[2,1], R[2,2])
            pitch = math.atan2(-R[2,0], sy)
            yaw   = math.atan2(R[1,0], R[0,0])
        else:
            roll  = math.atan2(-R[1,2], R[1,1])
            pitch = math.atan2(-R[2,0], sy)
            yaw   = 0

        return roll, pitch, yaw
    
    def tagdata_to_robot_coordinates(self, rawtagdata):
        """
        Coordinates returned are unintuitive. Change the coordinate to be more relevant 
        to robot orientation
        """
        if (rawtagdata is not None):
            tagdata = {
                "x": rawtagdata["x"],
                "y": rawtagdata["z"],
                "z": rawtagdata["y"],
                "roll": rawtagdata["yaw"],
                "pitch" : rawtagdata["roll"],
                "yaw" : rawtagdata["pitch"] 
            } 
            # self.logger.debug(tagdata)
            return tagdata
        else:
            return None
        
    def show_frame(self):
        """
        Displays a frame that the camera saw after running get_frame()
        """
        cv2.namedWindow("ChArUco Detection", cv2.WINDOW_NORMAL)
        cv2.imshow("ChArUco Detection", self.the_image)
        cv2.waitKey(1)
        
    def get_frame_brightness(self):
        if (self.brightness_avg.is_full()):
            return round(self.brightness_avg.get_avg() / 2.55, 1)
        else:
            return None

    def save_frame(self):
        """ _summary_ Take a picture and save it to the disk
        
        """
        ret, frame = self.cap.read()

        while (not ret):
            ret, frame = self.cap.read()

        filename = "sensors/img.jpg"

        cv2.imwrite(filename, frame)

    def get_frame(self, use_wall_board = True):
        """_summary_

        Args:
            use_wall_board (bool, optional): Whether we should be expecting the Charuco board attached to the wall. Defaults to True.

        Returns:
            Dict: {x,y,z,roll,pitch,yaw} Describing the orientation of the Charuco board
        """
        tagdata = {
            "pose": None,
            "pose_est": None
        }
        
        ret, frame = self.cap.read()
        # self.logger.info(f"FPS: {(1/(time.time() - self.last_frame_time)):0.2f}")
        # self.last_frame_time = time.time()
        if not ret:
            return tagdata # So that down stream code doesn't crash
        
        if (use_wall_board):
            charuco_board = self.board_wall
            frame = cv2.flip(frame, -1)
        else:
            charuco_board = self.board_tesla
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray)
        self.brightness_avg.add(brightness)
        
        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)
        
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            # Interpolate ChArUco corners
            num_corners, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, charuco_board
            )
            
            if (num_corners is not None and num_corners >= 8): # GET ACTUAL POSE
                # Draw ChArUco corners
                cv2.aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)

                # Estimate pose
                rvec = np.zeros((3, 1))
                tvec = np.zeros((3, 1))

                success = cv2.aruco.estimatePoseCharucoBoard(
                    charuco_corners,
                    charuco_ids,
                    charuco_board,
                    CameraConfig.K,
                    CameraConfig.dist,
                    rvec,
                    tvec
                )

                if success:
                    # Draw axis
                    cv2.drawFrameAxes(frame, CameraConfig.K, CameraConfig.dist, rvec, tvec, 0.05)

                    # Position
                    x, y, z = tvec.flatten()

                    # Rotation matrix
                    R, _ = cv2.Rodrigues(rvec)

                    # Euler angles
                    roll, pitch, yaw = self.rotation_matrix_to_euler_angles(R)

                    roll_deg  = math.degrees(roll)
                    pitch_deg = math.degrees(pitch)
                    yaw_deg   = math.degrees(yaw)

                    # Keep your normal-based angle
                    normal = R[:, 2]
                    angle = math.degrees(math.atan2(normal[0], normal[2]))

                    # Display info
                    rawtagdata = {
                        "x": float(round(x, 3)),
                        "y": float(round(y, 3)),
                        "z": float(round(z, 3)),
                        "roll": round(roll_deg,1),
                        "pitch" : round(pitch_deg, 1),
                        "yaw" : round(yaw_deg, 1) 
                    }
                    tagdata["pose"] = self.tagdata_to_robot_coordinates(rawtagdata)
            else: # ESTIMATE ROUGH ORIENTATION

                marker_centers = []
                all_points = np.concatenate(corners).reshape(-1, 2)
                x_coords = all_points[:, 0]
                pixel_width = np.max(x_coords) - np.min(x_coords)
                if use_wall_board:
                    BOARD_WIDTH_METERS = 6 * 0.04
                else:
                    BOARD_WIDTH_METERS = 6 * 0.014
                distance = (BOARD_WIDTH_METERS * CameraConfig.K[0,0]) / pixel_width
                
                for marker in corners:
                    marker = marker[0]
                    cx = np.mean(marker[:,0])
                    cy = np.mean(marker[:,1])
                    marker_centers.append([cx, cy])

                marker_centers = np.array(marker_centers)

                center_x = np.mean(marker_centers[:,0])
                center_y = np.mean(marker_centers[:,1])
                
                h, w = gray.shape

                normalized_x = (center_x - w/2) / (w/2)
                normalized_y = (center_y - h/2) / (h/2)
                
                tagdata["pose_est"] = {
                        "x": float(round(normalized_x, 3)),
                        "y": float(round(distance, 3)),
                        "z": float(round(normalized_y, 3)),
                    }
        self.the_image = frame
        return tagdata
