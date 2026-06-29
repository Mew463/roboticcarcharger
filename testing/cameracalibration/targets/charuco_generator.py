import cv2
path = "testing/cameracalibration/targets/"
def homing_charuco():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

    board = cv2.aruco.CharucoBoard((4, 6), 0.05, 0.04, aruco_dict)

    img = board.generateImage((2480, 3508))  # A4 @ 300 DPI

    cv2.imwrite(path + "homing_charuco.png", img)
    
def tesla_charuco():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

    board = cv2.aruco.CharucoBoard(
        (11, 14), 
        0.03,   # 4 cm squares
        0.02,   # 2 cm markers
        aruco_dict
    )

    # 3. Use a resolution that matches the 8:11 aspect ratio
    # Width = 800, Height = 1100 (or multiples thereof)
    img = board.generateImage((800, 1100), marginSize=10)
    
    cv2.imwrite(path + "tesla_charuco.png", img)

homing_charuco()