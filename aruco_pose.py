import cv2
import cv2.aruco as aruco
import numpy as np

cap = cv2.VideoCapture(0)

a = 0.5
objectPoints = np.array([[-a/2, a/2, 0],
                        [a/2, a/2, 0],
                        [a/2, -a/2, 0],
                        [-a/2, -a/2, 0]])

mtx = 0 # TODO
dist = 0 # TODO

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    # lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(corners)

    if corners:
        gray = aruco.drawDetectedMarkers(gray, corners)

        rett, rvec, tvec = cv2.solvePnP(objectPoints, corners[0][0], mtx, dist)
        rvec = cv2.Rodrigues(rvec)
        print(tvec)

        # print(rejectedImgPoints)
        # Display the resulting frame
        cv2.imshow('frame', gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()