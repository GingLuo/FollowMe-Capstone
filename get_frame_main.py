import cv2
from get_frame import get_frame

if __name__ == "__main__":
    cam = get_frame.Camera()
    try:
        while True:
            path1, path2, arr3 = cam.get_pic()
            rgb = cv2.imread(path1, cv2.IMREAD_COLOR)
            depthcolor = cv2.imread(path2, cv2.IMREAD_COLOR)
            cv2.imshow("RGB", rgb)
            cv2.imshow("Depth", depthcolor)
            if cv2.waitKey(1) == ord("q"):
                break  
    except KeyboardInterrupt:
        cam.pipeline.stop()
        exit()
