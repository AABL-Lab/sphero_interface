import cv2

def init_videocapture(width=1280, height=720):
    camera = cv2.VideoCapture(2, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    camera.set(cv2.CAP_PROP_FPS, 30)
    return camera

def main():
    # >>>> Open Video Stream
    video = init_videocapture()
    # Exit if video not opened.
    if not video.isOpened():
        print("Could not open video")
        return 

    print(f"Video fps is set to {video.get(cv2.CAP_PROP_FPS)}")

    while (True):
        # Read first frame.
        ok, frame = video.read()
        if not ok:
            print('Couldnt read frame')
            continue

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'): # if press SPACE bar
            break
    
main()