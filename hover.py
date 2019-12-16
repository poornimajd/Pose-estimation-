import cv2
import numpy as np
def hover():
    drawing = False # true if mouse is pressed
    mode = True # if True, draw rectangle. Press 'm' to toggle to curve
    ix,iy = -1,-1
    a=[]
    # mouse callback function
    def draw_circle(event,x,y,flags,param):
        global ix,iy,drawing,mode
        
        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            ix,iy = x,y

        elif event == cv2.EVENT_MOUSEMOVE:
            if drawing == True:
                a.append([x,y])

                for i in range(8):
                    cv2.circle(frame,(x,y),5,(0,0,255),-1)

        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False
           

    cap = cv2.VideoCapture("/home/dinesh/Desktop/sarvesh/poseestimation/src/floorfirst.avi") 
    ret, frame = cap.read()
    
    cv2.namedWindow('image')
    cv2.setMouseCallback('image',draw_circle)
    #print(a)

    while(1):
        cv2.imshow('image',frame)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('m'):
            mode = not mode
        elif k == 27:
            break

    cv2.destroyAllWindows()
    return a


if __name__ == '__main__':
    b = hover()
    print(b)