import sys
import cv2
import getopt
import math
###################this uses sort tracker############
from args import cnet_path, model_path, source_path, downrail, uprail
sys.path.insert(0, cnet_path)

from detectors.detector_factory import detector_factory
from opts import opts

from sort import *



class multi_pose:
    def __init__(self):
        
        self.TASK = 'multi_pose' # or 'multi_pose' for human pose estimation
        self.thresh = 0.2
        self.opt = opts().init('{} --load_model {} --vis_thresh {}'.format(self.TASK, model_path, self.thresh).split(' '))
        self.detector = detector_factory[self.opt.task](self.opt)

        self.count=0
        self.c = 0
        self.trackers_stop_timeer = list()
        self.previous_location = list()
        # self.dst = 10
        self.dst = 20
        self.trackers = cv2.MultiTracker_create()
        self.rol_avg_list = []



    def __repr__(self):
        return ("*** Hand railing complaince ***")


    def calculate_iou(self,boxA, boxB):
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
 
	    # compute the area of intersection rectangle
        interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
 
	    # compute the area of both the prediction and ground-truth
	    # rectangles
        boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
        boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
 
	    # compute the intersection over union by taking the intersection
	    # area and dividing it by the sum of prediction + ground-truth
	    # areas - the interesection area
        iou = interArea / float(boxAArea + boxBArea - interArea)
 
	    # return the intersection over union value
        return iou

    def pose(self):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        # out = cv2.VideoWriter('/home/dinesh/Desktop/sarvesh/poseestimation/src/n.mp4',fourcc, 25.0, (1024,576))

        cap = cv2.VideoCapture(source_path)
    
        pose_points = ['Nose','LeftEye','RightEye','LeftEar','RightEar','LeftShoulder',
                'RightShoulder','LeftElbow','RightElbow','LeftWrist','RightWrist','LeftHip',
                'RightHip','LeftKnee','RightKnee','LeftAnkle','RightAnkle']
        dit = []
        dit1 = []
        hold_trackers = {}
        hold = 0
        detection = []
        mot_tracker = Sort()
        while True:
            ret,img = cap.read()
            image = img.copy()
            if ret:
                detection = []

                result = self.detector.run(image)['results']
                
                for i in range(len(result[1])):
                        if (result[1][i][4])>self.thresh:
                            self.count+=1

                if self.count == 0:
                    hold = 0
                for j in range(self.count):
                    box = result[1][j][0:5]
                    box1 = result[1][j][5:39]
                    e1 = result[1][j][0]
                    f1 = result[1][j][1]
                    e2 = result[1][j][2]
                    f2 = result[1][j][3]
                    detection.append([e1,f1,e2,f2])
                    flag = False


                    for i in range(0,len(box1),2):
                        # cv2.circle(img,(int(box1[i]),int(box1[i+1])),5,(0,0,255),-1)
                        if self.c == 9:
                            xm = int(box1[i])
                            ym = int(box1[i+1])
                            

                        if self.c == 10:
                            xn = int(box1[i])
                            yn = int(box1[i+1])
                        
                        if self.c == 3:
                            xf = int(box1[i])
                            yf = int(box1[i+1])

                        if self.c == 4:
                            xt = int(box1[i])
                            yt = int(box1[i+1])
                        self.c += 1


                    for w_pair in uprail:
                        xq = w_pair[0]
                        yq = w_pair[1]
                        l_w = math.sqrt(math.pow((xq - xn),2) + math.pow((yn - yq),2))
                        r_w = math.sqrt(math.pow((xq - xm),2) + math.pow((ym - yq),2))
                        distance_w = min(l_w,r_w)

                        #try:
                        cv2.circle(image,(xf,yf),5,(0,0,255),-1)
                        cv2.circle(image,(xn,yn),5,(0,0,255),-1)
                        cv2.circle(image,(xm,ym),5,(0,0,255),-1)
                        l_n = math.sqrt(math.pow((xm - xt),2) + math.pow((ym - yt),2))
                        r_n = math.sqrt(math.pow((xn - xf),2) + math.pow((yn - yf),2))
                        distance_n=min(l_n,r_n)
                        # cv2.putText(image, "N{}".format(int(distance_n)), (50,50), cv2.FONT_HERSHEY_COMPLEX, 1.3, (0, 0, 255), 1, cv2.LINE_AA) 
                        #print("wwwwww",int(distance_w))
                        dit.append(distance_w)
                        dit1.append(distance_n)

                    try:
                        for d in bbs_ids:
                            if self.calculate_iou((e1,f1,e2,f2),(d[0],d[1],d[2],d[3]))>0.5:
                                if "{}".format(int(d[4])) in hold_trackers:
                                    if min(dit) <= self.dst and min(dit1) > 65:
                                        hold_trackers['{}'.format(int(d[4]))][0] += 1
                                        hold_trackers['{}'.format(int(d[4]))][1] += 1
                                    else:
                                        hold_trackers['{}'.format(int(d[4]))][0] -= 1
                                        hold_trackers['{}'.format(int(d[4]))][1] += 1
                    except:
                        print("pass")
                    
                    dit = []
                    dit1 = []
                    
                    ''' iterate through each tracker and update the dictionary, hold_trackers'''
                    try:
                        for d in bbs_ids:
                            if hold_trackers['{}'.format(int(d[4]))][1] == 10:
                                if hold_trackers['{}'.format(int(d[4]))][0] > 2:
                                    # print("list",hold_trackers['{}'.format(int(d[4]))][0])
                                    hold = 1
                                    hold_trackers['{}'.format(int(d[4]))] = [0,0,1]
                                    print("holding:", hold_trackers)
                                    cv2.putText(image, "holding rail", (d[0],d[1]), cv2.FONT_HERSHEY_COMPLEX, 1.3, (0, 255, 0), 1, cv2.LINE_AA)
                                else:
                                    hold = 2
                                    hold_trackers['{}'.format(int(d[4]))] = [0,0,-1]
                                    print("not holding:", hold_trackers)
                                    cv2.putText(image, "holding rail", (d[0],d[1]), cv2.FONT_HERSHEY_COMPLEX, 1.3, (0, 255, 0), 1, cv2.LINE_AA)
                    except:
                        print("pass2")
                        
                    cv2.rectangle(image, (int(result[1][j][0]),int(result[1][j][1])),(int(result[1][j][2]),int(result[1][j][3])),[0,0,255],2)
                    
                    self.c = 0

                ''' See if third element in the item for a key is 1 or -1 and update the status''' 
                try:
                    for d in bbs_ids:
                        if hold_trackers['{}'.format(int(d[4]))][2] == 1:
                            cv2.putText(image, "holding rail", (int(d[0]),int(d[1])), cv2.FONT_HERSHEY_COMPLEX, 1.3, (0, 255, 0), 1, cv2.LINE_AA)
                        elif hold_trackers['{}'.format(int(d[4]))][2] == -1:
                            cv2.putText(image, "not holding rail", (int(d[0]),int(d[1])), cv2.FONT_HERSHEY_COMPLEX, 1.3, (0, 255, 0), 1, cv2.LINE_AA)
                except Exception as e:
                    print("pass2",e)

                ######### update sort ########
                ''' update trackers, if hold_trakers is empty, add all the detections from the trackers list (bbs_ids)
                else delete previously detected tracker if it is not detected presently and add the newly found trackers to
                the dictionary '''
                set_flag = False
                bbs_ids = mot_tracker.update(np.array(detection))
                for d in bbs_ids:
                    cv2.putText(image, '{}'.format(int(d[4])), (int(d[0]), int(
                        d[1])), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 255), 1)
                if not hold_trackers:
                    for d in bbs_ids:
                        hold_trackers['{}'.format(int(d[4]))] = [0,0,0]
                else:
                    for d in bbs_ids:
                        if '{}'.format(int(d[4])) not in hold_trackers:
                            hold_trackers['{}'.format(int(d[4]))] = [0,0,0]
                    for key,value in hold_trackers.copy().items():
                        for d in bbs_ids:
                            if int(key) == d[4]:
                                set_flag = True
                        if not set_flag:
                            hold_trackers.pop(key,None)
                        set_flag = False


                # out.write(image)
                # print("my dict", hold_trackers)
                self.count = 0
                
                cv2.imshow("Multi_pose", image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    break
            else:
                break

        # out.release()

if __name__ == '__main__':

    s = multi_pose.__new__(multi_pose)
    s.__init__()
    print(s)
    s.pose()
    