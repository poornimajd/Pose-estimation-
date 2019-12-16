import sys
import cv2
import getopt
import math

from args import cnet_path, model_path, source_path, a, a1
sys.path.insert(0, cnet_path)

from detectors.detector_factory import detector_factory
from opts import opts

#from hover import hover


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

    def destroy_trackers(self,img):
        
        rel = self.trackers.getObjects()
        del_timer = []
        rel = list(rel)
        for i in range(0,len(self.trackers_stop_timeer)):
            
            if self.trackers_stop_timeer[i] > 20:
                del_timer.append(i)
        if len(del_timer) != 0:
            self.trackers = None
            self.trackers = cv2.MultiTracker_create()

            for i in range(0,len(del_timer)):
                ind = max(del_timer)
                rel.pop(ind)

                self.trackers_stop_timeer.pop(ind)
                self.previous_location.pop(ind)
                del_timer.remove(ind)

            for i in range(0,len(rel)):
                tracker = cv2.TrackerCSRT_create()
                self.trackers.add(tracker, img, (rel[i][0],rel[i][1],rel[i][2],rel[i][3]))

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
        out = cv2.VideoWriter('/home/dinesh/Desktop/sarvesh/poseestimation/src/outputfinalfinals.mp4',fourcc, 25.0, (1024,576))

        cap = cv2.VideoCapture(source_path)
    
        pose_points = ['Nose','LeftEye','RightEye','LeftEar','RightEar','LeftShoulder',
                'RightShoulder','LeftElbow','RightElbow','LeftWrist','RightWrist','LeftHip',
                'RightHip','LeftKnee','RightKnee','LeftAnkle','RightAnkle']
        dit = []
        dit1 = []
        hold = 0
        
        
        while True:
            ret,img = cap.read()
            image = img.copy()
            if ret:
                #hold = 0
                result = self.detector.run(image)['results']
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                (success, bboxes) = self.trackers.update(image)
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                for i in range(0,len(bboxes)):

                    if (((bboxes[i][0]+ (bboxes[i][2])/2)-self.previous_location[i][0])**2 + ((bboxes[i][1]+ (bboxes[i][3])/2)-self.previous_location[i][1])**2)**(1/2) <8:
                        self.trackers_stop_timeer[i] +=1 
                            
                    else:
                        self.previous_location[i] = (bboxes[i][0]+ (bboxes[i][2])/2,bboxes[i][1]+ (bboxes[i][3])/2)
                        self.trackers_stop_timeer[i] = 0

                self.destroy_trackers(image)
                for i in range(len(result[1])):
                        if (result[1][i][4])>self.thresh:
                            self.count+=1
                rel = self.trackers.getObjects()
                #print(rel)
                if self.count == 0:
                    hold = 0
                for j in range(self.count):
                    box = result[1][j][0:5]
                    box1 = result[1][j][5:39]
                    e1 = result[1][j][0]
                    f1 = result[1][j][1]
                    e2 = result[1][j][2]
                    f2 = result[1][j][3]
                    flag = False
                    for ii in range(0,len(rel)):
                        if self.calculate_iou((e1,f1,e2,f2),(rel[ii][0],rel[ii][1],rel[ii][0]+rel[ii][2],rel[ii][1]+rel[ii][3]))>0.1:  
                            flag = True
                            inde = ii

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


                        for w_pair in a:
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
                        if min(dit) <= self.dst and min(dit1) > 65:
                            self.rol_avg_list[inde].append(1)
                        else:
                            self.rol_avg_list[inde].append(-1)
                            #print("ind",len(self.rol_avg_list[inde]))

                        dit = []
                        dit1= []
                        if len(self.rol_avg_list[inde]) == 10:
                            if sum(self.rol_avg_list[inde]) >=2 :
                                print("sum",sum(self.rol_avg_list[inde]) , "list", self.rol_avg_list[inde])
                                hold = 1
                                # cv2.putText(image, "holding rail", (150,150), cv2.FONT_HERSHEY_COMPLEX, 1.3, (0, 0, 255), 1, cv2.LINE_AA)
                            else:
                                hold = 2
                                # cv2.putText(image, "not holding rail", (50,50), cv2.FONT_HERSHEY_COMPLEX, 1.3, (0, 0, 255), 1, cv2.LINE_AA)

                            self.rol_avg_list[inde] = []
                        
                            
                    except:
                        continue
                    
                    
                    results = self.trackers.getObjects()
                    for res in results:
                        cv2.rectangle(image, (int(res[0]), int(res[1])), (int(res[0] + res[2]), int(res[1] + res[3])), (0, 255, 0), 1)


                    # for res in results:

                    #     val = int(res[1]+res[3])
                    #     if val < 0:
                    #         val = 0

                    #     if self.check_for_points((int(res[0]),  int(res[1])+int(res[3]))) or self.check_for_points((int(res[0] + res[2]), int(res[1] + res[3]))):
                    #         cv2.rectangle(frame, (int(res[0]), int(res[1])), (int(res[0] + res[2]), int(res[1] + res[3])), (0, 0, 255), 1)

                    #     else:
                    #         cv2.rectangle(frame, (int(res[0]), int(res[1])), (int(res[0] + res[2]), int(res[1] + res[3])), (0, 255, 0), 1)

                  
                    
                    cv2.rectangle(image, (int(result[1][j][0]),int(result[1][j][1])),(int(result[1][j][2]),int(result[1][j][3])),[0,0,255],2)
                    

                    self.c = 0

                if flag == False:
                    #print('###########################')

                    #print(abs(self.pickle_file[y1]-x1))
                    #print('###########################')
                    tracker = cv2.TrackerCSRT_create()
                    self.trackers.add(tracker, image, (e1,f1,abs(e2-e1),abs(f2-f1)))
                    self.trackers_stop_timeer.append(0)
                    w = abs(e2-e1)/2
                    h = abs(f2-f1)/2
                    self.previous_location.append((e1+w, f1+h))

                    self.rol_avg_list.append([])
                
                if hold == 1:
                    cv2.putText(image, "holding rail", (50,50), cv2.FONT_HERSHEY_COMPLEX, 1.3, (0, 255, 0), 1, cv2.LINE_AA)
                elif hold == 2:
                    cv2.putText(image, "not holding rail", (50,50), cv2.FONT_HERSHEY_COMPLEX, 1.3, (0, 0, 255), 1, cv2.LINE_AA)
                else:
                    cv2.putText(image, "No Person", (50,50), cv2.FONT_HERSHEY_COMPLEX, 1.3, (255,0, 0), 1, cv2.LINE_AA)

            
                out.write(image)

                self.count = 0
                
                # cv2.imshow("Multi_pose", image)
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     cv2.destroyAllWindows()
                #     break
            else:
                break

        out.release()

if __name__ == '__main__':
    # s = multi_pose()

    s = multi_pose.__new__(multi_pose)
    s.__init__()
    print(s)
    
    s.pose()
    