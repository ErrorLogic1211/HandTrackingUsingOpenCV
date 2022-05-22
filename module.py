import cv2
import mediapipe as mp
import time
from math import *
import serial

serialcomm = serial.Serial('/dev/ttyACM1',9600)
serialcomm.timeout = 1
thumb = [50,110]
index = [0,160]
mid = [0,175]
ring = [0,160]
pinky = [0,140]

class handDetector():
    def __init__(self, mode=False, maxHands = 2, detectionCon=1, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  #cv2.COLOR_BGR2RGB
        self.results = self.hands.process(img)
        # print(results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
        return img


    def findPosition(self, img, handNo=0, draw= True ):

        lmlist = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]

            for id, lm in enumerate(myHand.landmark):
                # print(id,lm)
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                #print(id, cx, cy)
                lmlist.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 10, (255, 0, 255), cv2.FILLED)

        return lmlist

        #static_image_mode = False,
        #max_num_hands = 2,
        #min_detection_confidence = 0.5,
        #min_tracking_confidence = 0.5

def main():
    pTime = 0
    cTime = 0
    cap = cv2.VideoCapture(0)
    detector = handDetector()

    while True:
        success, img = cap.read()
        img = detector.findHands(img)
        lmlist = detector.findPosition(img)
        '''if len(lmlist) !=0:
            print(lmlist[4])'''
        for l in lmlist:
        	print(l," ")
        print("\n")
        print(lmlist)
        if(lmlist):
        	thumbd = sqrt(((lmlist[2][1]-lmlist[4][1])**2)+((lmlist[2][2]-lmlist[4][2])**2))
        	indexd = sqrt(((lmlist[5][1]-lmlist[8][1])**2)+((lmlist[5][2]-lmlist[8][2])**2))
        	midd = sqrt(((lmlist[9][1]-lmlist[12][1])**2)+((lmlist[9][2]-lmlist[12][2])**2))
        	ringd = sqrt(((lmlist[13][1]-lmlist[16][1])**2)+((lmlist[13][2]-lmlist[16][2])**2))
        	pinkyd = sqrt(((lmlist[17][1]-lmlist[20][1])**2)+((lmlist[17][2]-lmlist[20][2])**2))
        	thumbv = (lmlist[2][2]-lmlist[4][2])
        	thumbx = (lmlist[2][2]-lmlist[4][2])
        	indexv = (lmlist[5][2]-lmlist[8][2])
        	midv = (lmlist[9][2]-lmlist[12][2])
        	ringv = (lmlist[13][2]-lmlist[16][2])
        	pinkyv = (lmlist[17][2]-lmlist[20][2])
        	if(thumbv<=0 or thumbd <80):
        		thumbd = 0
        	if(indexv<=0):
        		indexd = index[0]
        	if(midv<=0):
        		midd = mid[0]
        	if(ringv<=0):
        		ringd = ring[0]
        	if(pinkyv<=0):
        		pinkyd = pinky[0]
        	thumbd=round(100-((min(thumbd,thumb[1]-thumb[0])/(thumb[1]-thumb[0]))*100))
        	indexd=round(100-((min(indexd,index[1])/(index[1]-index[0]))*100))
        	midd=round(100-((min(midd,mid[1])/(mid[1]-mid[0]))*100))
        	ringd=round(100-((min(ringd,ring[1])/(ring[1]-ring[0]))*100))
        	pinkyd=round(100-((min(pinkyd,pinky[1])/(pinky[1]-pinky[0]))*100))
        	print(thumbd,indexd,midd,ringd,pinkyd,"\n")
        	s = str(thumbd) + " " + str(indexd) + " " +str(midd) + " " +str(ringd) + " " +str(pinkyd) + "\n"
        	serialcomm.write(s.encode())
     # change 4 to any number this to choose landmark of hand very good go back to land marks

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(img, f'FPS: {int(fps)}', (10, 70), cv2.FONT_HERSHEY_PLAIN,
                    3, (255, 0, 255), 3)

        cv2.imshow("Image", img)
        cv2.waitKey(1)
        if 0xFF == ord('q'):
        	break
    serialcomm.close()
    cap.release()
    cv2.destroyAllWindows()    
        	
        

if __name__ =="__main__":
    main()
