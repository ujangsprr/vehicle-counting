import cv2
import numpy as np
import vehicles
import time
import paho.mqtt.client as mqttClient
 
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        global Connected                #Use global variable
        Connected = True                #Signal connection 
    else:
        print("Connection failed")
 
Connected = False   #global variable for the state of the connection
 
broker_address= "mqtt.dioty.co"
port = 1883
user = "ujangsprr@gmail.com"
password = "a1e2b36d"
 
client = mqttClient.Client("Python")               #create new instance
client.username_pw_set(user, password=password)    #set username and password
client.on_connect= on_connect                      #attach function to callback
client.connect(broker_address, port=port)          #connect to broker
 
client.loop_start()        #start the loop
 
while Connected != True:    #Wait for connection
    time.sleep(0.1)

cnt_up=0
cnt_down=0

cap = cv2.VideoCapture('4K Road traffic video.mp4')
# cap = cv2.VideoCapture('surveillance.m4v')
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

#Get width and height of video
w = cap.get(3)
h = cap.get(4)
frameArea = h*w
areaTH = frameArea/400

# print(str(w), str(h))
# print(str(frameW), str(frameH))

#Lines
line_up=int(2*(h/5))
line_down=int(3*(h/5))

up_limit=int(1*(h/5))
down_limit=int(4*(h/5))

# print("Red line y:",str(line_down))
# print("Blue line y:",str(line_up))

line_down_color=(255,0,0)
line_up_color=(255,0,255)
pt1 =  [0, line_down]
pt2 =  [w, line_down]
pts_L1 = np.array([pt1,pt2], np.int32)
pts_L1 = pts_L1.reshape((-1,1,2))
pt3 =  [0, line_up]
pt4 =  [w, line_up]
pts_L2 = np.array([pt3,pt4], np.int32)
pts_L2 = pts_L2.reshape((-1,1,2))

pt5 =  [0, up_limit]
pt6 =  [w, up_limit]
pts_L3 = np.array([pt5,pt6], np.int32)
pts_L3 = pts_L3.reshape((-1,1,2))
pt7 =  [0, down_limit]
pt8 =  [w, down_limit]
pts_L4 = np.array([pt7,pt8], np.int32)
pts_L4 = pts_L4.reshape((-1,1,2))

#Background Subtractor
fgbg=cv2.createBackgroundSubtractorMOG2(detectShadows=True)

#Kernals
kernalOp = np.ones((3,3),np.uint8)
kernalOp2 = np.ones((5,5),np.uint8)
kernalCl = np.ones((11,11),np.uint)

font = cv2.FONT_HERSHEY_SIMPLEX
cars = []
max_p_age = 5
pid = 1

# out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (int(w),int(h)))

while(cap.isOpened()):
    ret,frame=cap.read()
    # frame = cv2.resize(frame, (frameW,frameH) ,fx=0.4, fy=0.4)

    for i in cars:
        i.age_one()
    fgmask=fgbg.apply(frame)

    today = time.strftime("%c")
    setTime = time.strftime("%S") #strftime("%m/%d/%Y, %H:%M:%S")

    if ret==True:

        #Binarization
        ret,imBin=cv2.threshold(fgmask,200,255,cv2.THRESH_BINARY)

        # cv2.imshow('imBin',imBin)

        #OPening i.e First Erode the dilate
        mask=cv2.morphologyEx(imBin,cv2.MORPH_OPEN,kernalOp)

        #Closing i.e First Dilate then Erode
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernalCl)

        # cv2.imshow('Mask',mask)

        #Find Contours
        countours0,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        for cnt in countours0:
            area=cv2.contourArea(cnt)
            # print(area)
            if area>areaTH:
                ####Tracking######
                m=cv2.moments(cnt)
                cx=int(m['m10']/m['m00'])
                cy=int(m['m01']/m['m00'])
                x,y,w,h=cv2.boundingRect(cnt)

                new=True
                if cy in range(up_limit,down_limit):
                    for i in cars:
                        if abs(x - i.getX()) <= w and abs(y - i.getY()) <= h:
                            new = False
                            i.updateCoords(cx, cy)

                            if i.going_UP(line_down,line_up)==True:
                                cnt_up+=1
                                print("ID:",i.getId(),'crossed going up at', today)
                            elif i.going_DOWN(line_down,line_up)==True:
                                cnt_down+=1
                                print("ID:", i.getId(), 'crossed going up at', today)
                            break
                        if i.getState()=='1':
                            if i.getDir()=='down'and i.getY()>down_limit:
                                i.setDone()
                            elif i.getDir()=='up'and i.getY()<up_limit:
                                i.setDone()
                        if i.timedOut():
                            index=cars.index(i)
                            cars.pop(index)
                            del i

                    if new==True: #If nothing is detected,create new
                        p=vehicles.Car(pid,cx,cy,max_p_age)
                        cars.append(p)
                        pid+=1

                cv2.circle(frame,(cx,cy),5,(0,0,255),-1)
                img=cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

        for i in cars:
            cv2.putText(frame, str(i.getId()), (i.getX(), i.getY()), font, 0.3, i.getRGB(), 1, cv2.LINE_AA)

        cnt_total = cnt_up + cnt_down

        client.publish("/ujangsprr@gmail.com/smart",cnt_total)

        str_up='UP: '+str(cnt_up)
        str_down='DOWN: '+str(cnt_down)
        str_total = 'TOTAL: '+str(cnt_total)

        print('Set Time', setTime)
        if setTime == '00':
            cnt_down = 0
            cnt_up = 0

        # print(str_up)
        # print(str_down)
        # print(str_total)

        frame=cv2.polylines(frame,[pts_L1],False,line_down_color,thickness=2)
        frame=cv2.polylines(frame,[pts_L2],False,line_up_color,thickness=2)
        frame=cv2.polylines(frame,[pts_L3],False,(255,255,255),thickness=2)
        frame=cv2.polylines(frame,[pts_L4],False,(255,255,255),thickness=2)

        cv2.putText(frame, 'SMART TRAFFIC LIGHT SYSTEM', (20, 40), font, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'IOT - KMIPN 2021', (640, 60), font, 1, (0, 0, 255), 3, cv2.LINE_AA)

        cv2.putText(frame, str_up, (20, 60), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, str_up, (20, 60), font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, str_down, (20, 80), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, str_down, (20, 80), font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, str_total, (20, 100), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, str_total, (20, 100), font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, today, (20, 120), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, today, (20, 120), font, 0.5, (0, 0, 255), 1, cv2.LINE_AA) 

        cv2.imshow('Frame',frame)

        # out.write(frame)

        if cv2.waitKey(1)&0xff==ord('q'):
            break

    else:
        break

# out.release()
# frame.release()

client.disconnect()
client.loop_stop()

cap.release()
cv2.destroyAllWindows()
