import cv2
import serial
import numpy as np
import imutils
import time

cap = cv2.VideoCapture(0)

# cap.set(3, 640)
# cap.set(4, 480)

# Setting up the serial connection (PORT Arduino is connected to, baud rate of the connection)
ser = serial.Serial('COM3', 115200)
# Wait for the connection to be established
time.sleep(5)


while True:
    objectType = 4;
    objectX = 50;
    objectY = 60;

    time.sleep(0.1)
    ret, frame1 = cap.read()
    frame = cv2.flip(frame1, 1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([90, 60, 0])
    upper_blue = np.array([121, 255, 255])

    lower_green = np.array([36, 20, 20])
    upper_green = np.array([86, 255, 255])

    maskb = cv2.inRange(hsv, lower_blue, upper_blue)
    maskg = cv2.inRange(hsv, lower_green, upper_green)

    cntsb = cv2.findContours(maskb, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cntsg = cv2.findContours(maskg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cntsb = imutils.grab_contours(cntsb)
    cntsg = imutils.grab_contours(cntsg)

    # print("Object Area: ", cv2.contourArea(cntsb))

    for c in cntsb:

        if  200000 > cv2.contourArea(c) > 100000:
            # cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)

            M = cv2.moments(c)

            CX = int(M['m10']/M['m00'])
            CY = int(M['m01']/M['m00'])

            # print("Blue Centroid Y: ", ((639-CX)*40/640))
            # print("Blue Centroid X: ", (CY*30/480))
            # print("Blue Big Box")

            cv2.circle(frame, (CX, CY), 7, (255, 255, 255), -1)
            cv2.putText(frame, "Center", (CX-20, CY-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            objectType = 3;
            objectY = ((639-CX)*40/640);
            objectX = (CY*30/480);  
        
        elif 100000 > cv2.contourArea(c) > 70000:
            #cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)

            M = cv2.moments(c)

            CX = int(M['m10']/M['m00'])
            CY = int(M['m01']/M['m00'])

            # print("Blue Centroid Y: ", ((639-CX)*40/640))
            # print("Blue Centroid X: ", (CY*30/480))
            # print("Blue small box")

            cv2.circle(frame, (CX, CY), 7, (255, 255, 255), -1)
            cv2.putText(frame, "Center", (CX-20, CY-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            objectType = 1;
            objectY = ((639-CX)*40/640);
            objectX = (CY*30/480);
       
        elif 70000 > cv2.contourArea(c) > 50000:
            #cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)

            M = cv2.moments(c)

            CX = int(M['m10']/M['m00'])
            CY = int(M['m01']/M['m00'])

            # print("Blue Centroid Y: ", ((639-CX)*40/640))
            # print("Blue Centroid X: ", (CY*30/480))
            # print("Blue identified")

            cv2.circle(frame, (CX, CY), 7, (255, 255, 255), -1)
            cv2.putText(frame, "Center", (CX-20, CY-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            objectType = 5;
            objectY = ((639-CX)*40/640);
            objectX = (CY*30/480);

    for c in cntsg:

        if  200000 > cv2.contourArea(c) > 100000:
            #cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)

            M = cv2.moments(c)

            CX = int(M['m10']/M['m00'])
            CY = int(M['m01']/M['m00'])

            # print("Green Centroid Y: ", ((639-CX)*40/640))
            # print("Green Centroid X: ", (CY*30/480))
            # print("Green Big Box")

            cv2.circle(frame, (CX, CY), 7, (255, 255, 255), -1)
            cv2.putText(frame, "Center", (CX-20, CY-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            objectType = 2;
            objectY = ((639-CX)*40/640);
            objectX = (CY*30/480);

        elif 100000 > cv2.contourArea(c) > 70000:
            #cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)

            M = cv2.moments(c)

            CX = int(M['m10']/M['m00'])
            CY = int(M['m01']/M['m00'])

            # print("Green Centroid Y: ", ((639-CX)*40/640))
            # print("Green Centroid X: ", (CY*30/480))
            # print("Green small box")

            cv2.circle(frame, (CX, CY), 7, (255, 255, 255), -1)
            cv2.putText(frame, "Center", (CX-20, CY-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            objectType = 0;
            objectY = ((639-CX)*40/640);
            objectX = (CY*30/480);

        elif 70000 > cv2.contourArea(c) > 50000:
            # cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)

            M = cv2.moments(c)

            CX = int(M['m10']/M['m00'])
            CY = int(M['m01']/M['m00'])

            # print("Green Centroid Y: ", ((639-CX)*40/640))
            # print("Green Centroid X: ", (CY*30/480))
            # print("Green identified")

            cv2.circle(frame, (CX, CY), 7, (255, 255, 255), -1)
            cv2.putText(frame, "Center", (CX-20, CY-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            objectType = 5;
            objectY = ((639-CX)*40/640);
            objectX = (CY*30/480);

    cellDetails= np.array([[ 7, 29 ],
                          [ 15, 34 ],
                          [ 23, 29 ],
                          [ 7, 19 ],
                          [ 15, 24 ],
                          [ 23, 19 ],
                          [ 7, 10 ],
                          [ 15, 14 ],
                          [ 23, 10 ],
                          [ 15, 5 ]])
    
    for i in range(len(cellDetails)):
        cv2.circle(frame, ((639-(cellDetails[i][1]*16)), (cellDetails[i][0]*16)), 5, (0, 0, 0), -1)

    print("Object Type: ", objectType)
    print("Object X: ", objectX)
    print("Object Y: ", objectY)

    # Convert data to binary format
    data_to_send = bytearray([objectType, int(objectX), int(objectY)])

    # Send data
    ser.write(data_to_send)
    ser.flush()

    ack = ser.readline().decode().strip()  # Read the acknowledgment
    if ack == "ACK":
        print("Acknowledgment received!")
    else:
        print("No valid acknowledgment received.")


    cv2.rectangle(frame, (639-(15*16), 9*16), (639-(20*16), 20*16), (0, 0, 0), 2)
    # cv2.circle(frame, (639, 300), 2, (225, 225, 225), -1)

    frame = cv2.flip(frame, 1)
    # frame full screen
    # frame = cv2.resize(frame, (1920, 1080))
    # frame = imutils.resize(frame, width=1920)

    cv2.imshow('Centroid', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()