import cv2
import matplotlib.pyplot as plt
import math
# aruco.DICT_6X6_250
def gen_imgs_from_VID(path_to_VID):
    capture = cv2.VideoCapture(path_to_VID)
    imgs_folder = "/data/imgs/"
    nframe = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = capture.get(5) # frame rate
    while(capture.isOpened()):
        frameID = capture.get(1)
        ret, frame = capture.read()
        if(ret != True):
            break
        if(frameID < 150):
            print(frameID)
            filename = imgs_folder + "image_" + str(int(frameID)) + ".jpg"
            cv2.imwrite(filename, frame)
        
    capture.release()
    print("DONE!")

capture = cv2.VideoCapture('data/calibrationVID.mp4')
imgs_folder = "./data/imgs/"
nframe = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
fps = capture.get(5) # frame rate
while(capture.isOpened()):
    frameID = capture.get(1)
    ret, frame = capture.read()
    if(ret != True):
        break
    if(frameID < 150):
        print(frameID)
        filename = imgs_folder + "image_" + str(int(frameID)) + ".jpg"
        cv2.imwrite(filename, frame)
    
capture.release()
print("DONE!")

