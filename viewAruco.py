import cv2
import matplotlib.pyplot as plt

cap = cv2.VideoCapture('data/Experiment01VIDaruco.mp4')
nframe = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

print("nframe =", nframe)
cap.set(1, 300) # arguments: 1: laisser, 2: numéro du frame
ret, frame = cap.read()
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
plt.figure()
plt.imshow(gray)
plt.show()
cap.release()