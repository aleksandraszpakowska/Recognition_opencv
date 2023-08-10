import cv2

cap = cv2.VideoCapture(0)
ar_icon = cv2.imread('ar_maker.png')

# cv2.imshow("ar_icon", ar_icon)
# cv2.waitKey(0)

height, weight, channel = ar_icon.shape

orb = cv2.ORB_create(nfeatures=1000)  # creating the detector
# key point, descriptor
kp1, des1 = orb.detectAndCompute(ar_icon, None)
# ar_icon = cv2.drawKeypoints(ar_icon,kp1,None)


while True:
    _, frame = cap.read()
    kp2, des2 = orb.detectAndCompute(frame, None)
    # frame = cv2.drawKeypoints(frame, kp2, None)

    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)
    good = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good.append(m)

    if len(good) > 50:
        print("I found AR marker on a screen! :)")
        list_kp1 = [kp1[matches.queryIdx].pt for matches in good]  # list of key points 1
        list_kp2 = [kp2[matches.trainIdx].pt for matches in good]  # list of key points 2

        round_list_kp1 = [(int(x), int(y)) for (x, y) in list_kp1]  # image keypoints coordinates
        round_list_kp2 = [(int(x), int(y)) for (x, y) in list_kp2]  # camera keypoints coordinates

        print("List of keypoints from Image: ", round_list_kp1)
        print("List of keypoints from camera: ", round_list_kp2)

    # print(len(good))
    imgFeatures = cv2.drawMatches(ar_icon, kp1, frame, kp2, good, None, flags=2)

    cv2.imshow("ImgFeatures", imgFeatures)
    # cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('d'):
        break

cap.release()
cv2.destroyAllWindows()
