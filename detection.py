"""
Created on 2022
@author: hugo & jn
"""

import os
import sys
import time
from datetime import datetime
import cv2
import cv2.aruco as aruco
import numpy as np
from pymavlink import mavutil
from picamera import PiCamera,Color
from picamera.array import PiRGBArray
from dronekit import (LocationGlobal, LocationGlobalRelative, VehicleMode,
                      connect)


class Detection:
    def __init__(self, camera):
        #Booléens
        self.aruco_found = False
        self.square_found = False
        #Camera
        self.camera = camera
        self.camera_res = (100,100) #TODO : trouver la bonne res
        self.camera.framerate = 32
        #self.rawCapture = PiRGBArray(self.camera, size=(640, 480)) (package Picamera nécessaire)
        #Aruco
        self.marker_size = 25  #en cm, on prend un gros marqueur pour être tranquille (quasi aucune detection de carrés blancs)

        #Resolution
        # Focal length and sensors dimensions for Pi camera
        # See: https://www.raspberrypi.com/documentation/accessories/camera.html
        focal_length = 3.60  # Focal length [mm]
        horizotal_res = 640  # Horizontal resolution (x dimension) [px]
        vertical_res = 480  # Vertical resolution (y dimension) [px]
        sensor_length = 3.76  # Sensor length (x dimension) [mm]
        sensor_height = 2.74  # Sensor length (y dimension) [mm]
        self.dist_coeff_x = sensor_length / (focal_length * horizotal_res)
        self.dist_coeff_y = sensor_height / (focal_length * vertical_res)
        self.x_imageCenter = int(horizotal_res / 2)
        self.y_imageCenter = int(vertical_res / 2)

        #Ecriture dossiers
        self.dossier = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
        self.path = os.path.join("\imgs\data", self.dossier)
        os.mkdir(self.path)  # création d'un dossier pour stocker la nouvelle image

        #dictionnaire aruco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        self.parameters = aruco.DetectorParameters_create()
        self.closeToAruco = False

    def Detection_aterr(self, latitude, longitude, altitude, direction):
        #INITIALISATION
        start_time = time.time()
        self.aruco_found, self.square_found = False
        x_pixel, y_pixel = None

        # TODO : régler le pathing
        arucoId = 0
        frame = cv2.imread('arucos.jpg')  # définir un path d'une image
        font = cv2.FONT_HERSHEY_PLAIN  # Text font for frame annotation
        cv2.imshow('begin', frame)

        # Detection Aruco
        # Il faut renvoyer deux bool (aruco_detected / white_square detected) et la position en (x,y) d'une potentielle detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, id, rej = aruco.detectMarkers(gray, self.arucoDictionnary, parameters=self.arucoParameters)

        if len(corners) == 0:  # si on ne détecte pas d'aruco, on cherche un carré blanc sur l'image pour asservir dessus
            # --------------- Detection White Squares ------------------------
            # ------------- Image processing for white squares -------------
            blur = cv2.GaussianBlur(frame, (5, 5), 0)  # Gaussian blur filter : removing noise
            hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)  # Convert from BGR to HLS color space
            lower_bound = (0, 230, 0)  # white color in HLS space
            upper_bound = (255, 255, 255)
            mask_hls = cv2.inRange(hls, lower_bound, upper_bound)
            cv2.imshow('blur', blur)
            cv2.imshow('hls', mask_hls)

            name = "Test_1_Img_"
            cv2.imwrite(os.path.join(self.path, "hls_" + name + ".png"),
                        mask_hls)  # écriture de notre image traitée dans le dossier
            # Closing detected elements

            closing_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))  # notre kernell est de 7x7 et test du rectangle
            mask_closing = cv2.morphologyEx(mask_hls, cv2.MORPH_CLOSE, closing_kernel)
            contours, hierarchy = cv2.findContours(mask_closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # --------------White square corners ---------------------------
            for c in contours:
                # pour identifier un carre
                peri = cv2.arcLength(c, True)  # prend les contours fermés et calcul ce qu'il faut
                approx = cv2.approxPolyDP(c, 0.02 * peri,
                                          True)  # TODO : trouver le bon paramètre de précision #p
                area = cv2.contourArea(c)
                # ---------------Sqr filter -----------------
                x, y, w, h = cv2.boundingRect(
                    approx)  # on récup l'approximation polygonale du contour et on l'inscrit dans un rect
                ar = w / float(h)

                if 0.90 <= ar <= 1.10:  # Condition sur l'aire de notre carré
                    cv2.drawContours(frame, [c], -1, (0, 0, 255), 2)
                    x_centerPixel_target = np.mean(c, axis=0)[0][0]
                    y_centerPixel_target = np.mean(c, axis=0)[0][1]
                    # arrete_marker_pxl = sqrt(area)
                    cv2.imwrite(os.path.join(self.path, "final_.png"), frame)
                    pixelTest = mask_closing[int(y_centerPixel_target), int(x_centerPixel_target)]
                    if pixelTest == 255:  # verifie couleur du carre detecte 255 c est blanc
                        # Boolean and counter update
                        self.square_found = True
                        # ici : on a un carré blanc donc potentiellement un aruco, il faut asservir le drone en position pour se rapprocher et verifier si oui ou non
        else:
            ids = id.flatten()  # on s'assure que notre liste d'id est une liste de dimension 1
            for (markersCorners, markersIds) in zip(corners,
                                                    ids):  # on boucle sur les tuples de (corners, id)
                corners = markersCorners.reshape((4, 2))  # les 4 coins et 2 coordonnées
                (topLeft, topRight, bottomLeft, bottomRight) = corners
                topLeft = (
                    int(topLeft[0]),
                    int(topLeft[1]))  # on récupère les coordonnées de chaque point pour l'affichage
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))

                cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                cv2.putText(frame, str(markersIds),
                            (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_PLAIN, 0.5, (0, 0, 255), 2)
                cv2.imwrite(os.path.join(self.path, "final_.png"), frame)
        cv2.imshow('final', frame)
        cv2.waitKey(0)
        return(x_pixel, y_pixel , self.aruco_found , self.square_found) #on renvoie les deux booleens et nos pixels cible