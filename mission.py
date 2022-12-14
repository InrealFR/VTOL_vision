#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2022

@author: Thomas Pavot modified by Hugo Pons
"""
import os
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time
import math

from threading import Thread
import threading

from math import atan2, cos, radians, sin, sqrt, pi
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from array import array
from datetime import datetime
#from picamera import PiCamera,Color
#from picamera.array import PiRGBArray

from detection import Detection
from commande_drone import Drone
import sys
import time

#--------------------- Parametres du vehicule ----------------------------
vitesse = .3 #m/s
altitudeDeVol = 15
research_whiteSquare = True
distanceAccuracy = 2 # rayon en metre pour valider un goto

# Boolean variables
global aruco_seen
global good_aruco_found
global white_square_seen

# Counter variables
global counter_no_detect
counter_no_detect = 0
global counter_white_square
counter_white_square = 0
global counter_something
counter_something = 0

global id_to_test
id_to_test = -1
global saved_markers
# Initialized saved_markers with Unmanned Valley GPS position
saved_markers = {-1: (LocationGlobalRelative(52.171490,4.417461,0), True)}


global x_centerPixel_target
x_centerPixel_target =  None
global y_centerPixel_target
y_centerPixel_target =  None
global x_imageCenter
x_imageCenter = 0
global y_imageCenter
y_imageCenter = 0
global altitudeAuSol
global altitudeRelative
global longitude
global latitude

class RedirectText(object):
    def __init__(self,aWxTextCtrl):
        self.file=aWxTextCtrl
         
    def flush(self):
        pass
 
    def write(self,string):
        f = open(self.file,'a')
        f.write(string)
        f.close()

log="./test.log"
redir=RedirectText(log)
# sys.stdout=redir
# sys.stderr=redir



def asservissement(drone_object, detection_object, last_errx, last_erry, errsumx, errsumy,altitudeAuSol,x_centerPixel_target, y_centerPixel_target):

  # PID Coefficients
  kpx = 0.005
  kpy = 0.005
  kdx = 0.0001  # 0.00001 working "fine" for both
  kdy = 0.0001
  kix = 0.000001  # 0.0000001
  kiy = 0.000001
  
  vx = vy = vz = 0

  if altitudeAuSol < 5 : #on asservit moins violent quand on se raproche du sol
    kpx = 0.003
    kpy = 0.003
  else:
    kpx = 0.005
    kpy = 0.005

  print("Pixels values - x:%s - y:%s" % (x_centerPixel_target, y_centerPixel_target))
  
  if x_centerPixel_target == None or y_centerPixel_target == None : # echec Detection
    if counter_no_detect > 10 :   #on fixe le nombre d'image consecutive sans Detection pour considerer qu il ne detecte pas
      if altitudeAuSol > 30 :  # arr??t du drone si il est trop haut
        drone_object.set_velocity(0, 0, 0, 1)
        print ("[asserv] altitudeRelative > 30")
        return 0, 0, 0, 0
    
      else :  # pas de Detection Drone prend de l'altitude => champ de vision plus large => d??tection plus ais??e
        vx = vy =  0
        vz = -1
        drone_object.set_velocity(vx, vy, vz, 1)
        #
    elif counter_no_detect > 1 :   # fixer la position du Drone en cas de non detection pour s'assurer qu'il traite l'image un certain nombre de fois
      vx = vy = vz = 0
      drone_object.set_velocity(vx, vy, vz, 1)   
    errx = 0
    erry = 0
    errsumx = 0
    errsumy = 0
    
  else :  # Detection ok
    print ("[asserv] detection OK")
    errx = 324 - x_centerPixel_target #le d??placement ?? effectuer en x (pos de notre cam??ra - pos du carr?? sur la camera)
    erry = 224 - y_centerPixel_target
    
    dist_center = math.sqrt(errx**2+erry**2)
    dist_angle = atan2(erry, errx)
    heading = drone_object.vehicle.attitude.yaw
    alpha = dist_angle + heading
    errx = dist_center * cos(alpha)
    erry = dist_center * sin(alpha)
    if abs(errx) <= 50:   #marge de 50pxl pour considerer que la cible est au centre de l image
      errx = 0
    if abs(erry) <= 20:
      erry = 0
        
    # PD control
    dErrx = (errx - last_errx)# / delta_time
    dErry = (erry - last_erry)# / delta_time
    errsumx += errx# * delta_time
    errsumy += erry# * delta_time
    
    vx = (kpx * errx) + (kdx * dErrx) + (kix * errsumx)
    vy = (kpy * erry) + (kdy * dErry) + (kiy * errsumy)
    vz = 0
    
    if altitudeAuSol < 3 :
      vz = 0.1  # a changer pour descendre
    elif altitudeAuSol > 9 :
      vz = 1  # a changer pour descendre
    elif altitudeAuSol > 5:
      vz = 0.5
    else:
      vz = 0.25

    # Establish limit to outputs
    vx = min(max(vx, -5.0), 5.0)
    vy = min(max(vy, -5.0), 5.0)
    vx = -vx                        # High opencv is south Dronekit
    
    # Dronekit
    # X positive Forward / North
    # Y positive Right / East
    # Z positive down
    
    if altitudeAuSol < 2 :
      dist_center_threshold = 50
    
    else :
      dist_center_threshold = 300
    if altitudeAuSol <= 1:
      print("Vehicle in LAND mode")
      drone_object.vehicle.mode = VehicleMode("LAND")
      drone_object.set_velocity(0,0,0.1,1)
      while (drone_object.vehicle.location.global_relative_frame.alt !=0 & drone_object.vehicle.armed != True):
       pass
      print("Aterrissage r??ussi. Arr??t moteurs.")
      drone_object.vehicle.armed = False
      drone_object.vehicle.close()
    if dist_center <= dist_center_threshold :
      drone_object.set_velocity(vy, vx, vz, 1) 
      #print("vy : "+str(vy)+" vx : "+str(vx)+" vz : "+str(vz)+" dist_center <= 30"	
    else :
      #lancer un deplacement pour se rapprocher du centre sans descendre ou monter
      vz = 0
      drone_object.set_velocity(vy, vx, vz, 1)  # Pour le sense de la camera, X controle le 'east' et Y controle le 'North'
      #print("vy : "+str(vy)+" vx : "+str(vx)+" vz : "+str(vz)+" dist_center decale")

  # Return last errors and sums for derivative and integrator terms
  print(" errx :" + str(errx) + " erry : " + str(erry)+"\n")
  
  return errx, erry, errsumx, errsumy








#---------------------------------------------MISSION PRINCIPALE--------------------------------------

def mission_largage(drone_name, id_to_find, truck):

  id_to_test = id_to_find
  
  first_time_aruco_found = True
  
  last_errx = 0
  last_erry = 0
  errsumx = 0
  errsumy = 0
  vid = cv2.VideoCapture(0)
  counter_no_detect = 0
  counter_white_square = 0
  altitudeAuSol = 0.0
  dist_center = 0.0
  print("[mission] Mission started!")
  drone_object = Drone()    #permet de connecter le drone via dronekit en creant l objet drone
  detection_object = Detection(vid)  # creer l objet detection
  #########verrouillage servomoteur et procedure arm and takeoff
  print("[mission] Launching and take off routine...")
 
  drone_object.lancement_decollage(20, drone_name)

  #########passage en mode AUTO et debut de la mission
  drone_object.passage_mode_Auto()
  #self.vehicle.mode = VehicleMode("AUTO")
  
  # a partir d'un certain waypoint declencher le thread de detection
  while drone_object.vehicle.commands.next <= 0:
    pass

  while (drone_object.get_mode() == "GUIDED" or drone_object.get_mode() == "AUTO"):
    #time.sleep(0.1)
    capture, frame = vid.read()
    cv2.imshow('camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
    	break
    # actualisation de l altitude et gps
    print("actualisation gps")
    #altitudeAuSol = drone_object.vehicle.rangefinder.distance #On r??cup??re l'altitude au sol gr??ce ?? un capteur (rangefinder) qui regarde vers le bas
    altitudeAuSol = -1*drone_object.vehicle.location.local_frame.down
    print(altitudeAuSol)
    altitudeRelative = drone_object.vehicle.location.global_relative_frame.alt
    longitude = drone_object.vehicle.location.global_relative_frame.lon
    latitude = drone_object.vehicle.location.global_relative_frame.lat
    heading = drone_object.vehicle.attitude.yaw
    
    #le script Detection Target
    x_centerPixel_target, y_centerPixel_target, aruco_found, square_found = detection_object.Detection_aterr(latitude, longitude, altitudeAuSol, heading, id_to_test)
    # Asservissement control
    if drone_object.get_mode() == "GUIDED" :
      last_errx, last_erry, errsumx, errsumy = asservissement(drone_object, detection_object, last_errx, last_erry, errsumx, errsumy, altitudeAuSol,x_centerPixel_target, y_centerPixel_target)
    
    if not drone_object.get_mode() == "GUIDED" and not drone_object.get_mode() == "AUTO":
      break
  
    #--------------- Case 1: ArUco found -----------------------
    if aruco_found:
      print("[detection] Case 1: Good ArUco ID found!")

      if first_time_aruco_found:
        first_time_aruco_found = False
        start_time = time.time()
      
      counter_no_detect = 0

      while drone_object.get_mode() != "GUIDED":
        drone_object.set_mode("GUIDED")

      print("x_centerPixel_target : "+str(x_centerPixel_target))
      print("y_centerPixel_target : "+str(y_centerPixel_target))
      print("x_object : "+str(detection_object.x_imageCenter))
      print("y_object : "+str(detection_object.y_imageCenter))
      dist_center = math.sqrt((detection_object.x_imageCenter-x_centerPixel_target)**2+(detection_object.y_imageCenter-y_centerPixel_target)**2)
      #print("[mission] Current distance: %a ; Altitude: %b" % (dist_center, altitudeAuSol))

      elapsed_time = time.time() - start_time
 
    #--------------- Case 2: Some white square seen --------------------
    elif square_found:
      print("[detection] Case 2: Some white square seen.")

      counter_no_detect = 0
      counter_white_square += 1

      print("[mission] Detection of 1 or many white squares (%i times)" % counter_white_square)

      # Check saved_ids in detection dictionary
      for saved_id in saved_markers :
        # Check boolean: if False, needs to be explored
        if saved_markers[saved_id][1] == False:
          # if saved_id > 1001 and saved_markers[saved_id-1][1] == False:
          #  saved_markers[saved_id-1].pop()
          while drone_object.get_mode() != "GUIDED":
            drone_object.set_mode("GUIDED")
          id_to_test = saved_id
          print("[mission] Detection targetted towards id %s" % id_to_test)

    #--------------- Case 3: No detection of white or ArUco ------------
    else:
      print("[detection] Case 3: No detection of white or ArUco. (%a times)" %counter_no_detect)

      counter_no_detect += 1
      counter_white_square = 0

      # print("[mission] No detection or wrong tag (%i times)" % compteur_no_detect)
      # print("[mission] compteur_whiteSquare (%i times)" % compteur_whiteSquare)
      
      if counter_no_detect > 35:
        print("[mission] 35 times without tag or white detection, not interesting place.")

        while drone_object.get_mode() != "RTL" :
          drone_object.set_mode("RTL")

          # Reset visual PID errors
          last_errx = 0
          last_erry = 0
          errsumx = 0
          errsumy = 0

  # Geofencing security, shutdown motors, VERY DANGEROUS!!!
  # if drone_object.get_mode() == "BRAKE":
  #   # print("Enter BRAKE mode")
  #   msg= drone_object.vehicle.message_factory.command_long_encode(
  #     # time_boot_ms (not used)
  #     0, 0,  # target system, target component
  #     mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,  # frame
  #     0, 1, 0, 0, 0, 0, 0, 0)

  if drone_object.get_mode() == "GUIDED" or drone_object.get_mode() == "AUTO":  #securite pour ne pas que le drone reprenne la main en cas d interruption
    #########repart en mode RTL
    drone_object.set_mode("RTL") #### modif preciser qu on est en guided avant et ajouter l altitude du RTL


mission_largage('futuna',50,False)
#--------------------------------------------------------------
#if __name__ == "__main__":

  # Mission de largage classique
  #drone_name = str(sys.argv[1])
  #id_to_find = int(sys.argv[2])
  #if len(sys.argv) >= 3 and drone_name in ["futuna", "spacex", "walle"]:
    #mission_largage(drone_name, id_to_find, False)
    # print("Mission largage")
    #if len(sys.argv) == 4:
      #if sys.argv[3] == "silent":
        # Mission de largage silencieuse
        #mission_silent(drone_name)
        # print("Mission silent")
      #elif sys.argv[3] == "truck":
        # Mission de largage sur camion
        #mission_largage(drone_name, id_to_find, True)

  # Erreur dans la saisie des arguments
  #else:
    #print("Please specify: \n * a valid drone name \n * an ArUco id argument \n * a mission keyword if necessary (silent or truck)")
    
  #print ("End of code")
