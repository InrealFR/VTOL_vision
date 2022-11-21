"""
Delivery of a package at an ArUco tag with known GPS location.
"""

#TODO: largage du colis

from __future__ import print_function
from utils import points
import cv2
import cv2.aruco as aruco
import numpy as np
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

########################   CONNECTION AU DRONE   ###############################
# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
#################################################################################

#########################    VARIABLES GLOBALES    ##############################

altitude = 15
arucoToFind = 69
cap = cv2.VideoCapture(0) #N° de la cam sur l'ordi (quasiment tout le temps la 1ere donc 0)

#################################################################################

########################   FONCTIONS DE CONTROLE   ##############################
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


#Fonction pour atterrir à la position actuelle
def landing():
    #Récupère la position actuelle et place l'altitude à 0 avant de redemander un mouvement
    target = vehicle.location.global_relative_frame
    target.alt = 0
    vehicle.simple_goto(target)
    
    #On considère que c'est fini à 5cm près
    while vehicle.location.global_relative_frame.alt >= 0.05:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(1)
            
    print("Landing done")

#Fonction pour revenir depuis le sol est toute sécurité (envol avant de revenir)
def comeBack():
    global altitude
    #On s'envole jusqu'à 10m 
    print("Going back in the air")
    takeoff(10)
    
    #Paramètrage du retour en mode auto
    vehicle.parameters['RTL_ALT'] = altitude
    vehicle.parameters['WP_YAW_BEHAVIOR'] = 1
    vehicle.mode = VehicleMode("RTL")
    time.sleep(3)

#Fonction pour s'envoler
def takeoff(altitude):
    #Récupère la position actuelle et place l'altitude à celle voulue avant de redemander un mouvement
    target = vehicle.location.global_relative_frame
    target.alt = altitude
    vehicle.simple_goto(target)
    
    #On considère que c'est fini lorsque l'on a atteint au moins 95% de l'altitude
    while vehicle.location.global_relative_frame.alt <= altitude*0.95:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(1)
        
    print("Reached target altitude")
    
#Fonction pour se déplacer
def moveTo(point):
    #Déplacement
    vehicle.simple_goto(point)
    
    #Tant que l'on n'est pas assez proche de la cible, on continu à bouger
    while ((abs(vehicle.location.global_relative_frame.lat-point.lat)>=0.000003) or (abs(vehicle.location.global_relative_frame.lon-point.lon)>=0.000003)):
      time.sleep(1)
    print("Arrived on point") 
   
#Fonction pour trouver les aruco sur une image
def findArucoMarkers(img, markerSize=5, totalMarkers=1000, draw=True):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #Convertit l'image en nuance de gris
    arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_1000)   #Récupère le dictionnaire
    arucoParam = aruco.DetectorParameters_create()  #Des paramètres pour la détection
    # On récupère les polygones détectés, les id et les code qui ne sont pas dans le dict
    boxs, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)

    if draw:
        aruco.drawDetectedMarkers(img, boxs, borderColor=(0,255,0)) #affiche les aruco détectés et valides sur l'image
        #Met des box autour des aruco détectés
        if not ids is None:
            for box, di in zip(boxs, ids):
                print(di)
                cv2.putText(img, str(di), (int(box[0][0][0]), int(box[0][0][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                            (255, 0, 255), 1)

    return boxs,ids #On a besoin de récupérer les rectangles pour avoir la position sur l'image
     
#Fonction pour se positionner correctement au-dessus de l'aruco
def Positionate(arucoToFind):

    global altitude
    global cap
    center = (-1000 -1000)
    lat = 0
    lon = 0
    
    #Tant que l'on est trop loin du centre
    while abs(960-center[0])>50 or abs(540-center [1])>40:
        # Détermination de la latitude
    	if 960-center[0]>0:
    	    lon = vehicle.location.global_relative_frame.lat + 0.000005
    	elif 960 - center[0] < 0:
    	    lon = vehicle.location.global_relative_frame.lat - 0.000005
    	else :
    	    lon = vehicle.location.global_relative_frame.lat
    	    
    	# Détermination de la longitude
    	if 540-center[1]>0:
    	    lon = vehicle.location.global_relative_frame.lon + 0.000005
    	elif 540 - center[1] < 0:
    	    lon = vehicle.location.global_relative_frame.lon - 0.000005
    	else :
    	    lon = vehicle.location.global_relative_frame.lon
    	
    	#Ordre de déplacement
    	vehicle.simple_goto(LocationGlobalRelative(lat, lon, altitude))
    	
    	#Traitement à 30 Hz et laisse le temps de se déplacer un peu
    	cv2.waitkey(33)
    	
        # Récupération de l'image
        success, img = cap.read()
        img = cv2.resize(img, (1920, 1080), interpolation=cv2.INTER_AREA)

        # Appel de la fonction pour trouver les marqueurs
        arucoFound = findArucoMarkers(img)

        # Affichage de l'image
        cv2.imshow("Image", img)
        
        if not arucoFound[1] is None:
            if arucoToFind in arucoFound[1]:
                index = np.argwhere(arucoFound[1] == arucoToFind)
                print("We found it !")
                center = (arucoFound[0][index[0][0]][index[0][1]][0] + arucoFound[0][index[0][0]][index[0][1]][1] +
                          arucoFound[0][index[0][0]][index[0][1]][2] + arucoFound[0][index[0][0]][index[0][1]][3]) / 4
                
#def largage():
#################################################################################
      
##################################  MAIN  #######################################       

arm_and_takeoff(10)

print("Set default/target airspeed to 7m/s")
vehicle.airspeed = 7

print("Going to the target location")
pointToGo = LocationGlobalRelative(points["Known Drop Location"][0], points["Known Drop Location"][1], altitude)
moveTo(pointToGo)

Positionnate(arucoToFind)

print("Atterrissage")
landing()
# sleep during 5s so we can see the change in simu
time.sleep(5)

#TODO: largage du colis

print("Returning to Launch")
comeBack()

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
