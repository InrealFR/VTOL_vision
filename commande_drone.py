#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2022

@author: Thomas Pavot
"""
import os
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time
import math
import json

from threading import Thread
from pymavlink import mavutil, mavwp
import threading

from math import atan2, cos, radians, sin, sqrt, pi
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil 
from array import array
from datetime import datetime
from picamera import PiCamera,Color
from picamera.array import PiRGBArray
from utilities import get_distance_metres

class Drone:

  def __init__(self):
    #--------------------- Connection ----------------------------
    print("Connecting...")
    chemin_drone = '/dev/ttyACM0'
    self.vehicle = connect(chemin_drone, wait_ready=True, baud=57600, heartbeat_timeout=2)
    print("Connection OK")

#    self.fenceloader = None
#    self.fenceloader = mavwp.MAVFenceLoader()
#    self.fenceloader.clear()

  
  def arm_and_takeoff(self, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not self.vehicle.is_armable:
      print(" Waiting for vehicle to initialise...")
      time.sleep(1)
    print("Arming motors")
    # Copter should arm in GUIDED mode
    self.vehicle.mode = VehicleMode("GUIDED")
    self.vehicle.armed = True
  
    while not self.vehicle.armed:
      print(" Waiting for arming...")
      time.sleep(1)
  
    print("Taking off!")
    self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
  
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)      
        if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
  
  def move_servo(self, channel, ouvert, drone_name, duration=1):
    # Servo largage
    if channel == 10:
      if ouvert:
        if drone_name == "futuna":
          pwm = 1100
        else:
          pwm = 1900
        print("Opening servo")
      else:
        if drone_name == "futuna":
          pwm = 1900
        else:
          if drone_name == "walle":
            pwm = 1200
          else:
            pwm = 1100
        print("Closing servo")
        
    # Servo enrouleur
    if channel == 9:
      if ouvert:  # Descente
        pwm = 1450
        print("Opening servo")
      else:       # Maintien
        pwm = 1500
        print("Closing servo")

    msg = self.vehicle.message_factory.command_long_encode(0, 0,  # target_system, target_component
                                                      mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
                                                      0,  # confirmation
                                                      channel,  # servo number
                                                      pwm,  # servo position between 1000 ferme and 2000
                                                      0, 0, 0, 0, 0)  # param 3 ~ 7 not used
    # send command to vehicle
    for x in range(0,duration) :		
      self.vehicle.send_mavlink(msg)
      time.sleep(0.2)
  
  #set_mode - set the mode of the vehicle as long as we are in control
  def set_mode(self, mode):
    self.vehicle.mode = VehicleMode(mode)
    print("[mission] Mode set to %s." % mode)
    self.vehicle.flush()
   
  #get_mode - get current mode of vehicle 
  def get_mode(self):
    last_mode = self.vehicle.mode.name
    return last_mode

  def set_velocity(self, velocity_x, velocity_y, velocity_z, duration):
    # only let commands through at 10hz
    print("[mission] Velocity set to values: vx: %.2f ; vy: %.2f ; vz %.2f." % (velocity_x, velocity_y, velocity_z))
  
    #global last_set_velocity
    #last_set_velocity = time.time()
  
    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
      0,  # time_boot_ms (not used)
      0, 0,  # target system, target component
      mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
      0x0DC7,  # type_mask (ignore pos | ignore acc)
      0, 0, 0,  # x, y, z positions (not used)
      velocity_x, velocity_y, velocity_z,
      # x, y, z velocity in m/s -- X positive forward or North/ Y positive right or East / Z positive down
      0, 0, 0,  # x, y, z acceleration (not used)
      0, 0)  # yaw, yaw_rate (not used)
    
    # send command to vehicle
    for x in range(0,duration) :
      self.vehicle.send_mavlink(msg)
      time.sleep(0.1)
   
  def goto(self, targetLocation, distanceAccuracy):
    """
    Function to move to a target location with a given precision.
    
    Based on the simple_goto function from DroneKit completed with a
    wait function checking if the drone is in a desired accuracy
    circle around the target location.
    """
    # Simple goto DroneKit function
    self.vehicle.simple_goto(targetLocation)

    # Stop action if we are no longer in GUIDED mode
    while self.vehicle.mode.name=="GUIDED": 
      currentLocation = self.vehicle.location.global_relative_frame
      remainingDistance = get_distance_metres(currentLocation, targetLocation)
      print ("[mission] Distance to the GPS target: ", remainingDistance)
      # print("Distance to the GPS target: %.2fm" % d)

      # If the distance to the target verifies the distance accuracy
      if remainingDistance <= distanceAccuracy:
        print("[mission] Reached GPS target!")
        break  # Then break the waiting loop
      time.sleep(1)

  def lancement_decollage(self, altitudeDeVol, drone_name):
    #Initialisaion du programme en mode stabilize
    self.vehicle.mode = VehicleMode("STABILIZE")
    #verrouillage servomoteur de larguage
    self.move_servo(10, False, drone_name)
    while True:
      print ("[mission] Current mode: %s. Waiting for AUTO mode." % self.vehicle.mode)
      if (self.vehicle.mode == VehicleMode("AUTO")):
        self.arm_and_takeoff(altitudeDeVol)
        break
      time.sleep(0.25)

  def readmission(self,aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    cmds = self.vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist
    
  def upload_mission(self, aFileName):
    """
    Upload a mission from a file. 
    """
    #Read mission from file
    missionlist = readmission(aFileName)
    
    print("\nUpload mission from a file: %s" % aFileName)
    #Clear existing mission from vehicle
    print(' Clear mission')
    cmds = self.vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    self.vehicle.commands.upload()

  def download_mission(self):
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = self.vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

  def save_mission(self, aFileName):
    """
    Save a mission in the Waypoint file format 
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print("\nSave mission from Vehicle to file: %s" % aFileName)    
    #Download mission from vehicle
    missionlist = self.download_mission()
    #Add file-format information
    output='QGC WPL 110\n'
    #Add home location as 0th waypoint
    home = self.vehicle.home_location
    print("home.lat : "+str(home.lat))
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    #Add commands
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        print(" Write mission to file")
        file_.write(output)
        
  def printfile(self, aFileName):
    """
    Print a mission file to demonstrate "round trip"
    """
    print("\nMission file: %s" % aFileName)
    with open(aFileName) as f:
        for line in f:
            print(' %s' % line.strip()) 


  def distance_to_current_waypoint(self):
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = self.vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=self.vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

  def passage_mode_Auto(self):
    """
    Permet d'initialiser le code pour lancer la mission en auto
    """
    print("[mission] Starting mission AUTO.")
    # Reset mission set to first (0) waypoint
    self.vehicle.commands.next=0
    
    # Set mode to AUTO to start mission
    self.vehicle.mode = VehicleMode("AUTO")
