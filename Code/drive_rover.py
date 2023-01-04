# Do the necessary imports
import argparse
import shutil
import base64
from datetime import datetime
import os
import cv2
import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from PIL import Image
from flask import Flask
from io import BytesIO, StringIO
import json
import pickle
import matplotlib.image as mpimg
import time
import keyboard

# Import functions for perception and decision making
from perception import perception_step
from decision import decision_step
from supporting_functions import update_rover, create_output_images

# Initialize socketio server and Flask application
# (learn more at: https://python-socketio.readthedocs.io/en/latest/)
sio = socketio.Server()
app = Flask(__name__)

# Read in ground truth map and create 3-channel green version for overplotting
# NOTE: images are read in by default with the origin (0, 0) in the upper left
# and y-axis increasing downward.
ground_truth = mpimg.imread('../calibration_images/map_bw.png')
# This next line creates arrays of zeros in the red and blue channels
# and puts the map into the green channel.  This is why the underlying
# map output looks green in the display image
ground_truth_3d = np.dstack((ground_truth * 0, ground_truth * 255, ground_truth * 0)).astype(np.float)


# Define RoverState() class to retain rover state parameters
class RoverState():
    def __init__(self):
        self.toggle = 0  # used when stuck
        self.first_yaw = 0  # used when stuck
        self.rot_yaw = 0  # used when stuck
        self.visited = 0  # used when stuck
        self.p_vis = []  # used for visited
        self.t_vis = []  # used when stuck
        self.gold_flag = False  # if he sees gold he marks this as true
        self.rotate_timer = 0  # rotating timer
        self.start_time = None  # To record the start time of navigation
        self.total_time = None  # To record total duration of navigation
        self.img = None  # Current camera image
        self.pos = None  # Current position (x, y)
        self.pos_prev = (0, 0)  # Current position (x, y)
        self.pos_count = 0  # Current position (x, y) how long have i been in the same pos
        self.yaw = 0  # Current yaw angle
        self.pitch = None  # Current pitch angle
        self.roll = None  # Current roll angle
        self.vel = None  # Current velocity
        self.steer = 0  # Current steering angle
        self.throttle = 0  # Current throttle value
        self.brake = 0  # Current brake value
        self.nav_angles = None  # Angles of navigable terrain pixels
        self.nav_dists = None  # Distances of navigable terrain pixels
        self.ground_truth = ground_truth_3d  # Ground truth worldmap
        self.debug = 0  # Debugging mode enable initially disabled
        self.mode = 'forward'  # Current mode (can be forward or stop)
        self.throttle_set = 0.2  # Throttle setting when accelerating
        self.brake_set = 25  # Brake setting when braking
        # The stop_forward and go_forward fields below represent total count
        # of navigable terrain pixels.  This is a very crude form of knowing
        # when you can keep going and when you should stop.
        self.stop_forward = 50  # Threshold to initiate stopping
        self.go_forward = 500  # Threshold to go forward again
        self.max_vel = 1.8  # Maximum velocity (meters/second)
        # Image output from perception step
        # Update this image to display your intermediate analysis steps
        # on screen in autonomous mode
        self.vision_image = np.zeros((160, 320, 3), dtype=np.float)
        # Worldmap
        # Update this image with the positions of navigable terrain
        # obstacles and rock samples
        self.worldmap = np.zeros((200, 200, 3), dtype=np.float)
        self.samples_pos = None  # To store the actual sample positions
        self.samples_to_find = 6  # To store the initial count of samples
        self.samples_located = 0  # To store number of samples located on map
        self.samples_collected = 0  # To count the number of samples collected
        self.near_sample = 0  # Will be set to telemetry value data["near_sample"]
        self.picking_up = 0  # Will be set to telemetry value data["picking_up"]
        self.send_pickup = False  # Set to True to trigger rock pickup
        self.backward_timer = 0  # timer to go back after picking up a stone
        self.steer_count = 0  # timer to go back after picking up a stone how long have i been on the same steering angle
        self.steer_prev = 0  # timer to go back after picking up a stone

        self.max_steer_count= 200
        self.max_pos_count = 40


# Initialize our rover
Rover = RoverState()

# Variables to track frames per second (FPS)
# Intitialize frame counter
frame_counter = 0
# Initalize second counter
second_counter = time.time()
fps = None


# Define telemetry function for what to do with incoming data
@sio.on('telemetry')
def telemetry(sid, data):
    global frame_counter, second_counter, fps
    frame_counter += 1
    # Do a rough calculation of frames per second (FPS)
    if (time.time() - second_counter) > 1:
        fps = frame_counter
        frame_counter = 0
        second_counter = time.time()
    #print("Current FPS: {}".format(fps))

    if data:
        global Rover
        # Initialize / update Rover with current telemetry
        Rover, image = update_rover(Rover, data)
        if keyboard.is_pressed('m'):
            if Rover.debug == 0:
                Rover.debug = 1
            else:
                Rover.debug = 0
        if np.isfinite(Rover.vel):

            # Execute the perception and decision steps to update the Rover's state
            Rover = perception_step(Rover)
            Rover = decision_step(Rover)
            str_cnt = int((Rover.steer_count / Rover.max_steer_count) * 15)
            pos_cnt = int((Rover.pos_count / Rover.max_pos_count) * 15)
            print(
                f"\r |steer count |{str_cnt * '█' + '-' * (15 - str_cnt)}|  |pos count| {pos_cnt * '▮' + '▯' * (15 - pos_cnt)} |rock_found| {int(Rover.gold_flag == True) * '⚫'} |mode| {Rover.mode} |P_Visited| {Rover.p_vis} |t_Visited| {Rover.t_vis} |total visits| {len(Rover.t_vis)}",
                end="\r", flush=True)


            # Create output images to send to server
            out_image_string1, out_image_string2 = create_output_images(Rover)

            # The action step!  Send commands to the rover!

            # Don't send both of these, they both trigger the simulator
            # to send back new telemetry so we must only send one
            # back in respose to the current telemetry data.

            # If in a state where want to pickup a rock send pickup command
            if Rover.send_pickup and not Rover.picking_up:
                send_pickup()
                # Reset Rover flags
                Rover.send_pickup = False
            else:
                # Send commands to the rover!
                commands = (Rover.throttle, Rover.brake, Rover.steer)
                send_control(commands, out_image_string1, out_image_string2)

        # In case of invalid telemetry, send null commands
        else:

            # Send zeros for throttle, brake and steer and empty images
            send_control((0, 0, 0), '', '')

        # If you want to save camera images from autonomous driving specify a path
        # Example: $ python drive_rover.py image_folder_path
        # Conditional to save image frame if folder was specified
        if args.image_folder != '':
            timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
            image_filename = os.path.join(args.image_folder, timestamp)
            image.save('{}.jpg'.format(image_filename))

    else:
        sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    #print("connect ", sid)
    send_control((0, 0, 0), '', '')
    sample_data = {}
    sio.emit(
        "get_samples",
        sample_data,
        skip_sid=True)


def send_control(commands, image_string1, image_string2):
    # Define commands to be sent to the rover
    data = {
        'throttle': commands[0].__str__(),
        'brake': commands[1].__str__(),
        'steering_angle': commands[2].__str__(),
        'inset_image1': image_string1,
        'inset_image2': image_string2,
    }
    # Send commands via socketIO server
    sio.emit(
        "data",
        data,
        skip_sid=True)
    eventlet.sleep(0)


# Define a function to send the "pickup" command
def send_pickup():
    #print("Picking up")
    pickup = {}
    sio.emit(
        "pickup",
        pickup,
        skip_sid=True)
    eventlet.sleep(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument(
        'image_folder',  # 'image_folder'
        type=str,
        nargs='?',
        default='',
        help='Path to image folder. This is where the images from the run will be saved.'
    )
    args = parser.parse_args()
    # Specify destination folder to save into
    args.image_folder = '../IMG_RUN'

    os.system('rm -rf IMG_stream/*')
    if args.image_folder != '':
        #print("Creating image folder at {}".format(args.image_folder))
        if not os.path.exists(args.image_folder):
            os.makedirs(args.image_folder)
        else:
            shutil.rmtree(args.image_folder)
            os.makedirs(args.image_folder)
        #print("Recording this run ...")
    else:
        pass
        #print("NOT recording this run ...")

    # wrap Flask application with socketio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
