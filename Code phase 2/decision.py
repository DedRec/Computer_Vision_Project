import random
import numpy as np

places = [
    {"center": (35, 94), "radius": 35, "partially_visited": False, "totally_visited": False, "number": "red",
     "yaw": 330.1},
    {"center": (118, 35), "radius": 35, "partially_visited": False, "totally_visited": False, "number": "yellow",
     "yaw": 160.1},
    {"center": (115, 165), "radius": 40, "partially_visited": False, "totally_visited": False, "number": "blue",
     "yaw": 210.1},
    {"center": (152, 100), "radius": 17, "partially_visited": False, "totally_visited": False, "number": "green",
     "yaw": 210.1}

]
# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function


def decision_step(Rover):  # checks if the position is nearly is the same as prev position

    # print("ROVER MODE " ,Rover.mode)
    def same_pos():
        if (abs(Rover.pos[0] - Rover.pos_prev[0]) < 0.01) and (
                abs(Rover.pos[1] - Rover.pos_prev[1]) < 0.01) and Rover.mode == "forward":
            return True
        else:
            return False

    if len(Rover.t_vis) >=4:
        for place in places:
            place["partially_visited"] = False
            place["totally_visited"] = False
        Rover.p_vis = []
        Rover.t_vis = []
        print("most visited")

    def check_partially_visiting():
        # mark visited
        x, y = Rover.pos
        for place in places:
            upperx = place["center"][0] + place["radius"]
            lowerx = place["center"][0] - place["radius"]
            uppery = place["center"][1] + place["radius"]
            lowery = place["center"][1] - place["radius"]
            if (x >= lowerx and x <= upperx and y >= lowery and y <= uppery):
                place["partially_visited"] = True
                if place["number"] not in Rover.p_vis:
                    Rover.p_vis.append(place["number"])

    def check_totally_visited():
        x, y = Rover.pos
        for place in places:
            upperx = place["center"][0] + place["radius"]
            lowerx = place["center"][0] - place["radius"]
            uppery = place["center"][1] + place["radius"]
            lowery = place["center"][1] - place["radius"]
            if not (x >= lowerx and x <= upperx and y >= lowery and y <= uppery) and place["partially_visited"]:
                place["totally_visited"] = True
                if place["number"] not in Rover.t_vis:
                    Rover.t_vis.append(place["number"])
                    Rover.visited+=1

    def check_if_inside_visited():
        # rotate 180 and go forward for 0.5 seconds
        x, y = Rover.pos
        for place in places:
            upperx = place["center"][0] + place["radius"]
            lowerx = place["center"][0] - place["radius"]
            uppery = place["center"][1] + place["radius"]
            lowery = place["center"][1] - place["radius"]
            if (x >= lowerx and x <= upperx and y >= lowery and y <= uppery) and place["totally_visited"]:
                return place["yaw"] + 1.0
        return False

    # print(Rover.steer_count , "steer count")
    # print(Rover.pos_count , "POS count")
    # print(Rover.mode , "mode")
    ###STUCK ########################################

    check_partially_visiting()
    check_totally_visited()

    if Rover.mode == "forward" and check_if_inside_visited():
        Rover.mode = "jump_scare"

    if same_pos():
        Rover.pos_count += 1
    else:
        Rover.pos_count = 0

    Rover.pos_prev = Rover.pos
    if (Rover.pos_count >= Rover.max_pos_count):

        Rover.toggle += 1
        print("fffffff", Rover.toggle)
        print("fffffff22", Rover.toggle%2)
        if Rover.toggle % 2 == 0:
            Rover.mode = "reverse"
        else:
            Rover.rot_yaw = Rover.yaw
            Rover.mode = "rotate"  ##should be the stop mode where it rotate
            # seif change

        Rover.pos_count = 0
        return Rover

    ###########POS END#######################################
    #####STERING FOR TOO LONG #############
    upper = Rover.steer_prev + 2
    lower = Rover.steer_prev - 2
    if (Rover.steer >= lower) and (Rover.steer <= upper) and (
            Rover.steer > 10 or Rover.steer < -10) and not Rover.gold_flag and Rover.mode == "forward":
        Rover.steer_count += 1
    else:
        Rover.steer_count = 0

    if (Rover.steer_count >= Rover.max_steer_count):  ####better 250 needs to be tried ;;;;;;;;;;
        Rover.steer_count = 0
        Rover.brake = 15
        Rover.steer = -15
        Rover.mode = "reverse"
    Rover.steer_prev = Rover.steer

    Rover.gold_flag = False

    #####STERING END ###############################

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    Rover.throttle = Rover.throttle_set
    if Rover.nav_angles is not None:
        # Check for Rover.mode status

        if Rover.mode == 'reverse':  # i just picked up a rock and probably facing a wall # 1e4 is estimated to be 1 second
            # print("i just picked up")
            Rover.steer = 0  # bid5ol fel kol el amaken el day2a + bigarab amaken gdeda
            Rover.throttle = - 0.1
            Rover.brake = 0
            # print("going backward pep pep...")
            if Rover.picking_up == 0:
                Rover.backward_timer += 1
            #    print("increasing")
            # print("Timer values",Rover.backward_timer)

            if Rover.backward_timer > int(90):
                Rover.mode = 'forward'
                Rover.brake = 15
                Rover.backward_timer = 0

        elif Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else:  # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'

            if abs(Rover.steer) >= 13:
                Rover.throttle = 0.1
            else:
                Rover.throttle = Rover.throttle_set



        elif Rover.mode == "rotate":
            r = random.randint(-2, 2)  # if stuck try different moves
            Rover.throttle = 0
            # Release the brake to allow turning
            Rover.brake = 0
            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
            Rover.steer = -15  # Could be more clever here about which way to turn -1/1
            if ( abs(Rover.yaw - Rover.rot_yaw) >= (120 + (-r * (30))) ) :
                Rover.mode = "forward"  ##should be the stop mode where it rotate


        elif Rover.mode == "jump_scare":  # just saw the devil
            if Rover.vel == 0:
                Rover.throttle = 0
                Rover.mode = "scared"
                Rover.first_yaw = Rover.yaw
                return Rover
            else:
                Rover.brake = 15
                Rover.throttle = 0
                # Release the brake to allow turning


        elif Rover.mode == "scared":  # rotate to run
            # Release the brake to allow turning
            r = random.randint(-1, 1)  # if stuck try different moves
            Rover.brake = 0
            Rover.throttle = 0
            yaw2 = check_if_inside_visited()
            Rover.brake = 0
            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
            if(Rover.first_yaw >= 180):
                Rover.steer = -15  # Could be more clever here about which way to turn -1/1
            else :
                Rover.steer = 15
            Rover.throttle = 0
            # if (abs(Rover.yaw-yaw) <= 2): #rotate 180 degres till equal yaw
            if (abs((Rover.yaw - Rover.first_yaw)) >= (155 + 10*r)):  # rotate till = place["yaw"]
                Rover.mode = "run"
            return Rover

        elif Rover.mode == "run":  # keep going until out

            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -8, 8)
            Rover.brake = 0
            Rover.throttle = 0.1
            if not check_if_inside_visited():
                Rover.mode = "forward"
            return Rover


        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15  # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                    Rover.mode = 'forward'

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    if Rover.near_sample:
        Rover.gold_flag = True
        Rover.throttle = 0
        Rover.brake = 20
        Rover.steer = 0


    elif len(Rover.rocks_angles) > 0:
        Rover.gold_flag = True
        Rover.throttle = 0.1
        Rover.steer = np.clip(np.mean(Rover.rocks_angles * 180 / np.pi), -15, 15)

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.mode = "reverse"

    return Rover
