import numpy as np
import cv2
import random


# Identify pixels above the threshold and
# Returns a colored image with 3 color channels
def color_thresh(img, rgb_thresh=(160, 160, 160), kernel_size=5):
    gimg = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])

    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will contain a boolean array with "True" whenever threshold is met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                   & (img[:, :, 1] > rgb_thresh[1]) \
                   & (img[:, :, 2] > rgb_thresh[2])

    above_thresh_gaussian = (gimg[:, :, 0] > rgb_thresh[0]) \
                            & (gimg[:, :, 1] > rgb_thresh[1]) \
                            & (gimg[:, :, 2] > rgb_thresh[2])

    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    color_select[above_thresh_gaussian] = 1
    # Return the binary image
    return color_select

def color_thresh_color_img(img, rgb_thresh=(160, 160, 160), kernel_size=5):
    gimg = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, :])

    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will contain a boolean array with "True" whenever threshold is met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                   & (img[:, :, 1] > rgb_thresh[1]) \
                   & (img[:, :, 2] > rgb_thresh[2])

    above_thresh_gaussian = (gimg[:, :, 0] > rgb_thresh[0]) \
                            & (gimg[:, :, 1] > rgb_thresh[1]) \
                            & (gimg[:, :, 2] > rgb_thresh[2])

    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = [255]*3
    color_select[above_thresh_gaussian] = [255]*3
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float32)
    y_pixel = -(xpos - binary_img.shape[1]/2).astype(np.float32)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply rotation
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result
    return xpix_rotated, ypix_rotated

# Define a function to apply translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to map rover space pixels to world space
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))

    return warped

# Define a function to identify rocks
def find_rocks(img,Rock_thresh_low=(100,100,0),Rock_thresh_high=(255,255,55)):

    color_select = cv2.inRange(img,Rock_thresh_low,Rock_thresh_high)

    return color_select

def impose_range(xpix, ypix, range=80):
    dist = np.sqrt(xpix**2 + ypix**2)
    return xpix[dist < range], ypix[dist < range]


def divideConquer(image, source, destination,orgdist,organgle, Rover):
    image1 = np.zeros_like(image[:,:,:])
    image2 = np.zeros_like(image[:,:,:])
    image1[:,:160,:] += image[:,:160,:]
    image2[:,160:,:] += image[:,160:,:]
    warped1 = perspect_transform(image1, source, destination) #wraped is the bird-eye view perspective
    threshed1 = color_thresh(warped1)
    warped2 = perspect_transform(image2, source, destination) #wraped is the bird-eye view perspective
    threshed2 = color_thresh(warped2)
    xpix1, ypix1 = rover_coords(threshed1)
    dist1, angles1 = to_polar_coords(xpix1, ypix1)

    xpix2, ypix2 = rover_coords(threshed2)
    dist2, angles2 = to_polar_coords(xpix2, ypix2)

    magnitude1 = np.count_nonzero(threshed1)
    magnitude2 = np.count_nonzero(threshed2)

    if abs(np.mean(angles1)) < 0.5 and abs(np.mean(angles2)) < 0.5:
        ret_dist = orgdist
        ret_angle = organgle
        #return orgdist, organgle
    #elif abs(magnitude1-magnitude2) < 100:
     #   ret_dist = Rover.prev_angles[0]
      #  ret_angle = Rover.prev_angles[1]
    elif abs(np.mean(angles1)) > 0.6 and abs(np.mean(angles2)) < 0.4:
        ret_dist = orgdist
        ret_angle = organgle
    elif abs(np.mean(angles1)) < 0.4 and abs(np.mean(angles2)) > 0.6:
        ret_dist = orgdist
        ret_angle = organgle
    elif abs(magnitude1-magnitude2)<150:
        r = random.randint(1,2)
        if r == 1:
            ret_dist = dist1
            ret_angle = angles1
        else:
            ret_dist = dist2
            ret_angle = angles2
    elif magnitude1 > magnitude2:
        ret_dist = dist1
        ret_angle = angles1
        #return dist1, angles1
    else:
        ret_dist = dist2
        ret_angle = angles2
        #return dist2, angles2

    #Rover.prev_angles = [ret_dist, ret_angle]
    return ret_dist, ret_angle


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    image = np.zeros_like(Rover.img[:, :, :])
    image[80:, :, :] += Rover.img[80:, :, :]
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])

    destination = np.float32([[image.shape[1] / 2 - dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst_size, image.shape[0] - dst_size * 2 - bottom_offset],
                              [image.shape[1] / 2 - dst_size, image.shape[0] - dst_size * 2 - bottom_offset]
                              ])
    # 2) Apply perspective transform
    warped = perspect_transform(image, source, destination)


    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    #thresh_obs = np.abs(np.float32(thresh_navig)-1)*mask
    rock_threshed = find_rocks(warped)  # find rock samples
    
    path_threshed = color_thresh(warped)  # find only the navigatable path

    obs_threshed = 1 - color_thresh(warped, (85, 85, 85))
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:, :, 0] = obs_threshed
    Rover.vision_image[:, :, 1] = rock_threshed
    Rover.vision_image[:, :, 2] = path_threshed
    idx = np.nonzero(Rover.vision_image)
    Rover.vision_image[idx] = 255
    # 5) Convert map image pixel values to rover-centric coords
    xpix_obs, ypix_obs = rover_coords(Rover.vision_image[:, :, 0])
    xpix_rock, ypix_rock = rover_coords(Rover.vision_image[:, :, 1])
    xpix_path, ypix_path = rover_coords(Rover.vision_image[:, :, 2])
    # 6) Convert rover-centric pixel values to world coordinates
    scale = 2 * dst_size
    xpix_path, ypix_path = impose_range(xpix_path, ypix_path)
    xpix_obs, ypix_obs = impose_range(xpix_obs, ypix_obs)
    obs_x_world, obs_y_world = pix_to_world(xpix_obs, ypix_obs,
                                            Rover.pos[0], Rover.pos[1],
                                            Rover.yaw, Rover.worldmap.shape[0], scale)
    rock_x_world, rock_y_world = pix_to_world(xpix_rock, ypix_rock,
                                              Rover.pos[0], Rover.pos[1],
                                              Rover.yaw, Rover.worldmap.shape[0], scale)
    navigable_x_world, navigable_y_world = pix_to_world(xpix_path, ypix_path,
                                                        Rover.pos[0], Rover.pos[1],
                                                        Rover.yaw, Rover.worldmap.shape[0], scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    #Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    '''roll_limit = 0.6
    pitch_limit = 0.6
    -> fidelity 0.80
    ->mapping 0.92
    if (Rover.roll < roll_limit or Rover.roll > 360 - roll_limit) and (Rover.pitch < pitch_limit or Rover.pitch > 360 - pitch_limit):
        Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    '''

    roll_limit = 0.5  ## 3shan el medan ele fel nos 0.55
    pitch_limit = 0.5 ## 3shan el medan ele fel nos 0.55
    if (Rover.roll < roll_limit or Rover.roll > 360 - roll_limit) and (Rover.pitch < pitch_limit or Rover.pitch > 360 - pitch_limit):
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
        nav_pix = Rover.worldmap[:, :, 2] > 0
        Rover.worldmap[nav_pix, 0] = 0
        # clip to avoid overflow
        Rover.worldmap = np.clip(Rover.worldmap, 0, 255)
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles

	#    Rover.nav_dists = dist
	#    Rover.nav_angles = angles

    orgdist, organgle = to_polar_coords(xpix_path, ypix_path)
    Rover.nav_dists, Rover.nav_angles = divideConquer(image, source, destination, orgdist, organgle,Rover)
    Rover.rocks_dists, Rover.rocks_angles = to_polar_coords(xpix_rock, ypix_rock)

    return Rover


