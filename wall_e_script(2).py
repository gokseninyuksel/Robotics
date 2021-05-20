import colorsys
import numpy as np
import sim
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import time
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

# FUNCTIONS TO INTERFACE WITH THE ROBOT
def compress():
    sim.simxSetIntegerSignal(clientID=clientID, signalName="compress", signalValue=1,
                             operationMode=sim.simx_opmode_blocking)


def set_speed(speed_l, speed_r):
    sim.simxSetStringSignal(clientID=clientID, signalName="motors", signalValue=sim.simxPackFloats([speed_l, speed_r]),
                            operationMode=sim.simx_opmode_blocking)


def get_battery():
    '''
   
    Returns the battery
    -------
    TYPE
        DESCRIPTION.

    '''
    return sim.simxGetStringSignal(clientID=clientID, signalName="battery",
                                   operationMode=sim.simx_opmode_blocking)


def get_bumper_sensor():
    # Bumper reading as 3-dimensional force vector
    # 
    bumper_force_vector = [0, 0, 0] # If there is a force, 
    return_code, bumper_force_vector_packed = sim.simxGetStringSignal(clientID=clientID, signalName="bumper_sensor",
                                                                       operationMode=sim.simx_opmode_blocking)
    
    
    if return_code == 0:
        bumper_force_vector = sim.simxUnpackFloats(bumper_force_vector_packed)
    return bumper_force_vector


def get_sonar_sensor():
    # Sonar reading as distance to closest object detected by it, -1 if no data
    sonar_dist = -1
    return_code, sonar_dist_packed = sim.simxGetStringSignal(clientID=clientID, signalName="sonar_sensor",
                                                             operationMode=sim.simx_opmode_blocking)
    if return_code == 0:
        sonar_dist = sim.simxUnpackFloats(sonar_dist_packed)
    return sonar_dist


def get_image_small_cam():
    # Image from the small camera
    return_code, return_value = sim.simxGetStringSignal(clientID=clientID, signalName="small_cam_image",
                                                        operationMode=sim.simx_opmode_blocking)
    if return_code == 0:
        image = sim.simxUnpackFloats(return_value)
        res = int(np.sqrt(len(image) / 3))
        return image_correction(image, res)
    else:
        return return_code


def get_image_top_cam():
    # Image from the top camera
    return_code, return_value = sim.simxGetStringSignal(clientID=clientID, signalName="top_cam_image",
                                                        operationMode=sim.simx_opmode_blocking)
    if return_code == 0:
        image = sim.simxUnpackFloats(return_value)
        res = int(np.sqrt(len(image) / 3))
        return image_correction(image, res)
    else:
        return return_code


# HELPER FUNCTIONS
def image_correction(image, res):
    """
    This function can be applied to images coming directly out of CoppeliaSim.
    It turns the 1-dimensional array into a more useful res*res*3 array, with the first
    two dimensions corresponding to the coordinates of a pixel and the third dimension to the
    RGB values. Aspect ratio of the image is assumed to be square (1x1).

    :param image: the image as a 1D array
    :param res: the resolution of the image, e.g. 64
    :return: an array of shape res*res*3
    """

    image = [int(x * 255) for x in image]
    image = np.array(image).reshape((res, res, 3))
    image = np.flip(m=image, axis=0)
    return image


def show_image(image):
    plt.imshow(image)
    plt.show()

# END OF FUNCTIONS

def object_detection(sonar):
    if ((sonar[0] > 0.465) & (sonar[0] < 0.480)):
            print("Nothing detected")
    elif ( sonar[0] != 0):
            print("Detected, distance: {}".format(sonar[0]))
    else:
            print("Crashed")
    


def find_yellow(image):
    '''
    Check for the array of values

    Parameters
    ----------
    image : TYPE
        DESCRIPTION.

    Returns
    -------
    list
        DESCRIPTION.

    '''
    image = image[:,32:35,:]
    image_Blue = image[:,:,2]
    image_Red = image[:,:,0]
    image_Green = image[:,:,1]
    Red_channel = np.arange(235,255)
    Blue_channel = np.arange(0,20)
    Green_channel = np.arange(235,255)
    x = np.isin(image_Red,Red_channel).any() and np.isin(image_Green,Green_channel).any() and np.isin(image_Blue,Blue_channel).any()
    return x

def stay_yellow(bottom_image):
    image = bottom_image[50:62,:,:]
    Red_channel = np.arange(235,255)
    Blue_channel = np.arange(0,20)
    Green_channel = np.arange(235,255)
    image_Blue = image[:,:,2]
    image_Red = image[:,:,0]
    image_Green = image[:,:,1]
    if np.isin(image_Red,Red_channel).any() and np.isin(image_Green,Green_channel).any() and np.isin(image_Blue,Blue_channel).any():
        return True
    else:
        return False
        
def stay(battery_level,threshold):
    battery = float(get_battery()[1].decode())
    if(battery > battery_level[-1]):
        return True
    else:
        return False

def on_the_wall(image_bottom,image_top):
    left_image = image_bottom[:, :32 ,:]
    right_image = image_bottom[:,32:,:]
    Red_channel = np.arange(120,145)
    Green_channel = np.arange(120,145)
    Blue_channel = np.arange(128,180)
    gray_values_left = left_image
    gray_values_right = right_image
    isGray_left = np.isin(gray_values_left[:,:,0],Red_channel).all() and np.isin(gray_values_left[:,:,1],Green_channel).all() and np.isin(gray_values_left[:,:,2],Blue_channel).all()
    isGray_right = np.isin(gray_values_right[:,:,0],Red_channel).all() and np.isin(gray_values_right[:,:,1],Green_channel).all() and np.isin(gray_values_right[:,:,2],Blue_channel).all()
    wall_top_cam = False
    count_Red_channel = 0  
    if ( (np.all(image_bottom == image_bottom[0])) and (not isGray_left) and (not isGray_right)):
        count_Red_channel = 0
        for a in image_top[:,:,0].flatten():
            if a in Red_channel:
                count_Red_channel = count_Red_channel + 1
        
        count_Green_channel = 0
        for a in image_top[:,:,1].flatten():
            if a in Green_channel:
                count_Green_channel = count_Green_channel + 1
        
        count_Blue_channel = 0
        for a in image_top[:,:,2].flatten():
            if a in Blue_channel:
                count_Blue_channel = count_Blue_channel + 1
    
        average = (count_Red_channel + count_Blue_channel + count_Red_channel) / 3 
        percentage = average / (64 * 64)
        if(percentage > 0.58 and percentage < 0.65):
            wall_top_cam = True
   
    return wall_top_cam | isGray_left | isGray_right


def seek(image_bottom,look):
    image_center = image_bottom[:,32:33,:]
    if look:
        red,green = on_the_box(image_center,look)
        
    else:
        red,green = on_the_box(image_bottom,look)
    foundCenter = red or green
    return [foundCenter,red,green]
        

def on_the_box(image_bottom,look):
    Red_channel_brown = np.arange(80,150)
    Green_channel_brown = np.arange(30,70)
    Blue_channel_brown = np.arange(0,50)
    Red_channel_green = np.arange(20,80)
    Green_channel_green = np.arange(120,255)
    Blue_channel_green = np.arange(0,50)
    Red_channel_image = image_bottom[:,:,0] 
    Green_channel_image = image_bottom[:,:,1]
    Blue_channel_image = image_bottom[:,:,2]
    if look:
        all_in_Red = np.isin(Red_channel_image,Red_channel_brown).any() and np.isin(Green_channel_image,Green_channel_brown).any() and np.isin(Blue_channel_image,Blue_channel_brown).any()
        all_in_Green = np.isin(Red_channel_image,Red_channel_green).any() and np.isin(Green_channel_image,Green_channel_green).any() and np.isin(Blue_channel_image,Blue_channel_green).any()
        
    else:
        all_in_Red = np.isin(Red_channel_image,Red_channel_brown).all() and np.isin(Green_channel_image,Green_channel_brown).all() and np.isin(Blue_channel_image,Blue_channel_brown).all()
        all_in_Green = np.isin(Red_channel_image,Red_channel_green).all() and np.isin(Green_channel_image,Green_channel_green).all() and np.isin(Blue_channel_image,Blue_channel_green).all()
    return [all_in_Red,all_in_Green]

def findRed(image_top):
    image_center = image_top[:,32:36,:]
    red_channel = np.arange(122,125)
    blue_green_channel = np.arange(0,3)
    rgb = np.transpose([np.tile(red_channel, len(blue_green_channel)), np.repeat(blue_green_channel, len(red_channel))])
    rgb = np.stack((rgb,rgb,rgb)).reshape(-1,2)
    z = np.arange(0,3)
    z = np.repeat(z,9).reshape(-1,1)
    show_image(image_center)
    rgb = np.append(rgb, z, axis=1)
    detected = False
    for a in rgb:
        detected = detected or (image_center == a).all(axis = 2).any(axis = 1).any()
    return detected            
def findBlue(image_top):
    image_center = image_top[:,32:36,:]
    show_image(image_center)
    red_channel = np.arange(0,5)
    green_channel = np.arange(110,120)
    blue_channel = np.arange(140,145)
    detected = np.isin(image_center[:,:,0], red_channel).any() and np.isin(image_center[:,:,1], green_channel).any() and np.isin(image_center[:,:,2], blue_channel).any()
    return detected    
    
    
# MAIN CONTROL LOOP
if clientID != -1:
    print('Connected')
    base_level = 0.465
    prev_battery = []
    stage = "Explore"
    while True:
        battery_level = float(get_battery()[1].decode())
        sonar = get_sonar_sensor()
        #object_detection(sonar)
        image_top = get_image_top_cam()
        image_bottom = get_image_small_cam()
        threshold = 0.999
        threshold_low = 0.5
        
        if (stage == "Explore"):
            set_speed(0,20)
            found = seek(image_top, True)[0]
            if(found):
                stage = "Box_center"
                print("Box in the center")
            

                
            
        if(battery_level < threshold_low):
            prev_battery.append(battery_level)
            stage = "battery_low"
        
            if (not find_yellow(image_top)):
                set_speed(0,20)
            elif(find_yellow(image_top) and not stay_yellow(image_bottom)):
                set_speed(80,80)
            elif (stay(prev_battery,threshold)):
                set_speed(0,0)
        elif(battery_level >= threshold):
            stage = "Explore"
        
        if(on_the_wall(image_bottom,image_top) or stage == "On_the_wall"):
            stage = "On_the_wall"
            set_speed(-20,-20)
            if(not on_the_wall(image_bottom,image_top)):
                print("Exploring")
                stage = "Explore"
        if(stage == "Box_center"):
            print("Box in the center goinngggg")
            set_speed(40,40)
            red = seek(image_bottom,False)[1]
            green = seek(image_bottom,False)[2]
            print(found,red,green)

            if(red):
                time.sleep(1.5)
                stage = "Red_box"
                findRed(image_top)
            elif(green):
                time.sleep(1.5)
                stage = "Green_box"
                findBlue(image_top)
        if(stage == "Red_box"):
            found = findRed(image_top)
            if(found):
                set_speed(40,40)
            else:
                set_speed(0,20)

        if(stage == "Green_box"):
            found = findBlue(image_top)
            if(found):
                set_speed(40,40)
            else:
                set_speed(0,20)
        
        
                

                
        
        
            
            
            

        # if(stage == "Box_center" and stage != "Battery_low"): 
        #         set_speed(50,50)
        #         print("Box_center")
        #         if(seek(image_bottom, False)[0]):
        #             time.sleep(1.5)
        #             print("Setting 0 ")
        #             set_speed(0,0)
        #             stage = "Box_found"
        #             red, green = seek(image_bottom, False)[1],seek(image_bottom, False)[2]
        #         else:
        #             print("Box is not near now")
        
        # if(stage == "Box_found"):
        #     print(red)
        #     print("Box found")
        #     set_speed(0,20)
        #     if(red):
        #         findRed(image_top)
                
        #     else:
        #         findBlue(image_top)
            
      
        # if(on_the_wall(image_bottom,image_top)):
        #     stage = "On_the_wall"
        #     print("On the wall")
        #     set_speed(-20,-20)
        #     time.sleep(1)
        #     if( not on_the_wall(image_bottom,image_top)):
        #         stage = "Explore"
        
        # if(battery_level < 0.7 and stage != "On_the_wall"):
        #     stage = "Battery_low"
        
        # if (stage == "Battery_low"):
        #     prev_battery.append(battery_level)
        #     if(find_yellow(image_top) and not stay(prev_battery,threshold)):
        #         set_speed(50,50)
        #     elif(stay(prev_battery,threshold)):
        #         set_speed(0,0)
        #     elif (battery_level >= threshold):
        #             stage = "Explore"
        #     else:
        #         set_speed(0,20)

                
        
        # if (stage == "Explore"):
        #       set_speed(0,10)  

        #       if(stage != 'Box_found' and stage != "Box_center"):
        #          found = seek(image_top, True)[0]
        #          if(found and (stage != "On_the_wall" and stage != "Battery_low")):
        #              stage = "Box_center"
              

                
        
    
        

    # End connection
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
