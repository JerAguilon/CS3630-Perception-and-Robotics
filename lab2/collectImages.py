''' Take a picture

Takes most recent image from Cozmo
Converts it to 8-bit black and white
Saves to destination
'''

import sys
import cozmo
import datetime
import time

def run(sdk_conn):
    
    robot = sdk_conn.wait_for_robot()
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()

    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    myargs = sys.argv[1:]
    
    if len(myargs) <= 1:
        sys.exit("Incorrect arguments")
    
    num_images_per_type = int(myargs[0])  # number of images to take of each type of object
    
    print("Taking ", num_images_per_type, "images each of ", myargs[1:])

    for type in myargs[1:]:
        for i in range(num_images_per_type):
            time.sleep(.5)
            latest_image = robot.world.latest_image
            new_image = latest_image.raw_image

            robot.say_text(type + str(i)).wait_for_completed()

            timestamp = datetime.datetime.now().strftime("%dT%H%M%S%f")

            new_image.save("./imgs/" + str(type) + "_" + timestamp + ".bmp")

            time.sleep(4)

if __name__ == '__main__':
    cozmo.setup_basic_logging()

    try:
        cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
