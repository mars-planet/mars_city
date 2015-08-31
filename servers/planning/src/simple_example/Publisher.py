import random
import json
import os
import re
import time
import numpy
from PyTango import *
from PyTango import AttrQuality, AttrWriteType, DispLevel, DevState, DebugIt
from PyTango.server import Device, DeviceMeta, attribute, command
from PyTango.server import device_property

coordinates={"rock1":(1,1),"rock2":(3,3),"rock3":(2,2),"rock4":(9,9)}
# Connect to planner device via PyTango
planner= DeviceProxy("C3/pso/1")

def get_actions():
    return planner.get_actions()

def get_objects():
    return planner.get_objects()


actions = json.loads(get_actions())
print "EUROPA PLAN:",actions["Rover"],"\n"


def cal_co_ord(actions):
    for n, action in enumerate(actions["Rover"]):
        event = action["events"][0]
        print "PLAN EVENT CALLED:",event
        if ".Go" in event:
            destination=re.search("dest={(.*)[/(]", event).group(1)
            #print destination
            return(coordinates.get("{0}".format(destination)))




co_ordinates= cal_co_ord(actions)
print co_ordinates


class Publisher(Device):
    __metaclass__ = DeviceMeta
    POLLING = 30
    coordinates = attribute(label = "Destination co-ordinates",
                          dtype = (int,),
                          unit = "(meters, meters)",
                          access=AttrWriteType.READ,
                          polling_period = POLLING,
                          max_dim_x = 100,
                          max_dim_y = 100,
                          doc="An attribute for Linear and angular \
                               displacements")




    def init_device(self):
        Device.init_device(self)
        self.info_stream('In Python init_device method')
        self.set_state(PyTango.DevState.ON)

    self.push_change_event('coordinates', co_ordinates, 2)

if __name__ == "__main__":
    run([Publisher])













"""

#def cal_co_ord(actions):
        #step = (time.clock()-start_time)/1000
        #for n, action in enumerate(actions["Rover"]):
            #action_time=action["upper"][0] if type(action["upper"]
                #)==list else action["upper"]
            #if step==action_time:
                #event = action["events"][0]
                #print "PLAN EVENT CALLED:",event

                #if ".Go" in event:
                    #destination=re.search("dest={(.*)[/(]", event).group(1)
                    #print destination
                    #return(coordinates.get("{0}".format(destination)))
class Object(pygame.sprite.Sprite):

    moving = None

    def __init__(self, image, x, y):

        # Call the parent class (Sprite) constructor
        pygame.sprite.Sprite.__init__(self)

        self.image, self.rect = load_image(image, -1)
        self.rect.x = x
        self.rect.y = y

    def reset_pos(self):
        self.rect.y = random.randrange(-300, -20)
        self.rect.x = random.randrange(0, screen_width)

    def update(self):
        if self.moving:

            if self.rect.x>self.moving.rect.x:
                self.rect.x-=1
            else:
                self.rect.x+=1
            if self.rect.y>self.moving.rect.y:
                self.rect.y-=1
            else:
                self.rect.y+=1

            # Are we on the target object?
            if (self.rect.x==self.moving.rect.x and self.rect.y==self.moving.rect.y):
                self.moving=None


    def move_to(self, to):
        self.moving = to

class Rover(Object):

    def __init__(self, x ,y):
        Object.__init__(self, "rover.png", x, y)

class Rock(Object):

    def __init__(self, name, x ,y):
        self.name = name
        Object.__init__(self, "rock.png", x, y)


# Initialize Pygame
pygame.init()

# Set the height and width of the screen
screen_width = 700
screen_height = 400
screen = pygame.display.set_mode([screen_width, screen_height])

# This is a list of 'sprites.' Each rover in the program is
# added to this list. The list is managed by a class called 'Group.'
rock_list = pygame.sprite.Group()

# This is a list of every sprite. All rocks and the rover as well.
all_sprites_list = pygame.sprite.Group()


# Create a rover on screen
rover = Rover(0, 0)
all_sprites_list.add(rover)

# Create a new rock on screen
rock1 = Rock("rock1",90*4, 90*4)
rock_list.add(rock1)
all_sprites_list.add(rock1)

# Create a new rock on screen
rock2 = Rock("rock2",10*4, 60*4)
rock_list.add(rock2)
all_sprites_list.add(rock2)

# Create a new rock on screen
rock3 = Rock("rock3",40*4, 80*4)
rock_list.add(rock3)
all_sprites_list.add(rock3)

# Create a new rock on screen
rock4 = Rock("rock4",30*4, 90*4)
rock_list.add(rock4)
all_sprites_list.add(rock4)



#Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

score = 0

# -------- Main Program Loop -----------
while not done:

    step = pygame.time.get_ticks()/1000
    for n, action in enumerate(actions["Rover"]):
        action_time=action["upper"][0] if type(action["upper"]
            )==list else action["upper"]
        if step==action_time:
            event = action["events"][0]
            print "PLAN EVENT CALLED:",event

            if ".Go" in event:
                destination=re.search("dest={(.*)[/(]", event).group(1)
                print destination
                for sprite in list(rock_list):
                    if sprite.name==destination:
                        rover.move_to(sprite)
            # Don't need to execute plan event again
            del actions["Rover"][n]

    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done = True # Flag that we are done so we exit this loop

    # Clear the screen
    screen.fill(WHITE)

    # Calls update() method on every sprite in the list
    all_sprites_list.update()

    # See if the rover has collided with anything.
    blocks_hit_list = pygame.sprite.spritecollide(rover, rock_list, False)

    # Draw all the spites
    all_sprites_list.draw(screen)

    # Limit to 20 frames per second
    clock.tick(20)

    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

pygame.quit()
"""
