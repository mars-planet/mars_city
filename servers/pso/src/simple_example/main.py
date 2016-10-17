import pygame
import random
import json
import os
import re
from PyTango import *


# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)
RED      = ( 255,   0,   0)

# Connect to planner device via PyTango
planner= DeviceProxy("c3/waldo/pso")

def get_actions():
    return planner.get_actions()

def get_objects():
    return planner.get_objects()

def load_image(name, colorkey=None):
    try:
        image = pygame.image.load(name)
    except pygame.error, message:
        print 'Cannot load image:', name
        raise SystemExit, message
    image = image.convert()
    if colorkey is not None:
        if colorkey is -1:
            colorkey = image.get_at((0,0))
        image.set_colorkey(colorkey, 255)
    return image, image.get_rect()


actions = json.loads(get_actions())
print "EUROPA PLAN:",actions["Rover"],"\n"


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