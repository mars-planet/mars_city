import pygame
import random
import json
import os
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
print actions["Rover"]


class Object(pygame.sprite.Sprite):
    """
    This class represents the ball        
    It derives from the "Sprite" class in Pygame
    """
    def __init__(self, image, x, y):
        """ Constructor. Pass in the color of the block, 
        and its x and y position. """
        # Call the parent class (Sprite) constructor
        pygame.sprite.Sprite.__init__(self) 
  
        self.image, self.rect = load_image(image, -1)
        self.rect.x = x
        self.rect.y = y

    def reset_pos(self):
        """ Reset position to the top of the screen, at a random x location.
        Called by update() or the main program loop if there is a collision.
        """
        self.rect.y = random.randrange(-300, -20)
        self.rect.x = random.randrange(0, screen_width)        
 
class Rover(Object):
    """ The player class derives from Object, but overrides the 'update' 
    functionality with new a movement function that will move the block 
    with the mouse. """

    def __init__(self, x ,y):
        Object.__init__(self, "rover.png", x, y) 

    def update(self):
        # Get the current mouse position. This returns the position
        # as a list of two numbers.
        pos = pygame.mouse.get_pos()
          
        self.rect.x = 0
        self.rect.y = 0        
 
class Rock(Object):
    """ The player class derives from Object, but overrides the 'update' 
    functionality with new a movement function that will move the block 
    with the mouse. """

    def __init__(self, x ,y):
        Object.__init__(self, "rock.png", x, y) 
 

# Initialize Pygame
pygame.init()
  
# Set the height and width of the screen
screen_width = 700
screen_height = 400
screen = pygame.display.set_mode([screen_width, screen_height])
  
# This is a list of 'sprites.' Each block in the program is
# added to this list. The list is managed by a class called 'Group.'
block_list = pygame.sprite.Group()
  
# This is a list of every sprite. All blocks and the player block as well.
all_sprites_list = pygame.sprite.Group()
  
for i in range(4):
  
    # Set a random location for the block
    x = random.randrange(screen_width)
    y = random.randrange(screen_height)

    # Create a new rock on screen      
    rock = Rock(x, y)

    # Add the rock to the list of objects on screen
    block_list.add(block)
    all_sprites_list.add(block)
      
      
  
# Create a rover on screen
player = Rover(0, 0)
all_sprites_list.add(player)
  
#Loop until the user clicks the close button.
done = False
  
# Used to manage how fast the screen updates
clock = pygame.time.Clock()
  
score = 0
  
# -------- Main Program Loop -----------
while not done:
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done = True # Flag that we are done so we exit this loop
  
    # Clear the screen
    screen.fill(WHITE)
     
    # Calls update() method on every sprite in the list
    all_sprites_list.update()
      
    # See if the rover has collided with anything.
    blocks_hit_list = pygame.sprite.spritecollide(player, block_list, False)  

    # Draw all the spites
    all_sprites_list.draw(screen)
      
    # Limit to 20 frames per second
    clock.tick(20)
  
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()
  
pygame.quit()