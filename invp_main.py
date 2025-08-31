import pygame
import invp_handler

# Initialize Pygame
pygame.init()
print("game initialized")

# Set window (canvas) size
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))

# Set window title
pygame.display.set_caption("My Pygame Canvas")

#init clock
clock = pygame.time.Clock()

# Main loop
running = True
refresh = 60
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:  # Close window with X
            running = False
        elif event.type == pygame.KEYDOWN:  # Check key press
            if event.key == pygame.K_ESCAPE:  # ESC key
                running = False

    # Fill canvas with a background color (RGB)
    screen.fill((0, 0, 0))
    
    refresh_rate = clock.tick(refresh)/1000
    
    invp_handler.runGame(screen=screen, refresh=refresh_rate)
    

    # Update the display (refresh)
    pygame.display.flip()

# Quit Pygame
pygame.quit()
