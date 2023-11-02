import pygame
import math


def GetDataFromArduino(line):
    # [:-3] get rid of end of line sign and additional comma separator that is sent from arduino
    data = line[:-3]
    # print(data)
    d_list = data.split(",")
    return d_list


def main():
    pygame.init()

    # constant based on lidar resolution
    LIDAR_RESOLUTION = 240

    # Set up the drawing window
    screen = pygame.display.set_mode([800, 800])
    sysfont = pygame.font.get_default_font()
    font1 = pygame.font.SysFont(sysfont, 72)

    file1 = open('test7.txt', 'r')
    Lines = file1.readlines()

    running = True
    delta = 360 / LIDAR_RESOLUTION
    theta = 0

    for line in Lines:
        if not running:
            break

        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    key_event = pygame.key.get_pressed()
                    if key_event[pygame.K_ESCAPE]:
                        running = False

            distances = GetDataFromArduino(line)
            if (len(distances) == LIDAR_RESOLUTION):
                # Did the user click the window close button?
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False

                # Fill the background with white
                screen.fill((250, 250, 250))

                start = int(0 / delta)
                end = int(360 / delta)
                for x in range(start, end, 1):
                    theta = delta * x
                    dist = int(distances[x]) / 6
                    c = (0, 0, 0)
                    r = 2
                    if x in [141, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 203, 204, 205, 206, 207]:
                        c = (255, 0, 0)
                        r = 3
                    x = dist * math.cos(math.radians(theta))
                    y = dist * math.sin(math.radians(theta))
                    pygame.draw.circle(screen, c, (400 + x, 400 + y), r)
                pygame.draw.circle(screen, (252, 132, 3), (400, 400), 12)

                # Flip the display
                pygame.display.flip()
                pygame.time.wait(100)
        except OSError as msg:
            print(msg)
        except KeyboardInterrupt:
            break
    pygame.quit()


if __name__ == "__main__":
    main()
