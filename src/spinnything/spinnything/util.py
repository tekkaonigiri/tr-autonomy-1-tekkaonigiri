import curses

import numpy as np
from numpy.linalg import norm


def draw_circle_and_points(screen, point1: np.ndarray, point2: np.ndarray) -> None:
    SCREENSIZE = min(int(curses.COLS / 2), curses.LINES)
    if SCREENSIZE % 2 == 0:
        SCREENSIZE -= 1
    HALFWAY = (SCREENSIZE - 1) / 2
    CENTER = np.array([HALFWAY, HALFWAY])

    CIRCLERADIUS = int(SCREENSIZE * .4)

    point1 = np.round(point1)
    point2 = np.round(point2)
    screen.erase()
    for i in range(SCREENSIZE):
        for j in range(SCREENSIZE):
            coord = np.array([i, j])
            if (np.isclose(point1, point2).all() and np.isclose(point1, coord).all()):
                screen.addstr(i, j * 2, '╳╳')
            elif (np.isclose(point1, coord).all()):
                screen.addstr(i, j * 2, '//')
            elif (np.isclose(point2, coord).all()):
                screen.addstr(i, j * 2, '\\\\')
            else:
                screen.addstr(i, j * 2, '┄┄' if norm(coord - CENTER) <= CIRCLERADIUS else '  ')
    screen.refresh()
