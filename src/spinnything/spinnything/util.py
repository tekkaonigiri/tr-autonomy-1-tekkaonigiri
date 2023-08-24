import curses

import numpy as np
from numpy.linalg import norm


def draw_circle_and_points(screen, point1: np.ndarray, point2: np.ndarray) -> None:
    SIZE = min(curses.COLS, curses.LINES)
    if (SIZE % 2 == 0):
        SIZE -= 1
    HALFWAY = (SIZE - 1) / 2
    CENTER = np.array([HALFWAY, HALFWAY])
    point1 = np.round(point1)
    point2 = np.round(point2)
    screen.erase()
    for i in range(SIZE):
        for j in range(SIZE):
            coord = np.array([i, j])
            if (np.isclose(point1, coord).all()):
                screen.addstr(i, j * 2, '╳╳')
            elif (np.isclose(point2, coord).all()):
                screen.addstr(i, j * 2, '%%')
            else:
                screen.addstr(i, j * 2, '┄┄' if norm(coord - CENTER) <= SIZE / 2 else '  ')
    screen.refresh()
