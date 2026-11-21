"""Generacion de trayectorias de referencia (listas de waypoints)."""
import math

import numpy as np


def figure8(a: float, b: float, n: int = 400) -> np.ndarray:
    """Lemniscata de Gerono: x = a sin t, y = b sin 2t (cruza en el origen)."""
    t = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return np.column_stack((a * np.sin(t), b * np.sin(2.0 * t)))


def oval(straight: float, radius: float, n: int = 400) -> np.ndarray:
    """Circuito ovalado (estadio) que arranca en el origen hacia +x."""
    perim = 2.0 * straight + 2.0 * math.pi * radius
    s = np.linspace(0.0, perim, n, endpoint=False)
    pts = []
    for si in s:
        if si < straight:                                   # recta inferior
            pts.append((si, 0.0))
        elif si < straight + math.pi * radius:              # curva derecha
            th = (si - straight) / radius - math.pi / 2.0
            pts.append((straight + radius * math.cos(th), radius + radius * math.sin(th)))
        elif si < 2.0 * straight + math.pi * radius:        # recta superior
            pts.append((straight - (si - straight - math.pi * radius), 2.0 * radius))
        else:                                               # curva izquierda
            th = (si - 2.0 * straight - math.pi * radius) / radius + math.pi / 2.0
            pts.append((radius * math.cos(th), radius + radius * math.sin(th)))
    return np.array(pts)


def resample(points: np.ndarray, step: float, closed: bool) -> np.ndarray:
    """Re-muestrea la poligonal con separacion constante `step` [m]."""
    pts = np.vstack((points, points[:1])) if closed else points
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    s_new = np.arange(0.0, s[-1], step)
    out = np.column_stack((np.interp(s_new, s, pts[:, 0]), np.interp(s_new, s, pts[:, 1])))
    return out


def align_to_start(points: np.ndarray, x0=0.0, y0=0.0, yaw0=0.0) -> np.ndarray:
    """Traslada/rota la trayectoria para que empiece en (x0, y0) con rumbo yaw0."""
    p = points - points[0]
    d = p[min(3, len(p) - 1)]
    rot = yaw0 - math.atan2(d[1], d[0])
    c, s = math.cos(rot), math.sin(rot)
    p = p @ np.array([[c, s], [-s, c]])
    return p + np.array([x0, y0])


def headings(points: np.ndarray) -> np.ndarray:
    """Rumbo tangente de cada waypoint."""
    d = np.gradient(points, axis=0)
    return np.arctan2(d[:, 1], d[:, 0])


def max_curvature(points: np.ndarray) -> float:
    """Curvatura maxima (1/m) de una poligonal muestreada uniformemente."""
    dx, dy = np.gradient(points[:, 0]), np.gradient(points[:, 1])
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    k = np.abs(dx * ddy - dy * ddx) / np.power(dx * dx + dy * dy, 1.5)
    return float(np.max(k[2:-2]))
