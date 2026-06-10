# grid_builder.py — BLOCKS + BALLS VERSION
# Marks every block and ball in AirSim Blocks as an obstacle.
# Add your own by copying lines in the blocks/balls lists below.

import numpy as np
from config import (GRID_RESOLUTION, GRID_SIZE_X,
                    GRID_SIZE_Y, GRID_SIZE_Z, SAFETY_RADIUS)


def build_grid():
    nx = int(GRID_SIZE_X / GRID_RESOLUTION)
    ny = int(GRID_SIZE_Y / GRID_RESOLUTION)
    nz = int(GRID_SIZE_Z / GRID_RESOLUTION)
    grid = np.zeros((nx, ny, nz), dtype=float)

    # ── BLOCKS (cube shaped) ───────────────────────────────
    # Format: (x_center, y_center, height, width)
    # x_center = left/right in AirSim world (meters)
    # y_center = front/back in AirSim world (meters)
    # height   = how tall the block is
    # width    = how wide the block is
    #
    # TO ADD MORE: copy any line and change the numbers
    blocks = [
        # Row 1
        ( 8,  8, 10, 8),
        (22,  8, 14, 8),
        (36,  8, 10, 8),
        (50,  8, 12, 8),
        (64,  8, 10, 8),
        # Row 2
        ( 8, 22, 12, 8),
        (22, 22,  8, 8),
        (36, 22, 14, 8),
        (50, 22, 10, 8),
        (64, 22, 12, 8),
        # Row 3
        ( 8, 36, 10, 8),
        (22, 36, 12, 8),
        (36, 36,  8, 8),
        (50, 36, 14, 8),
        (64, 36, 10, 8),
        # Row 4
        ( 8, 50, 14, 8),
        (22, 50, 10, 8),
        (36, 50, 12, 8),
        (50, 50,  8, 8),
        (64, 50, 10, 8),
        # Row 5
        ( 8, 64, 10, 8),
        (22, 64, 14, 8),
        (36, 64, 10, 8),
        (50, 64, 12, 8),
        (64, 64,  8, 8),
        # ── ADD YOUR OWN BLOCKS HERE ──────────────────────
        # ( X,  Y, height, width),
    ]

    # ── BALLS (sphere shaped) ──────────────────────────────
    # Format: (x_center, y_center, z_center, radius)
    # radius = how big the ball is
    #
    # TO ADD MORE: copy any line and change the numbers
    balls = [
        (15, 15, 5, 4),
        (29, 15, 5, 4),
        (43, 15, 5, 4),
        (57, 15, 5, 4),
        (15, 29, 5, 4),
        (29, 29, 5, 4),
        (43, 29, 5, 4),
        (57, 29, 5, 4),
        (15, 43, 5, 4),
        (29, 43, 5, 4),
        (43, 43, 5, 4),
        (57, 43, 5, 4),
        (15, 57, 5, 4),
        (29, 57, 5, 4),
        (43, 57, 5, 4),
        (57, 57, 5, 4),
        # ── ADD YOUR OWN BALLS HERE ───────────────────────
        # ( X,  Y,  Z, radius),
    ]

    # Mark blocks in grid
    for (xc, yc, ht, wd) in blocks:
        h = wd / 2.0
        x1,x2 = max(0,xc-h), min(GRID_SIZE_X,xc+h)
        y1,y2 = max(0,yc-h), min(GRID_SIZE_Y,yc+h)
        z1,z2 = 0, min(GRID_SIZE_Z, ht)
        gx1,gx2 = int(x1/GRID_RESOLUTION), min(nx,int(x2/GRID_RESOLUTION)+1)
        gy1,gy2 = int(y1/GRID_RESOLUTION), min(ny,int(y2/GRID_RESOLUTION)+1)
        gz1,gz2 = int(z1/GRID_RESOLUTION), min(nz,int(z2/GRID_RESOLUTION)+1)
        grid[gx1:gx2, gy1:gy2, gz1:gz2] = 1.0

    # Mark balls in grid
    for (xc, yc, zc, r) in balls:
        x1,x2 = max(0,xc-r), min(GRID_SIZE_X,xc+r)
        y1,y2 = max(0,yc-r), min(GRID_SIZE_Y,yc+r)
        z1,z2 = max(0,zc-r), min(GRID_SIZE_Z,zc+r)
        gx1,gx2 = int(x1/GRID_RESOLUTION), min(nx,int(x2/GRID_RESOLUTION)+1)
        gy1,gy2 = int(y1/GRID_RESOLUTION), min(ny,int(y2/GRID_RESOLUTION)+1)
        gz1,gz2 = int(z1/GRID_RESOLUTION), min(nz,int(z2/GRID_RESOLUTION)+1)
        grid[gx1:gx2, gy1:gy2, gz1:gz2] = 1.0

    # Add safety buffer zones
    sc = int(SAFETY_RADIUS / GRID_RESOLUTION)
    danger = np.zeros_like(grid)
    for x in range(nx):
        for y in range(ny):
            for z in range(nz):
                if grid[x,y,z] >= 1.0:
                    continue
                md = sc + 1
                for dx in range(-sc, sc+1):
                    for dy in range(-sc, sc+1):
                        for dz in range(-sc, sc+1):
                            x2,y2,z2 = x+dx,y+dy,z+dz
                            if (0<=x2<nx and 0<=y2<ny and
                                0<=z2<nz and grid[x2,y2,z2]==1.0):
                                d = abs(dx)+abs(dy)+abs(dz)
                                if d < md: md = d
                if md <= sc:
                    danger[x,y,z] = 1.0-(md/sc)

    final = np.where(grid==1.0, 1.0, danger)

    print(f"\nGrid built!")
    print(f"  Blocks    : {len(blocks)}")
    print(f"  Balls     : {len(balls)}")
    print(f"  Obstacles : {int(np.sum(grid==1.0))} cells")
    print(f"  Danger    : {int(np.sum((final>0)&(final<1)))} cells")
    print(f"  Free      : {int(np.sum(final==0.0))} cells")
    return final


if __name__ == "__main__":
    g = build_grid()
    print(f"Shape: {g.shape}")
