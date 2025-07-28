#!/usr/bin/env python3
"""
Generate PCD file from static obstacle information
Based on the actual obstacle positions and sizes from the debug output
"""

import numpy as np
import math

def generate_box_voxels(position, size, resolution=0.1):
    """Generate voxel points for a box obstacle"""
    x_center, y_center, z_bottom = position  # z_bottom is the bottom of the box
    x_size, y_size, z_size = size
    
    # Calculate actual center (box position is at bottom center)
    z_center = z_bottom + z_size / 2.0
    
    # Calculate half sizes
    x_half = x_size / 2.0
    y_half = y_size / 2.0
    z_half = z_size / 2.0
    
    # Generate voxel grid - use smaller range to avoid oversized voxels
    voxels = []
    
    # Calculate the range of voxels to generate (slightly inside the boundaries)
    x_min = x_center - x_half + resolution/2
    x_max = x_center + x_half - resolution/2
    y_min = y_center - y_half + resolution/2
    y_max = y_center + y_half - resolution/2
    z_min = z_center - z_half + resolution/2
    z_max = z_center + z_half - resolution/2
    
    # Generate voxel points
    x = x_min
    while x <= x_max:
        y = y_min
        while y <= y_max:
            z = z_min
            while z <= z_max:
                voxels.append([x, y, z])
                z += resolution
            y += resolution
        x += resolution
    
    return voxels

def generate_wall_voxels(position, size, rotation, resolution=0.1):
    """Generate voxel points for a wall with rotation"""
    x_center, y_center, z_center = position
    x_size, y_size, z_size = size
    roll, pitch, yaw = rotation
    
    # Calculate half sizes
    x_half = x_size / 2.0
    y_half = y_size / 2.0
    z_half = z_size / 2.0
    
    # Generate local voxel grid (before rotation) - use smaller range
    local_voxels = []
    
    # Local coordinate ranges (slightly inside the boundaries)
    x_min = -x_half + resolution/2
    x_max = x_half - resolution/2
    y_min = -y_half + resolution/2
    y_max = y_half - resolution/2
    z_min = -z_half + resolution/2
    z_max = z_half - resolution/2
    
    # Generate local voxel points
    x = x_min
    while x <= x_max:
        y = y_min
        while y <= y_max:
            z = z_min
            while z <= z_max:
                local_voxels.append([x, y, z])
                z += resolution
            y += resolution
        x += resolution
    
    # Apply rotation and translation
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    
    world_voxels = []
    for local_point in local_voxels:
        x_local, y_local, z_local = local_point
        
        # Apply rotation (only yaw for walls)
        x_rotated = x_local * cos_yaw - y_local * sin_yaw
        y_rotated = x_local * sin_yaw + y_local * cos_yaw
        z_rotated = z_local
        
        # Apply translation
        x_world = x_rotated + x_center
        y_world = y_rotated + y_center
        z_world = z_rotated + z_center
        
        world_voxels.append([x_world, y_world, z_world])
    
    return world_voxels

def main():
    # Resolution for voxel generation
    resolution = 0.1
    
    # Define all static obstacles
    all_voxels = []
    
    print("Generating static obstacle voxels...")
    
    # Corridor walls - 使用debug输出的精确位置
    print("Processing corridor walls...")
    corridor_walls = [
        {
            'name': 'Wall_0',
            'position': (20.000, 2.500, 1.250),
            'size': (41.15, 0.15, 2.5),
            'rotation': (0.000, 0.000, 0.000)
        },
        {
            'name': 'Wall_2', 
            'position': (20.000, -2.500, 1.250),
            'size': (41.15, 0.15, 2.5),
            'rotation': (0.000, 0.000, 0.000)
        },
        {
            'name': 'Wall_4',
            'position': (-0.500, 0.000, 1.250),
            'size': (5.15, 0.15, 2.5),
            'rotation': (0.000, 0.000, -1.571)
        },
        {
            'name': 'Wall_6',
            'position': (40.500, 0.000, 1.250),
            'size': (5.15, 0.15, 2.5),
            'rotation': (0.000, 0.000, -1.571)
        }
    ]
    
    for wall in corridor_walls:
        print(f"  Generating {wall['name']}...")
        wall_voxels = generate_wall_voxels(wall['position'], wall['size'], wall['rotation'], resolution)
        all_voxels.extend(wall_voxels)
        print(f"    Added {len(wall_voxels)} voxels")
    
    # Box obstacles - 位置是底部中心，需要调整Z坐标
    
    print(f"\nTotal voxels generated: {len(all_voxels)}")
    
    # Calculate bounds
    if all_voxels:
        all_voxels_array = np.array(all_voxels)
        min_bounds = np.min(all_voxels_array, axis=0)
        max_bounds = np.max(all_voxels_array, axis=0)
        
        print(f"Voxel bounds:")
        print(f"  X: [{min_bounds[0]:.3f}, {max_bounds[0]:.3f}]")
        print(f"  Y: [{min_bounds[1]:.3f}, {max_bounds[1]:.3f}]")
        print(f"  Z: [{min_bounds[2]:.3f}, {max_bounds[2]:.3f}]")
        
        map_size_x = max_bounds[0] - min_bounds[0]
        map_size_y = max_bounds[1] - min_bounds[1]
        map_size_z = max_bounds[2] - min_bounds[2]
        print(f"Map size: {map_size_x:.1f}m × {map_size_y:.1f}m × {map_size_z:.1f}m")
    
    # Write PCD file
    output_file = "corridor_static_obstacles_fixed1.pcd"
    print(f"\nWriting PCD file: {output_file}")
    
    with open(output_file, 'w') as f:
        # Write PCD header
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(all_voxels)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(all_voxels)}\n")
        f.write("DATA ascii\n")
        
        # Write voxel data
        for voxel in all_voxels:
            f.write(f"{voxel[0]:.3f} {voxel[1]:.3f} {voxel[2]:.3f}\n")
    
    print(f"PCD file saved successfully!")
    print(f"Generated {len(all_voxels)} voxel points")
    
    # Summary by obstacle type
    wall_count = sum(len(generate_wall_voxels(wall['position'], wall['size'], wall['rotation'], resolution)) 
                    for wall in corridor_walls)
    
    
    print(f"\nBreakdown:")
    print(f"  Corridor walls: {wall_count} voxels")


if __name__ == "__main__":
    main() 