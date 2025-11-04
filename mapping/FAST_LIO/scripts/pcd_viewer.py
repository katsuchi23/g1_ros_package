#!/usr/bin/env python3
"""
Simple PCD Viewer using Open3D
Usage: python3 pcd_viewer.py <path_to_pcd_file>
"""

import sys
import os

# Force X11 backend for GLFW (fixes Wayland issues)
os.environ['DISPLAY'] = os.environ.get('DISPLAY', ':0')
os.environ['XDG_SESSION_TYPE'] = 'x11'

import open3d as o3d


def visualize_pcd(pcd_file):
    """
    Load and visualize a PCD file
    
    Args:
        pcd_file: Path to the PCD file
    """
    if not os.path.exists(pcd_file):
        print(f"Error: File '{pcd_file}' not found!")
        return False
    
    print(f"Loading point cloud from: {pcd_file}")
    
    try:
        # Load the point cloud
        pcd = o3d.io.read_point_cloud(pcd_file)
        
        # Print point cloud information
        print(f"Point cloud loaded successfully!")
        print(f"Number of points: {len(pcd.points)}")
        print(f"Has colors: {pcd.has_colors()}")
        print(f"Has normals: {pcd.has_normals()}")
        
        # Visualize
        print("\nControls:")
        print("  - Mouse left button: Rotate")
        print("  - Mouse right button: Translate")
        print("  - Mouse wheel: Zoom")
        print("  - Press 'Q' or close window to exit")
        print("\nOpening visualizer...")
        
        o3d.visualization.draw_geometries(
            [pcd],
            window_name="PCD Viewer",
            width=1024,
            height=768,
            left=50,
            top=50,
            point_show_normal=False,
            mesh_show_wireframe=False,
            mesh_show_back_face=False
        )
        
        return True
        
    except Exception as e:
        print(f"Error loading or visualizing point cloud: {e}")
        return False


def main():
    """Main function"""
    if len(sys.argv) < 2:
        print("Usage: python3 pcd_viewer.py <path_to_pcd_file>")
        print("\nExample:")
        print("  python3 pcd_viewer.py ../PCD/scans.pcd")
        sys.exit(1)
    
    pcd_file = sys.argv[1]
    
    # If relative path, make it absolute based on script location
    if not os.path.isabs(pcd_file):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pcd_file = os.path.join(script_dir, pcd_file)
    
    success = visualize_pcd(pcd_file)
    
    if not success:
        sys.exit(1)


if __name__ == "__main__":
    main()
