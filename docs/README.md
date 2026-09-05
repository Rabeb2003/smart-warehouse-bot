# Documentation

This directory contains supplementary documentation and visual assets for the ROS2 Autonomous Warehouse Robot project.

## Images

Place screenshots and visualizations in the `images/` subdirectory:

### Recommended Screenshots

1. **warehouse_map.png**: SLAM map showing warehouse layout and navigation paths
2. **gazebo_simulation.png**: Gazebo simulation window with robot and ArUco markers
3. **navigation_rviz.png**: RViz2 visualization showing costmaps, robot pose, and AMCL particles
4. **aruco_detection.png**: OpenCV window showing ArUco marker detection with bounding boxes
5. **mission_progress.png**: Terminal output showing mission execution progress

### Adding Screenshots

```bash
# Take screenshots during simulation
# Then copy to docs/images/
cp /path/to/screenshot.png docs/images/warehouse_map.png
```

### Image Guidelines

- Use PNG format for best quality
- Keep file sizes under 500KB each
- Ensure text and UI elements are readable
- Include relevant context (robot, markers, paths)

## Architecture

See the main README.md for system architecture diagram.

## Additional Documentation

- [Main README](../README.md) - Project overview and setup instructions
- [License](../LICENSE) - MIT License details
