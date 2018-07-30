# virtual_costmap_layer

This package includes a plugin to add a virtual layer of obstacles and to define a navigation zone in the 2D costmap. 
The user can define several geometric forms (point, line, circle, polygon) in the map frame.   

1. **Point**: geometry_msgs::Point [x, y, z=0]
2. **Line**:  2 * geometry_msgs::Point[x, y, z=0]
3. **Circle**: geometry_msgs::Point[x, y, z>0]
4. **Polygon with n edges**: n * geometry_msgs::Point[x, y, z=0]

The plugin subscribes to different topics for zone and for obstacles to receive data. It is also possible to define all those forms in the config YAML file.

![Presentation](/demo/presentation.gif "Presentation")



