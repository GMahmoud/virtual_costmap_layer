# virtual_costmap_layer

This package includes a plugin to add a virtual layer of obstacles and to define a navigation zone in the 2D costmap. 
The user can define several geometric forms (point, line, circle, polygon) in the map frame.   

1. **Point**: geometry_msgs::Point [x, y, z=0]
2. **Line**:  2 * geometry_msgs::Point[x, y, z=0]
3. **Circle**: geometry_msgs::Point[x, y, z>0]
4. **Polygon with n edges**: n * geometry_msgs::Point[x, y, z=0]

The plugin subscribes to different topics for zone and for obstacles to receive data. It is also possible to define all those forms in the config YAML file.

# Examples
## Point
```
rostopic pub /virtual_costamp_layer/obsctacles custom_msgs/Obstacles "list: [form: [{x: 2.0, y: 1.0, z: 0.0}]]"
```
## Line
```
rostopic pub /virtual_costamp_layer/obsctacles custom_msgs/Obstacles "list: [form: [{x: 1.0, y: 1.0, z: 0.0}, {x: 2.0, y: 1.0, z: 0.0}]]"
```
## Circle
```
rostopic pub /virtual_costamp_layer/obsctacles custom_msgs/Obstacles "list: [form: [{x: 1.0, y: 1.0, z: 1.0}]]"
```
## Polygon
```
rostopic pub /virtual_costamp_layer/obsctacles custom_msgs/Obstacles "list: [form: [{x: 1.0, y: 1.0, z: 0.0}, {x: 2.0, y: 1.0, z: 0.0}, {x: 1.0, y: 2.0, z: 0.0}, {x: 2.0, y: 2.0, z: 0.0}]]"
```
## Multiple
```
rostopic pub /virtual_costamp_layer/obsctacles custom_msgs/Obstacles "list: [form: [{x: -1.0, y: 1.0, z: 1.0}], form: [{x: 1.0, y: 1.0, z: 0.0}, {x: 2.0, y: 1.0, z: 0.0}, {x: 1.0, y: 2.0, z: 0.0}, {x: 2.0, y: 2.0, z: 0.0}]]"
```


![Presentation](/demo/presentation.gif "Presentation")



