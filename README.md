# virtual_costmap_layer

## Overview
This package includes a ROS1 plugin to add a virtual layer of geometric elements in the 2D costmap. 
The user can define several forms based on `well-known text representation of geometry` ([link](https://en.wikipedia.org/wiki/Well-known_text_representation_of_geometry) for more details) in the map frame.   

## Generated message and service

This project generate some new custom messages and services:

### Form message 
This is the main message used in the communication with the package, it has 4 possible type: 
   
   * LINESTRING
   * RING
   * POLYGON
   * CIRCLE : :warning: __NOT SUPPORTED YET__ :warning:

These form are represented with wkt data. Here is some example:

```
# A linestring
- LINESTRING (3 1, 1 3, 4 4)
   
# A ring (M letter filled)
- POLYGON ((), (1 1, 1 5, 2.5 3.5, 4 5, 4 1, 3.5 1, 3.5 4, 2.5 3, 1.5 4, 1.5 1))

# A polygon (M letter empty)
- POLYGON ((1 1, 1 5, 2.5 3.5, 4 5, 4 1, 3.5 1, 3.5 4, 2.5 3, 1.5 4, 1.5 1))

# A polygon (Random polygon with one inner)
- POLYGON ((3.5 1, 4.5 4.5, -1.5 4, -1 -2, 3.5 -1), (2 3, 1.5 2.5, 1 1, 1 3))

```

Please note that a rings are specific case of polygon in wkt data. A ploygon contains an outer ring and multiple inner rings. In our package, an element in the virtual layer that has a ring format is filled. If you need to define a hallow ring form, you may use a polygon with no inner (see the empty M letter example)    

<div align=right>
<table>
  <tr>
    <td><img src="./demo/linestring.png" alt="LineString" width="400"/></a></td>
    <td><img src="./demo/polygon_one_inner.png" alt="Polygon (One inner)" width="400"/></a></td>
  </tr>
</table>
</div>

<div align=right>
<table>
    <tr>
    <td><img src="./demo/polygon_m_letter.png" alt="Polygon (M letter)" width="400"/></a></td>
    <td><img src="./demo/ring_m_letter.png" alt="Ring (M letter)" width="400"/></a></td>
  </tr>
</table>
</div>

## ROS1 Node API

### Services

