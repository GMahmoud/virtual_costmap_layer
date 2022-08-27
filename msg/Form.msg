uint8 TYPE_CIRCLE=0
uint8 TYPE_LINESTRING=1
uint8 TYPE_RING=2
uint8 TYPE_POLYGON=3
uint8 type
string description      # Filled by the virtual costmap layer
string uuid             # Filled by the virtual costmap layer
string data             # wkt data 
duration lifetime       # How long the object should last before being automatically deleted. 0 means forever
