#ifndef VIRTUAL_COSTMAP_LAYER_VIRTUAL_LAYER_H_
#define VIRTUAL_COSTMAP_LAYER_VIRTUAL_LAYER_H_

#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <tf/transform_datatypes.h>
#include <mutex>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <unordered_map>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <custom_msgs/Zone.h>
#include <custom_msgs/Obstacles.h>
#include "virtual_costmap_layer/VirtualLayerConfig.h"

namespace virtual_costmap_layer
{

// point with integer coordinates
struct PointInt
{
  int x;
  int y;
};

/*    
* @class VirtualLayer
* @brief A class that creates of virtual obstacles and defines zone of navigation in a costmap layer.
*/
class VirtualLayer : public costmap_2d::Layer
{
public:
  /**
   * @brief default constructor
   */
  VirtualLayer();

  /**
   * @brief destructor
   */
  virtual ~VirtualLayer();
  /**
   * @brief function which get called at initialization of the costmap
   * it defines the reconfige callback, gets params from server and subscribes to topics
   */
  virtual void onInitialize();

  /**
   * @brief This is called by the LayeredCostmap to poll this plugin
   * as to how much of the costmap it needs to update.
   * Each layer can increase the size of this bounds. 
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double *min_x, double *min_y, double *max_x, double *max_y);

  /**
   * @brief function which get called at every cost updating procdure
   * of the overlayed costmap. The before readed costs will get
   * filled
   */
  virtual void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
                           int max_i, int max_j);

private:
  /**
   * @brief overlayed reconfigure callback function
   */
  void reconfigureCB(VirtualLayerConfig &config, uint32_t level);

  /**
   * @brief Computes bounds in world coordinates for the current set of points and polygons.
   * The result is stored in class members _min_x, _min_y, _max_x and _max_y.
   */
  void computeMapBounds();

  /**
   * @brief Set cost in a Costmap2D for a polygon (polygon may be located outside bounds)
   * 
   * @param master_grid    reference to the Costmap2D object
   * @param polygon        polygon defined by a vector of points (in world coordinates)
   * @param cost           the cost value to be set (0,255)
   * @param min_i          minimum bound on the horizontal map index/coordinate
   * @param min_j          minimum bound on the vertical map index/coordinate
   * @param max_i          maximum bound on the horizontal map index/coordinate
   * @param max_j          maximum bound on the vertical map index/coordinate
   * @param fill_polygon   if true, tue cost for the interior of the polygon will be set as well    
   */
  void setPolygonCost(costmap_2d::Costmap2D &master_grid, const std::vector<geometry_msgs::Point> &polygon,
                      unsigned char cost, int min_i, int min_j, int max_i, int max_j, bool fill_polygon);

  /**
   * @brief Converts polygon (in map coordinates) to a set of cells in the map
   * 
   * @remarks This method is mainly based on Costmap2D::convexFillCells() but accounts
   *          for a self-implemented polygonOutlineCells() method and allows negative map coordinates
   * 
   * @param polygon             polygon defined  by a vector of map coordinates
   * @param fill                if true, the interior of the polygon will be considered as well
   * @param[out] polygon_cells  new cells in map coordinates are pushed back on this container
   */
  void rasterizePolygon(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells, bool fill);

  /**
   * @brief Extracts the boundary of a polygon in terms of map cells
   * 
   * @remarks This method is based on Costmap2D::polygonOutlineCells() but accounts
   *          for a self-implemented raytrace algorithm and allows negative map coordinates
   * 
   * @param polygon             polygon defined  by a vector of map coordinates
   * @param[out] polygon_cells  new cells in map coordinates are pushed back on this container
   */
  void polygonOutlineCells(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells);

  /**
   * @brief Rasterizes line between two map coordinates into a set of cells
   * 
   * @remarks Since Costmap2D::raytraceLine() is based on the size_x and since we want to rasterize 
   *          polygons that might also be located outside map bounds we provide a modified raytrace
   *          implementation (also Bresenham) based on the integer version presented here:
   *          http://playtechs.blogspot.de/2007/03/raytracing-on-grid.html
   * 
   * @param x0          line start x-coordinate (map frame)
   * @param y0          line start y-coordinate (map frame)
   * @param x1          line end x-coordinate (map frame)
   * @param y1          line end y-coordinate (map frame)
   * @param[out] cells  new cells in map coordinates are pushed back on this container
   */
  void raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells);

  /**
   * @brief reads the topic names in YAML-Format from the
   * ROS parameter server in the namespace of this
   * plugin
   * e.g. /etom_navigation/global_costmap/virtual_layer/zone_topics
   *
   * @param nh          pointer to the rosnode handle
   * @param param       name of the parameter where the topic names are saved in 
   *                    YAML file
   * 
   * @return bool       true if the parsing was successful
   *                    false if it wasn't
   */
  bool parseTopicsFromYaml(ros::NodeHandle *nh, const std::string &param);

  /**
   * @brief zone callback function
   */
  void zoneCallback(const custom_msgs::ZoneConstPtr &msg);

  /**
   * @brief obstacle callback function
   */
  void obstaclesCallback(const custom_msgs::ObstaclesConstPtr &obstacles_msg);

  /**
   * @brief checks if the robot point is in the polygon to define as zone
   * 
   * @param ponts          list of the points
   *
   * @remarks works only for one zone otherwise returns true
   * 
   * @return bool       true if the robot is in the zone
   *                    false if it isn't
   */
  bool robotInZone(std::vector<geometry_msgs::Point> points);

  /**
   * @brief reads the forms in YAML-Format from the
   * ROS parameter server in the namespace of this plugin
   * 
   * @param nh          pointer to the ros-Node handle
   * @param param       name of the parameter where the
   *                    forms saved in YAML format
   *
   * @return bool       true if the parsing was successful
   *                    false if it wasn't
   */
  bool parseFormListFromYaml(ros::NodeHandle *nh, const std::string &param);

  /**
 * @brief gets a geometry_msgs::Point from a YAML-Array
 * The z-coordinate get always written to zero!
 *
 * @param val         YAML-array with to point-coordinates (x and y)
 * @param point       variable where the determined point get saved
 *
 * @return bool       true if the determining was successful
 *                    false if it wasn't
 */
  bool getPoint(XmlRpc::XmlRpcValue &val, geometry_msgs::Point &point);

  /**
 * @brief gets a current geometry_msgs::Point of the robot
 *
 * @return Geometry_msgs::Point      current pose
 */
  geometry_msgs::Point getRobotPoint();

  std::string tag_;
  dynamic_reconfigure::Server<VirtualLayerConfig> *dsrv_;            //!< dynamic_reconfigure server for the costmap
  std::mutex data_mutex_;                                            //!< mutex for the accessing forms
  double costmap_resolution_;                                        //!< resolution of the overlayed costmap to create the thinnest line out of two points
  bool one_zone_, clear_obstacles_;                                  //!< put in memory previous zones and obstacles if false
  std::string base_frame_;                                           //!< base frame of the robot by default "base_link"
  std::string map_frame_;                                            //!< map frame by default "map"
  std::vector<geometry_msgs::Point> obstacle_points_;                //!< vector to save the obstacle points in source coordinates
  std::vector<std::vector<geometry_msgs::Point>> zone_polygons_;     //!< vector to save the zone polygons (more than 3 edges) in source coordinates
  std::vector<std::vector<geometry_msgs::Point>> obstacle_polygons_; //!< vector to save the obstacle polygons (including lines) in source coordinates
  std::vector<std::vector<geometry_msgs::Point>> form_polygons_;     //!< vector to save the form polygons (including lines) in source coordinates
  std::vector<geometry_msgs::Point> form_points_;                    //!< vector to save the form points in source coordinates
  double _min_x, _min_y, _max_x, _max_y;                             //!< cached map bounds
  std::vector<ros::Subscriber> subs_;                                //!< vector to save all ros subscribers
};
} // namespace virtual_costmap_layer

#endif //VIRTUAL_COSTAMP_LAYER_VIRTUAL_LAYER_H_