#include "virtual_costmap_layer/VirtualLayer.h"

PLUGINLIB_EXPORT_CLASS(virtual_costmap_layer::VirtualLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace virtual_costmap_layer
{

VirtualLayer::VirtualLayer() : dsrv_(NULL), tag_("[ VIRTUAL_LAYER ] ")
{
}

VirtualLayer::~VirtualLayer()
{
  if (dsrv_ != NULL)
    delete dsrv_;
}

void VirtualLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  one_zone_ = true;
  clear_obstacles_ = true;
  dsrv_ = new dynamic_reconfigure::Server<VirtualLayerConfig>(nh);
  dynamic_reconfigure::Server<VirtualLayerConfig>::CallbackType cb =
      boost::bind(&VirtualLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  // get a pointer to the layered costmap and save resolution
  costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
  costmap_resolution_ = costmap->getResolution();

  // set initial bounds
  _min_x = _min_y = _max_x = _max_y = 0;

// reading the defined topics out of the namespace of this plugin!
  std::string topics_param = "zone_topics";
  if (!parseTopicsFromYaml(&nh, topics_param))
    ROS_ERROR_STREAM(tag_ << "Reading topic names from '" << nh.getNamespace() << "/" << topics_param << "' failed!");
  topics_param = "obstacle_topics";
  if (!parseTopicsFromYaml(&nh, topics_param))
    ROS_ERROR_STREAM(tag_ << "Reading topic names from '" << nh.getNamespace() << "/" << topics_param << "' failed!");

  // reading the defined forms out of the namespace of this plugin!
  std::string params = "forms";
  if (!parseProhibitionListFromYaml(&nh, params))
    ROS_ERROR_STREAM(tag_ << "Reading prohibition areas from '" << nh.getNamespace() << "/" << params << "' failed!");

  // compute map bounds for the current set of prohibition areas.
  computeMapBounds();

  ROS_INFO_STREAM(tag_ << "VirtualLayer initialized.");
}

void VirtualLayer::reconfigureCB(VirtualLayerConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  one_zone_ = config.one_zone;
  clear_obstacles_ = config.clear_obstacles;
}

void VirtualLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                double *min_x, double *min_y, double *max_x, double *max_y)
{
  if (!enabled_)
  {
    return;
  }

  std::lock_guard<std::mutex> l(data_mutex_);

  if (obstacle_points_.empty() && zone_polygons_.empty() && obstacle_polygons_.empty())
  {
    return;
  }

  *min_x = std::min(*min_x, _min_x);
  *min_y = std::min(*min_y, _min_y);
  *max_x = std::max(*max_x, _max_x);
  *max_y = std::max(*max_y, _max_y);
}

void VirtualLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
  {
    return;
  }

  std::lock_guard<std::mutex> l(data_mutex_);

  // set costs of zone polygons
  for (int i = 0; i < zone_polygons_.size(); ++i)
  {
    setPolygonCost(master_grid, zone_polygons_[i], LETHAL_OBSTACLE, min_i, min_j, max_i, max_j, false);
  }

  // set costs of obstacle polygons
  for (int i = 0; i < obstacle_polygons_.size(); ++i)
  {
    setPolygonCost(master_grid, obstacle_polygons_[i], LETHAL_OBSTACLE, min_i, min_j, max_i, max_j, true);
  }

  // set cost of obstacle points
  for (int i = 0; i < obstacle_points_.size(); ++i)
  {
    unsigned int mx;
    unsigned int my;
    if (master_grid.worldToMap(obstacle_points_[i].x, obstacle_points_[i].y, mx, my))
    {
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
  }
}

void VirtualLayer::computeMapBounds()
{
  std::lock_guard<std::mutex> l(data_mutex_);

  // reset bounds
  _min_x = _min_y = _max_x = _max_y = 0;

  // iterate zone polygons
  for (int i = 0; i < zone_polygons_.size(); ++i)
  {
    for (int j = 0; j < zone_polygons_.at(i).size(); ++j)
    {
      double px = zone_polygons_.at(i).at(j).x;
      double py = zone_polygons_.at(i).at(j).y;
      _min_x = std::min(px, _min_x);
      _min_y = std::min(py, _min_y);
      _max_x = std::max(px, _max_x);
      _max_y = std::max(py, _max_y);
    }
  }

  // iterate obstacle polygons
  for (int i = 0; i < obstacle_polygons_.size(); ++i)
  {
    for (int j = 0; j < obstacle_polygons_.at(i).size(); ++j)
    {
      double px = obstacle_polygons_.at(i).at(j).x;
      double py = obstacle_polygons_.at(i).at(j).y;
      _min_x = std::min(px, _min_x);
      _min_y = std::min(py, _min_y);
      _max_x = std::max(px, _max_x);
      _max_y = std::max(py, _max_y);
    }
  }

  // iterate obstacle points
  for (int i = 0; i < obstacle_points_.size(); ++i)
  {
    double px = obstacle_points_.at(i).x;
    double py = obstacle_points_.at(i).y;
    _min_x = std::min(px, _min_x);
    _min_y = std::min(py, _min_y);
    _max_x = std::max(px, _max_x);
    _max_y = std::max(py, _max_y);
  }
}

void VirtualLayer::setPolygonCost(costmap_2d::Costmap2D &master_grid, const std::vector<geometry_msgs::Point> &polygon, unsigned char cost,
                                  int min_i, int min_j, int max_i, int max_j, bool fill_polygon)
{
  std::vector<PointInt> map_polygon;
  for (unsigned int i = 0; i < polygon.size(); ++i)
  {
    PointInt loc;
    master_grid.worldToMapNoBounds(polygon[i].x, polygon[i].y, loc.x, loc.y);
    map_polygon.push_back(loc);
  }

  std::vector<PointInt> polygon_cells;

  // get the cells that fill the polygon
  rasterizePolygon(map_polygon, polygon_cells, fill_polygon);

  // set the cost of those cells
  for (unsigned int i = 0; i < polygon_cells.size(); ++i)
  {
    int mx = polygon_cells[i].x;
    int my = polygon_cells[i].y;
    // check if point is outside bounds
    if (mx < min_i || mx >= max_i)
      continue;
    if (my < min_j || my >= max_j)
      continue;
    master_grid.setCost(mx, my, cost);
  }
}

void VirtualLayer::polygonOutlineCells(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells)
{
  for (unsigned int i = 0; i < polygon.size() - 1; ++i)
  {
    raytrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
  }
  if (!polygon.empty())
  {
    unsigned int last_index = polygon.size() - 1;
    // we also need to close the polygon by going from the last point to the first
    raytrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
  }
}

void VirtualLayer::raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells)
{
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  PointInt pt;
  pt.x = x0;
  pt.y = y0;
  int n = 1 + dx + dy;
  int x_inc = (x1 > x0) ? 1 : -1;
  int y_inc = (y1 > y0) ? 1 : -1;
  int error = dx - dy;
  dx *= 2;
  dy *= 2;

  for (; n > 0; --n)
  {
    cells.push_back(pt);

    if (error > 0)
    {
      pt.x += x_inc;
      error -= dy;
    }
    else
    {
      pt.y += y_inc;
      error += dx;
    }
  }
}

void VirtualLayer::rasterizePolygon(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells, bool fill)
{
  // this implementation is a slighly modified version of Costmap2D::convexFillCells(...)

  //we need a minimum polygon of a traingle
  if (polygon.size() < 3)
    return;

  //first get the cells that make up the outline of the polygon
  polygonOutlineCells(polygon, polygon_cells);

  if (!fill)
    return;

  //quick bubble sort to sort points by x
  PointInt swap;
  unsigned int i = 0;
  while (i < polygon_cells.size() - 1)
  {
    if (polygon_cells[i].x > polygon_cells[i + 1].x)
    {
      swap = polygon_cells[i];
      polygon_cells[i] = polygon_cells[i + 1];
      polygon_cells[i + 1] = swap;

      if (i > 0)
        --i;
    }
    else
      ++i;
  }

  i = 0;
  PointInt min_pt;
  PointInt max_pt;
  int min_x = polygon_cells[0].x;
  int max_x = polygon_cells[(int)polygon_cells.size() - 1].x;

  //walk through each column and mark cells inside the polygon
  for (int x = min_x; x <= max_x; ++x)
  {
    if (i >= (int)polygon_cells.size() - 1)
      break;

    if (polygon_cells[i].y < polygon_cells[i + 1].y)
    {
      min_pt = polygon_cells[i];
      max_pt = polygon_cells[i + 1];
    }
    else
    {
      min_pt = polygon_cells[i + 1];
      max_pt = polygon_cells[i];
    }

    i += 2;
    while (i < polygon_cells.size() && polygon_cells[i].x == x)
    {
      if (polygon_cells[i].y < min_pt.y)
        min_pt = polygon_cells[i];
      else if (polygon_cells[i].y > max_pt.y)
        max_pt = polygon_cells[i];
      ++i;
    }

    PointInt pt;
    //loop though cells in the column
    for (int y = min_pt.y; y < max_pt.y; ++y)
    {
      pt.x = x;
      pt.y = y;
      polygon_cells.push_back(pt);
    }
  }
}

bool VirtualLayer::parseTopicsFromYaml(ros::NodeHandle *nh, const std::string &param)
{
  std::lock_guard<std::mutex> l(data_mutex_);
  XmlRpc::XmlRpcValue param_yaml;
  if (nh->getParam(param, param_yaml))
  {

    if ((param_yaml.valid() == false) || (param_yaml.getType() != XmlRpc::XmlRpcValue::TypeArray))
    {
      ROS_ERROR_STREAM(tag_ << "Invalid topic names list: it must be a non-empty list of strings");
      return false;
    }
    if (param_yaml.size() < 1)
    {
      ROS_WARN_STREAM(tag_ << "Empty topic names list: virtual layer will have no effect on costmap");
    }

    for (unsigned int i = 0; i < param_yaml.size(); i++)
    {
      if (param_yaml[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_WARN_STREAM(tag_ << "Invalid topic names list: element " << i << " is not a string, so it will be ignored");
      }
      else
      {
        std::string topic_name(param_yaml[i]);
        if ((topic_name.size() > 0) && (topic_name.at(0) != '/'))
        {
          ROS_WARN_STREAM(tag_ << "Topic name " << topic_name << " should start with / sperator");
          topic_name = "/" + topic_name;
        }
        ROS_WARN_STREAM(tag_ << "Topic name " << topic_name);

        //callback_ = boost::bind(&VirtualLayer::processMsg, this, _1);
        if (param == "zone_topics")
        {
          subs_.push_back(nh->subscribe(topic_name, 100, &VirtualLayer::zoneCallback, this));
        }
        else if (param == "obstacle_topics")
        {
          subs_.push_back(nh->subscribe(topic_name, 100, &VirtualLayer::obstacleCallback, this));
        }

        ROS_INFO_STREAM(tag_ << "Subscribed to topic " << subs_.back().getTopic().c_str());
      }
    }
  }
  else
  {
    ROS_ERROR_STREAM(tag_ << "Cannot read " << param << " from parameter server");
    return false;
  }
  return true;
}

void VirtualLayer::zoneCallback(const custom_msgs::ZoneConstPtr &zone_msg)
{
  if (zone_msg->area.form.size() > 2)
  {
    std::vector<geometry_msgs::Point> vector_to_add;
    for (int i = 0; i < zone_msg->area.form.size(); ++i)
    {
      vector_to_add.push_back(zone_msg->area.form[i]);
    }
    if (one_zone_)
    {
      zone_polygons_.clear();
    }
    zone_polygons_.push_back(vector_to_add);

    computeMapBounds();
  }
  else
  {
    ROS_ERROR_STREAM(tag_ << "A zone Layer needs to be a polygon with minimun 3 edges");
  }
}

void VirtualLayer::obstacleCallback(const custom_msgs::ObstacleConstPtr &obstacle_msg)
{
  if (clear_obstacles_)
  {
    obstacle_polygons_.clear();
    obstacle_points_.clear();
  }

  for (int i = 0; i < obstacle_msg->list.size(); ++i)
  {
    std::vector<geometry_msgs::Point> vector_to_add;
    if (obstacle_msg->list[i].form.size() == 1)
    {
      ROS_INFO_STREAM(tag_ << "Adding a Point");
      obstacle_points_.push_back(obstacle_msg->list[i].form[0]);
    }
    else if (obstacle_msg->list[i].form.size() == 2)
    {
      ROS_INFO_STREAM(tag_ << "Adding a Line");

      geometry_msgs::Point point_A = obstacle_msg->list[i].form[0];
      geometry_msgs::Point point_B = obstacle_msg->list[i].form[1];
      vector_to_add.push_back(point_A);
      vector_to_add.push_back(point_B);

      // calculate the normal vector for AB
      geometry_msgs::Point point_N;
      point_N.x = point_B.y - point_A.y;
      point_N.y = point_A.x - point_B.x;

      // get the absolute value of N to normalize and get
      // it to the length of the costmap resolution
      double abs_N = sqrt(pow(point_N.x, 2) + pow(point_N.y, 2));
      point_N.x = point_N.x / abs_N * costmap_resolution_;
      point_N.y = point_N.y / abs_N * costmap_resolution_;

      // calculate the new points to get a polygon which can be filled
      geometry_msgs::Point point;
      point.x = point_A.x + point_N.x;
      point.y = point_A.y + point_N.y;
      vector_to_add.push_back(point);

      point.x = point_B.x + point_N.x;
      point.y = point_B.y + point_N.y;
      vector_to_add.push_back(point);

      obstacle_polygons_.push_back(vector_to_add);
    }
    else
    {
      ROS_INFO_STREAM(tag_ << "Adding a Polygon");
      for (int j = 0; j < obstacle_msg->list[i].form.size(); ++j)
      {
        vector_to_add.push_back(obstacle_msg->list[i].form[j]);
      }
      obstacle_polygons_.push_back(vector_to_add);
    }
  }
}

// load polygones, lines and points out of the rosparam server
bool VirtualLayer::parseProhibitionListFromYaml(ros::NodeHandle *nh, const std::string &param)
{
  std::lock_guard<std::mutex> l(data_mutex_);
  std::unordered_map<std::string, geometry_msgs::Pose> map_out;

  XmlRpc::XmlRpcValue param_yaml;

  bool ret_val = true;

  if (nh->getParam(param, param_yaml))
  {
    if (param_yaml.getType() == XmlRpc::XmlRpcValue::TypeArray) // list of goals
    {
      for (int i = 0; i < param_yaml.size(); ++i)
      {
        if (param_yaml[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          std::vector<geometry_msgs::Point> vector_to_add;

          /* **************************************
           * differ between points and polygons
           * lines get to a polygon with the resolution
           * of the costmap
           **************************************** */

          // add a point
          if (param_yaml[i].size() == 1)
          {
            geometry_msgs::Point point;
            ret_val = getPoint(param_yaml[i][0], point);
            _prohibition_points.push_back(point);
          }
          // add a line
          else if (param_yaml[i].size() == 2)
          {
            if (param_yaml[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                param_yaml[i][0].getType() == XmlRpc::XmlRpcValue::TypeInt)
            {
              // add a lonely point
              geometry_msgs::Point point;
              ret_val = getPoint(param_yaml[i], point);
              _prohibition_points.push_back(point);
            }
            else
            {
              // add a line!
              geometry_msgs::Point point_A;
              ret_val = getPoint(param_yaml[i][0], point_A);
              vector_to_add.push_back(point_A);

              geometry_msgs::Point point_B;
              ret_val = getPoint(param_yaml[i][1], point_B);
              vector_to_add.push_back(point_B);

              // calculate the normal vector for AB
              geometry_msgs::Point point_N;
              point_N.x = point_B.y - point_A.y;
              point_N.y = point_A.x - point_B.x;

              // get the absolute value of N to normalize and get
              // it to the length of the costmap resolution
              double abs_N = sqrt(pow(point_N.x, 2) + pow(point_N.y, 2));
              point_N.x = point_N.x / abs_N * costmap_resolution_;
              point_N.y = point_N.y / abs_N * costmap_resolution_;

              // calculate the new points to get a polygon which can be filled
              geometry_msgs::Point point;
              point.x = point_A.x + point_N.x;
              point.y = point_A.y + point_N.y;
              vector_to_add.push_back(point);

              point.x = point_B.x + point_N.x;
              point.y = point_B.y + point_N.y;
              vector_to_add.push_back(point);

              _prohibition_polygons.push_back(vector_to_add);
            }
          }
          // add a point or add a polygon
          else if (param_yaml[i].size() >= 3)
          {
            // add a polygon with any number of points
            for (int j = 0; j < param_yaml[i].size(); ++j)
            {
              geometry_msgs::Point point;
              ret_val = getPoint(param_yaml[i][j], point);
              vector_to_add.push_back(point);
            }
            _prohibition_polygons.push_back(vector_to_add);
          }
        }
        else
        {
          ROS_ERROR_STREAM(tag_ << param << " with index " << i << " is not correct.");
          ret_val = false;
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM(tag_ << param << "struct is not correct.");
      ret_val = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM(tag_ << "Cannot read " << param << " from parameter server");
    ret_val = false;
  }
  return ret_val;
}

// get a point out of the XML Type into a geometry_msgs::Point
bool VirtualLayer::getPoint(XmlRpc::XmlRpcValue &val, geometry_msgs::Point &point)
{
  try
  {
    // check if there a two values for the coordinate
    if (val.getType() == XmlRpc::XmlRpcValue::TypeArray && val.size() == 2)
    {
      auto convDouble = [](XmlRpc::XmlRpcValue &val) -> double {
        if (val.getType() == XmlRpc::XmlRpcValue::TypeInt) // XmlRpc cannot cast int to double
          return int(val);
        return val; // if not double, an exception is thrown;
      };

      point.x = convDouble(val[0]);
      point.y = convDouble(val[1]);
      point.z = 0.0;
      return true;
    }
    else
    {
      ROS_ERROR_STREAM(tag_ << "A point has to consist two values!");
      return false;
    }
  }
  catch (const XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR_STREAM(tag_ << "Cannot add current point: " << ex.getMessage());
    return false;
  }
}

} // namespace virtual_costmap_layer