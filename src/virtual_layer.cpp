#include "virtual_costmap_layer/virtual_layer.hpp"

#include <geometry_msgs/Pose.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>

#include <XmlRpcException.h>

PLUGINLIB_EXPORT_CLASS(virtual_costmap_layer::VirtualLayer, costmap_2d::Layer);

static const std::string tag {"[VIRTUAL-LAYER] "};

namespace virtual_costmap_layer {

// ---------------------------------------------------------------------

VirtualLayer::VirtualLayer() :
    _dsrv(nullptr)
{}

// ---------------------------------------------------------------------

VirtualLayer::~VirtualLayer()
{
    if (_dsrv) {
        _dsrv = nullptr;
    }
}

// ---------------------------------------------------------------------

void VirtualLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);
    current_ = true;

    _base_frame = "base_link";
    _map_frame = "map";
    _one_zone_mode = true;
    _clear_obstacles = true;

    _dsrv = std::make_shared<dynamic_reconfigure::Server<VirtualLayerConfig>>(nh);

    dynamic_reconfigure::Server<VirtualLayerConfig>::CallbackType cb = boost::bind(&VirtualLayer::reconfigureCb, this, _1, _2);
    _dsrv->setCallback(cb);

    // save resolution
    _costmap_resolution = layered_costmap_->getCostmap()->getResolution();

    // set initial bounds
    _min_x = _min_y = _max_x = _max_y = 0;

    // reading the defined topics out of the namespace of this plugin!
    std::string param {"zone_topics"};
    parseTopicsFromYaml(nh, param);
    param = "obstacle_topics";
    parseTopicsFromYaml(nh, param);

    // reading the defined forms out of the namespace of this plugin!
    param = "forms";
    parseFormListFromYaml(nh, param);

    // compute map bounds for the current set of areas and obstacles.
    computeMapBounds();

    ROS_INFO_STREAM(tag << "layer is initialized: [points: " << _form_points.size() << "] [polygons: " << _form_polygons.size() << "]");
}

// ---------------------------------------------------------------------

void VirtualLayer::parseTopicsFromYaml(ros::NodeHandle &nh, const std::string &param)
{
    XmlRpc::XmlRpcValue param_yaml;
    if (nh.getParam(param, param_yaml)) {
        if ((param_yaml.valid() == false) || (param_yaml.getType() != XmlRpc::XmlRpcValue::TypeArray)) {
            ROS_ERROR_STREAM(tag << "invalid topic names list: it must be a non-empty list of strings");
            throw std::runtime_error("invalid topic names list: it must be a non-empty list of strings");
        }

        if (param_yaml.size() == 0) {
            ROS_WARN_STREAM(tag << "empty topic names list: virtual layer will have no effect on costmap");
        }

        for (std::size_t i = 0; i < param_yaml.size(); ++i) {
            if (param_yaml[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_WARN_STREAM(tag << "invalid topic names list: element " << i << " is not a string, so it will be ignored");
            } else {
                std::string topic_name(param_yaml[i]);
                if ((topic_name.empty()) && (topic_name.at(0) != '/')) {
                    topic_name = "/" + topic_name;
                }

                if (param.compare("zone_topics") == 0) {
                    _subs.push_back(nh.subscribe(topic_name, 100, &VirtualLayer::zoneCallback, this));
                } else if (param.compare("obstacle_topics") == 0) {
                    _subs.push_back(nh.subscribe(topic_name, 100, &VirtualLayer::obstaclesCallback, this));
                }

                ROS_INFO_STREAM(tag << "subscribed to topic " << _subs.back().getTopic().c_str());
            }
        }
    } else {
        ROS_ERROR_STREAM(tag << "cannot read " << param << " from parameter server");
    }
}

// ---------------------------------------------------------------------

// load polygones, lines and points out of the rosparam server
void VirtualLayer::parseFormListFromYaml(const ros::NodeHandle &nh, const std::string &param)
{
    XmlRpc::XmlRpcValue param_yaml;
    if (nh.getParam(param, param_yaml)) {
        if (param_yaml.getType() == XmlRpc::XmlRpcValue::TypeArray) {

            for (std::size_t i = 0; i < param_yaml.size(); ++i) {
                geometry_msgs::Point point;
                Polygon vector_to_add;
                if (param_yaml[i].getType() == XmlRpc::XmlRpcValue::TypeArray) {
                    if (param_yaml[i].size() == 1) { // add a point
                        try {
                            convert(param_yaml[i][0], point);
                        } catch (...) {
                            continue;
                        }
                        _form_points.push_back(point);
                    } else if (param_yaml[i].size() == 2) {
                        if (param_yaml[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                            param_yaml[i][0].getType() == XmlRpc::XmlRpcValue::TypeInt) { // add a point
                            try {
                                convert(param_yaml[i], point);
                            } catch (...) {
                                continue;
                            }
                            _form_points.push_back(point);
                        } else { // add a line
                            vector_to_add.reserve(3);
                            geometry_msgs::Point a;
                            geometry_msgs::Point b;
                            try {
                                convert(param_yaml[i][0], a);
                                convert(param_yaml[i][1], b);
                            } catch (...) {
                                continue;
                            }
                            vector_to_add.push_back(a);
                            vector_to_add.push_back(b);

                            // calculate the normal vector for AB
                            geometry_msgs::Point n;
                            n.x = b.y - a.y;
                            n.y = a.x - b.x;
                            // get the absolute value of N to normalize it and to set the length of the costmap resolution
                            double abs_n = sqrt(pow(n.x, 2) + pow(n.y, 2));
                            n.x = n.x / abs_n * _costmap_resolution;
                            n.y = n.y / abs_n * _costmap_resolution;

                            // calculate the new points to get a polygon which can be filled
                            point.x = a.x + n.x;
                            point.y = a.y + n.y;
                            vector_to_add.push_back(point);

                            point.x = b.x + n.x;
                            point.y = b.y + n.y;
                            vector_to_add.push_back(point);

                            _form_polygons.push_back(vector_to_add);
                        }
                    } else if (param_yaml[i].size() >= 3) { // add a polygon
                        vector_to_add.reserve(param_yaml[i].size());
                        for (std::size_t j = 0; j < param_yaml[i].size(); ++j) {
                            try {
                                convert(param_yaml[i][j], point);
                            } catch (...) {
                                vector_to_add.clear();
                                break;
                            }
                            vector_to_add.push_back(point);
                        }
                        if (!vector_to_add.empty()) {
                            _form_polygons.push_back(vector_to_add);
                        }
                    }
                } else {
                    ROS_ERROR_STREAM(tag << param << " with index #" << i << " is corrupted");
                }
            }

        } else {
            ROS_ERROR_STREAM(tag << param << "struct is corrupted");
        }

    } else {
        ROS_ERROR_STREAM(tag << "could not read " << param << " from parameter server");
    }
}

// ---------------------------------------------------------------------

// get a point out of the XML Type into a geometry_msgs::Point
void VirtualLayer::convert(const XmlRpc::XmlRpcValue &val, geometry_msgs::Point &point)
{
    try {
        // check if there a two values for the coordinate
        if (val.getType() == XmlRpc::XmlRpcValue::TypeArray && val.size() == 2) {
            auto convDouble = [](const XmlRpc::XmlRpcValue &val) -> double {
                auto val_copy = val;
                if (val_copy.getType() == XmlRpc::XmlRpcValue::TypeInt) // XmlRpc cannot cast int to double
                {

                    return int(val_copy);
                }
                return val_copy; // if not double, an exception is thrown;
            };

            point.x = convDouble(val[0]);
            point.y = convDouble(val[1]);
            point.z = 0.0;
        } else {
            ROS_ERROR_STREAM(tag << "a point has to contain two double values");
            throw std::runtime_error("a point has to contain two double values");
        }
    } catch (const XmlRpc::XmlRpcException &ex) {
        ROS_ERROR_STREAM(tag << "could not convert point: [" << ex.getMessage() << "]");
        throw std::runtime_error("could not convert point: [" + ex.getMessage() + "]");
    }
}

// ---------------------------------------------------------------------

bool VirtualLayer::robotInZone(const Polygon &zone)
{
    if (!_one_zone_mode) {
        ROS_WARN_STREAM(tag << "could be applied only for one_zone_mode");
        return true;
    }

    geometry_msgs::Point point = getRobotPoint();
    std::size_t i, j;
    std::size_t size = zone.size();
    bool result = false;

    for (i = 0, j = size - 1; i < size; j = ++i) {
        if (((zone[i].y > point.y) != (zone[j].y > point.y)) &&
            (point.x < (zone[j].x - zone[i].x) * (point.y - zone[i].y) / (zone[j].y - zone[i].y) + zone[i].x)) {
            result = !result;
        }
    }

    return result;
}

// ---------------------------------------------------------------------

void VirtualLayer::reconfigureCb(VirtualLayerConfig &config, uint32_t level)
{
    enabled_ = config.enabled;
    _one_zone_mode = config.one_zone;
    _clear_obstacles = config.clear_obstacles;
    _base_frame = config.base_frame;
    _map_frame = config.map_frame;
}

void VirtualLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                double *min_x, double *min_y, double *max_x, double *max_y)
{
    if (!enabled_) {
        return;
    }

    std::lock_guard<std::mutex> l(_data_mutex);

    if (_obstacle_points.empty() && _zone_polygons.empty() && _obstacle_polygons.empty()) {
        return;
    }

    *min_x = std::min(*min_x, _min_x);
    *min_y = std::min(*min_y, _min_y);
    *max_x = std::max(*max_x, _max_x);
    *max_y = std::max(*max_y, _max_y);
}

void VirtualLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_) {
        return;
    }

    std::lock_guard<std::mutex> l(_data_mutex);

    // set costs of zone polygons
    for (int i = 0; i < _zone_polygons.size(); ++i) {
        setPolygonCost(master_grid, _zone_polygons[i], costmap_2d::LETHAL_OBSTACLE, min_i, min_j, max_i, max_j, false);
    }

    // set costs of obstacle polygons
    for (int i = 0; i < _obstacle_polygons.size(); ++i) {
        setPolygonCost(master_grid, _obstacle_polygons[i], costmap_2d::LETHAL_OBSTACLE, min_i, min_j, max_i, max_j, true);
    }

    // set cost of obstacle points
    for (int i = 0; i < _obstacle_points.size(); ++i) {
        unsigned int mx;
        unsigned int my;
        if (master_grid.worldToMap(_obstacle_points[i].x, _obstacle_points[i].y, mx, my)) {
            master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
        }
    }
}

void VirtualLayer::computeMapBounds()
{
    std::lock_guard<std::mutex> l(_data_mutex);

    // reset bounds
    _min_x = _min_y = _max_x = _max_y = 0;

    // iterate zone polygons
    for (int i = 0; i < _zone_polygons.size(); ++i) {
        for (int j = 0; j < _zone_polygons.at(i).size(); ++j) {
            double px = _zone_polygons.at(i).at(j).x;
            double py = _zone_polygons.at(i).at(j).y;
            _min_x = std::min(px, _min_x);
            _min_y = std::min(py, _min_y);
            _max_x = std::max(px, _max_x);
            _max_y = std::max(py, _max_y);
        }
    }

    // iterate obstacle polygons
    for (int i = 0; i < _obstacle_polygons.size(); ++i) {
        for (int j = 0; j < _obstacle_polygons.at(i).size(); ++j) {
            double px = _obstacle_polygons.at(i).at(j).x;
            double py = _obstacle_polygons.at(i).at(j).y;
            _min_x = std::min(px, _min_x);
            _min_y = std::min(py, _min_y);
            _max_x = std::max(px, _max_x);
            _max_y = std::max(py, _max_y);
        }
    }

    // iterate obstacle points
    for (int i = 0; i < _obstacle_points.size(); ++i) {
        double px = _obstacle_points.at(i).x;
        double py = _obstacle_points.at(i).y;
        _min_x = std::min(px, _min_x);
        _min_y = std::min(py, _min_y);
        _max_x = std::max(px, _max_x);
        _max_y = std::max(py, _max_y);
    }
}

void VirtualLayer::setPolygonCost(costmap_2d::Costmap2D &master_grid, const Polygon &polygon, unsigned char cost,
                                  int min_i, int min_j, int max_i, int max_j, bool fill_polygon)
{
    std::vector<PointInt> map_polygon;
    for (unsigned int i = 0; i < polygon.size(); ++i) {
        PointInt loc;
        master_grid.worldToMapNoBounds(polygon[i].x, polygon[i].y, loc.x, loc.y);
        map_polygon.push_back(loc);
    }

    std::vector<PointInt> polygon_cells;

    // get the cells that fill the polygon
    rasterizePolygon(map_polygon, polygon_cells, fill_polygon);

    // set the cost of those cells
    for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
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
    for (unsigned int i = 0; i < polygon.size() - 1; ++i) {
        raytrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
    }
    if (!polygon.empty()) {
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

    for (; n > 0; --n) {
        cells.push_back(pt);

        if (error > 0) {
            pt.x += x_inc;
            error -= dy;
        } else {
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
    while (i < polygon_cells.size() - 1) {
        if (polygon_cells[i].x > polygon_cells[i + 1].x) {
            swap = polygon_cells[i];
            polygon_cells[i] = polygon_cells[i + 1];
            polygon_cells[i + 1] = swap;

            if (i > 0)
                --i;
        } else
            ++i;
    }

    i = 0;
    PointInt min_pt;
    PointInt max_pt;
    int min_x = polygon_cells[0].x;
    int max_x = polygon_cells[(int)polygon_cells.size() - 1].x;

    //walk through each column and mark cells inside the polygon
    for (int x = min_x; x <= max_x; ++x) {
        if (i >= (int)polygon_cells.size() - 1)
            break;

        if (polygon_cells[i].y < polygon_cells[i + 1].y) {
            min_pt = polygon_cells[i];
            max_pt = polygon_cells[i + 1];
        } else {
            min_pt = polygon_cells[i + 1];
            max_pt = polygon_cells[i];
        }

        i += 2;
        while (i < polygon_cells.size() && polygon_cells[i].x == x) {
            if (polygon_cells[i].y < min_pt.y)
                min_pt = polygon_cells[i];
            else if (polygon_cells[i].y > max_pt.y)
                max_pt = polygon_cells[i];
            ++i;
        }

        PointInt pt;
        //loop though cells in the column
        for (int y = min_pt.y; y < max_pt.y; ++y) {
            pt.x = x;
            pt.y = y;
            polygon_cells.push_back(pt);
        }
    }
}

void VirtualLayer::zoneCallback(const custom_msgs::ZoneConstPtr &zone_msg)
{
    if (zone_msg->area.form.size() > 2) {
        Polygon vector_to_add;
        for (int i = 0; i < zone_msg->area.form.size(); ++i) {
            vector_to_add.push_back(zone_msg->area.form[i]);
        }

        if (!robotInZone(vector_to_add)) {
            ROS_WARN_STREAM(tag << "Robot point is not the navigation zone");
            return;
        }

        if (_one_zone_mode) {
            _zone_polygons.clear();
        }
        _zone_polygons.push_back(vector_to_add);

        computeMapBounds();
    } else {
        ROS_ERROR_STREAM(tag << "A zone Layer needs to be a polygon with minimun 3 edges");
    }
}

void VirtualLayer::obstaclesCallback(const custom_msgs::ObstaclesConstPtr &obstacles_msg)
{
    if (_clear_obstacles) {
        _obstacle_polygons.clear();
        _obstacle_points.clear();
    }

    for (int i = 0; i < obstacles_msg->list.size(); ++i) {
        Polygon vector_to_add;
        if (obstacles_msg->list[i].form.size() == 1) {
            if (obstacles_msg->list[i].form[0].z == 0.0) {
                ROS_INFO_STREAM(tag << "Adding a Point");
                _obstacle_points.push_back(obstacles_msg->list[i].form[0]);
            } else if (obstacles_msg->list[i].form[0].z > 0.0) {
                ROS_INFO_STREAM(tag << "Adding a Circle");
                // Loop over 36 angles around a circle making a point each time
                int N = 36;
                geometry_msgs::Point pt;
                for (int j = 0; j < N; ++j) {
                    double angle = j * 2 * M_PI / N;
                    pt.x = obstacles_msg->list[i].form[0].x + cos(angle) * obstacles_msg->list[i].form[0].z;
                    pt.y = obstacles_msg->list[i].form[0].y + sin(angle) * obstacles_msg->list[i].form[0].z;
                    vector_to_add.push_back(pt);
                }
                _obstacle_polygons.push_back(vector_to_add);
            }
        } else if (obstacles_msg->list[i].form.size() == 2) {
            ROS_INFO_STREAM(tag << "Adding a Line");

            geometry_msgs::Point point_A = obstacles_msg->list[i].form[0];
            geometry_msgs::Point point_B = obstacles_msg->list[i].form[1];
            vector_to_add.push_back(point_A);
            vector_to_add.push_back(point_B);

            // calculate the normal vector for AB
            geometry_msgs::Point point_N;
            point_N.x = point_B.y - point_A.y;
            point_N.y = point_A.x - point_B.x;

            // get the absolute value of N to normalize and get
            // it to the length of the costmap resolution
            double abs_N = sqrt(pow(point_N.x, 2) + pow(point_N.y, 2));
            point_N.x = point_N.x / abs_N * _costmap_resolution;
            point_N.y = point_N.y / abs_N * _costmap_resolution;

            // calculate the new points to get a polygon which can be filled
            geometry_msgs::Point point;
            point.x = point_A.x + point_N.x;
            point.y = point_A.y + point_N.y;
            vector_to_add.push_back(point);

            point.x = point_B.x + point_N.x;
            point.y = point_B.y + point_N.y;
            vector_to_add.push_back(point);

            _obstacle_polygons.push_back(vector_to_add);
        } else {
            ROS_INFO_STREAM(tag << "Adding a Polygon");
            for (int j = 0; j < obstacles_msg->list[i].form.size(); ++j) {
                vector_to_add.push_back(obstacles_msg->list[i].form[j]);
            }
            _obstacle_polygons.push_back(vector_to_add);
        }
    }
    computeMapBounds();
}

geometry_msgs::Point VirtualLayer::getRobotPoint()
{
    tf::TransformListener tfListener;
    geometry_msgs::PoseStamped current_robot_pose, current_robot_pose_base;
    geometry_msgs::Point robot_point;
    geometry_msgs::TransformStamped current_transform_msg;
    tf::StampedTransform current_transform_tf;
    try {
        ros::Time now = ros::Time(0);
        tfListener.waitForTransform(_map_frame, _base_frame, now, ros::Duration(1.0));
        now = ros::Time::now();
        tfListener.getLatestCommonTime(_map_frame, _base_frame, now, nullptr);
        current_robot_pose_base.header.stamp = now;
        current_robot_pose_base.header.frame_id = _base_frame;
        current_robot_pose_base.pose.orientation = tf::createQuaternionMsgFromYaw(0);

        tfListener.transformPose(_map_frame, current_robot_pose_base, current_robot_pose);
        robot_point.x = current_robot_pose.pose.position.x;
        robot_point.y = current_robot_pose.pose.position.y;
        robot_point.z = 0.0;
    } catch (tf::TransformException &ex) {
        ROS_DEBUG_STREAM(tag << "Can't get robot pose: " << ex.what());
    }
    return robot_point;
}
} // namespace virtual_costmap_layer