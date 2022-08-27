// Copyright (C) Mahmoud Ghorbel - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
// Written by MG <mahmoud.ghorbel@hotmail.com>

#include <sstream>

#include <virtual_costmap_layer/virtual_layer.hpp>

#include <boost/geometry.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <geometry_msgs/Pose.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>

#include <XmlRpcException.h>

PLUGINLIB_EXPORT_CLASS(virtual_costmap_layer::VirtualLayer, costmap_2d::Layer);

static const std::string tag {"[VIRTUAL-LAYER] "};

static const std::string getUUID()
{
    boost::uuids::random_generator generator;
    return (boost::uuids::to_string(generator()));
}

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

    _dsrv = std::make_shared<dynamic_reconfigure::Server<VirtualLayerConfig>>(nh);

    dynamic_reconfigure::Server<VirtualLayerConfig>::CallbackType cb = boost::bind(&VirtualLayer::reconfigureCb, this, _1, _2);
    _dsrv->setCallback(cb);

    // save resolution
    _costmap_resolution = layered_costmap_->getCostmap()->getResolution();

    // set initial bounds
    _min_x = _min_y = _max_x = _max_y = 0;

    // advertising services
    _add_server = nh.advertiseService("add", &VirtualLayer::addElement, this);
    _remove_server = nh.advertiseService("remove", &VirtualLayer::removeElement, this);
    _get_server = nh.advertiseService("get", &VirtualLayer::getElement, this);
    _status_server = nh.advertiseService("status", &VirtualLayer::getElements, this);
    _clear_server = nh.advertiseService("clear", &VirtualLayer::clear, this);

    _geometries.insert(std::make_pair(GeometryType::LINESTRING, std::map<std::string, Geometry>()));
    _geometries.insert(std::make_pair(GeometryType::POLYGON, std::map<std::string, Geometry>()));
    _geometries.insert(std::make_pair(GeometryType::RING, std::map<std::string, Geometry>()));
    _geometries.insert(std::make_pair(GeometryType::CIRCLE, std::map<std::string, Geometry>()));

    // reading the defined forms out of the namespace of this plugin!
    parseFormListFromYaml(nh);

    // compute map bounds for the current set of areas and obstacles.
    computeMapBounds();

    // ROS_INFO_STREAM(tag << "layer is initialized: [points: " << _form_points.size() << "] [polygons: " << _form_polygons.size() << "]");
}

// ---------------------------------------------------------------------

bool VirtualLayer::addElement(virtual_costmap_layer::AddElementRequest& req, virtual_costmap_layer::AddElementResponse& res)
{
    GeometryType type;
    switch (req.form.type) {
        case virtual_costmap_layer::Form::TYPE_LINESTRING:
            type = GeometryType::LINESTRING;
            break;
        case virtual_costmap_layer::Form::TYPE_POLYGON:
            type = GeometryType::POLYGON;
            break;
        case virtual_costmap_layer::Form::TYPE_RING:
            type = GeometryType::RING;
            break;
        case virtual_costmap_layer::Form::TYPE_CIRCLE:
            type = GeometryType::CIRCLE;
            break;
        default:
            res.success = false;
            res.message = "Unsupported type request (" + std::to_string(req.form.type) + ")";
            return true;
    }

    switch (type) {
        case GeometryType::LINESTRING: {
            auto has_form = req.form.data.find("LINESTRING");
            if (has_form != std::string::npos) {
                rgk::core::LineString linestring;
                try {
                    boost::geometry::read_wkt(req.form.data, linestring);
                } catch (...) {
                    res.success = false;
                    res.message = "Add element failed: [reason: request data corrupted]";
                    ROS_WARN_STREAM(tag << res.message);
                    return true;
                }
                boost::geometry::correct(linestring);
                if (linestring.empty()) {
                    res.success = false;
                    res.message = "Add element failed: [reason: request data empty]";
                    ROS_WARN_STREAM(tag << res.message);
                    return true;
                }
                const auto uuid = saveLineStringGeometry(linestring);
                res.success = true;
                res.uuid = uuid;
            } else {
                res.success = false;
                res.message = "Add element failed: [reason: request data corrupted]";
                ROS_WARN_STREAM(tag << res.message);
                return true;
            }
        } break;

        case GeometryType::POLYGON: {
            auto has_form = req.form.data.find("POLYGON");
            if (has_form != std::string::npos) {
                rgk::core::Polygon polygon;
                try {
                    boost::geometry::read_wkt(req.form.data, polygon);
                } catch (...) {
                    res.success = false;
                    res.message = "Add element failed: [reason: request data corrupted]";
                    ROS_WARN_STREAM(tag << res.message);
                    return true;
                }
                boost::geometry::correct(polygon);
                if (polygon.outer().empty() && polygon.inners().empty()) {
                    res.success = false;
                    res.message = "Add element failed: [reason: request data empty]";
                    ROS_WARN_STREAM(tag << res.message);
                    return true;
                }

                const auto uuid = savePolygonGeometry(polygon);
                res.success = true;
                res.uuid = uuid;
            } else {
                res.success = false;
                res.message = "Add element failed: [reason: request data corrupted]";
                ROS_WARN_STREAM(tag << res.message);
                return true;
            }
        } break;

        case GeometryType::RING: {
            auto has_form = req.form.data.find("POLYGON");
            if (has_form != std::string::npos) {
                rgk::core::Polygon polygon;
                try {
                    boost::geometry::read_wkt(req.form.data, polygon);
                } catch (...) {
                    res.success = false;
                    res.message = "Add element failed: [reason: request data corrupted]";
                    ROS_WARN_STREAM(tag << res.message);
                    return true;
                }
                boost::geometry::correct(polygon);
                if (polygon.outer().empty() && polygon.inners().empty()) {
                    res.success = false;
                    res.message = "Add element failed: [reason: request data empty]";
                    ROS_WARN_STREAM(tag << res.message);
                    return true;
                }

                if (!polygon.outer().empty() || polygon.inners().size() != 1) {
                    res.success = false;
                    res.message = "Add element failed: [reason: request data mismatched ring type]";
                    ROS_WARN_STREAM(tag << res.message);
                    return true;
                }

                const auto uuid = savePolygonGeometry(polygon);
                res.success = true;
                res.uuid = uuid;
            } else {
                res.success = false;
                res.message = "Add element failed: [reason: request data corrupted]";
                ROS_WARN_STREAM(tag << res.message);
                return true;
            }
        } break;

        case GeometryType::CIRCLE:
            res.success = false;
            res.message = "Circle type not implemented yet";
            ROS_WARN_STREAM(tag << res.message);
            return true;
    }

    return true;
}

// ---------------------------------------------------------------------

bool VirtualLayer::removeElement(virtual_costmap_layer::RemoveElementRequest& req, virtual_costmap_layer::RemoveElementResponse& res)
{
    bool deleted = false;

    auto process = [this, &deleted, &req](GeometryType type) {
        if (_geometries[type].find(req.uuid) != _geometries[type].end()) {
            deleted = true;
            _geometries[type].erase(req.uuid);
        }
    };

    process(GeometryType::LINESTRING);
    if (deleted) {
        res.success = true;
        return true;
    }

    process(GeometryType::POLYGON);
    if (deleted) {
        res.success = true;
        return true;
    }

    process(GeometryType::RING);
    if (deleted) {
        res.success = true;
        return true;
    }

    process(GeometryType::CIRCLE);
    if (deleted) {
        res.success = true;
        return true;
    } else {
        res.success = false;
        res.message = "No element with the given uuid";
        return true;
    }

    return true;
}

// ---------------------------------------------------------------------

bool VirtualLayer::clear(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{

    ROS_INFO_STREAM(tag << "Clearing layer");
    _geometries[GeometryType::LINESTRING].clear();
    _geometries[GeometryType::POLYGON].clear();
    _geometries[GeometryType::RING].clear();
    _geometries[GeometryType::CIRCLE].clear();

    res.success = true;
    return true;
}

// ---------------------------------------------------------------------

bool VirtualLayer::getElement(virtual_costmap_layer::GetElementRequest& req, virtual_costmap_layer::GetElementResponse& res)
{
    bool found = false;

    auto process = [this, &found, &req, &res](GeometryType type) {
        if (_geometries[type].find(req.uuid) != _geometries[type].end()) {
            found = true;
            res.form = toForm(_geometries[type][req.uuid], type);
            res.form.uuid = req.uuid;
        }
    };

    process(GeometryType::LINESTRING);
    if (found) {
        res.success = true;
        return true;
    }

    process(GeometryType::POLYGON);
    if (found) {
        res.success = true;
        return true;
    }

    process(GeometryType::RING);
    if (found) {
        res.success = true;
        return true;
    }

    process(GeometryType::CIRCLE);
    if (found) {
        res.success = true;
        return true;
    } else {
        res.success = false;
        res.message = "No element with the given uuid";
        return true;
    }
}

// ---------------------------------------------------------------------

bool VirtualLayer::getElements(virtual_costmap_layer::GetElementsRequest& req, virtual_costmap_layer::GetElementsResponse& res)
{
    res.forms = toForms();
    return true;
}

// ---------------------------------------------------------------------

// load forms from ymal configuration
void VirtualLayer::parseFormListFromYaml(const ros::NodeHandle& nh)
{
    XmlRpc::XmlRpcValue param_yaml;
    std::string param = "forms";
    if (nh.getParam(param, param_yaml)) {
        if (param_yaml.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (std::size_t i = 0; i < param_yaml.size(); ++i) {
                if (param_yaml[i].getType() == XmlRpc::XmlRpcValue::TypeString) {

                    std::string value = std::string(param_yaml[i]);
                    bool found = false;

                    {
                        auto has_form = value.find("LINESTRING");
                        if (has_form != std::string::npos) {
                            found = true;
                            rgk::core::LineString linestring;
                            try {
                                boost::geometry::read_wkt(value, linestring);
                            } catch (...) {
                                ROS_ERROR_STREAM(tag << param << " with index #" << i << " is corrupted [reason: geometry corrupted]");
                                continue;
                            }
                            boost::geometry::correct(linestring);
                            saveLineStringGeometry(linestring);
                        }
                    }

                    if (found) {
                        continue;
                    }

                    {
                        auto has_form = value.find("POLYGON");
                        if (has_form != std::string::npos) {
                            found = true;
                            rgk::core::Polygon polygon;
                            try {
                                boost::geometry::read_wkt(value, polygon);
                            } catch (...) {
                                ROS_ERROR_STREAM(tag << param << " with index #" << i << " is corrupted [reason: geometry corrupted]");
                                continue;
                            }
                            boost::geometry::correct(polygon);
                            savePolygonGeometry(polygon);
                        }
                    }

                    if (found) {
                        continue;
                    }

                    ROS_ERROR_STREAM(tag << param << " with index #" << i << " is corrupted [reason: geometry not supported]");
                } else {
                    ROS_ERROR_STREAM(tag << param << " with index #" << i << " is corrupted [reason: not a string type]");
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

std::string VirtualLayer::saveLineStringGeometry(const rgk::core::LineString& linestring)
{
    const auto uuid = getUUID();
    Geometry geometry;
    geometry._linestring = linestring;
    ROS_INFO_STREAM(tag << "Adding LineString [uuid: " << uuid << "]");
    _geometries[GeometryType::LINESTRING].insert(std::make_pair(uuid, geometry));

    return uuid;
}

// ---------------------------------------------------------------------

std::string VirtualLayer::savePolygonGeometry(const rgk::core::Polygon& polygon)
{
    const auto uuid = getUUID();

    Geometry geometry;

    if (polygon.outer().empty() && polygon.inners().size() == 1) {
        ROS_INFO_STREAM(tag << "Adding Ring [uuid: " << uuid << "]");
        geometry._ring = polygon.inners()[0];
        _geometries[GeometryType::RING].insert(std::make_pair(uuid, geometry));

    } else {
        ROS_INFO_STREAM(tag << "Adding Polygon [uuid: " << uuid << "]");
        geometry._polygon = polygon;
        _geometries[GeometryType::POLYGON].insert(std::make_pair(uuid, geometry));
    }

    return uuid;
}

// ---------------------------------------------------------------------

virtual_costmap_layer::Form VirtualLayer::toForm(const Geometry& geometry, GeometryType type) const
{
    virtual_costmap_layer::Form form;
    switch (type) {
        case GeometryType::LINESTRING: {
            form.type = virtual_costmap_layer::Form::TYPE_LINESTRING;
            std::stringstream ss;
            ss << boost::geometry::wkt(geometry._linestring.value());
            form.data = ss.str();
            form.description = "TYPE_LINESTRING";
        } break;

        case GeometryType::POLYGON: {
            form.type = virtual_costmap_layer::Form::TYPE_POLYGON;
            std::stringstream ss;
            ss << boost::geometry::wkt(geometry._polygon.value());
            form.data = ss.str();
            form.description = "TYPE_POLYGON";
        } break;

        case GeometryType::RING: {
            form.type = virtual_costmap_layer::Form::TYPE_RING;
            rgk::core::Polygon polygon;
            polygon.inners().reserve(1);
            polygon.inners().push_back(geometry._ring.value());
            std::stringstream ss;
            ss << boost::geometry::wkt(polygon);
            form.data = ss.str();
            form.description = "TYPE_RING";
        } break;

        case GeometryType::CIRCLE:
            // form.type = virtual_costmap_layer::Form::TYPE_CIRCLE;
            // form.data=
            break;
    }

    return form;
}

// ---------------------------------------------------------------------

std::vector<virtual_costmap_layer::Form> VirtualLayer::toForms() const
{
    auto size = _geometries.at(GeometryType::LINESTRING).size() +
                _geometries.at(GeometryType::POLYGON).size() +
                _geometries.at(GeometryType::RING).size() +
                _geometries.at(GeometryType::CIRCLE).size();

    std::vector<virtual_costmap_layer::Form> forms;
    forms.reserve(size);

    auto process = [this, &forms](GeometryType type) {
        for (const auto& pair : _geometries.at(type)) {
            auto form = toForm(pair.second, type);
            form.uuid = pair.first;
            forms.push_back(form);
        }
    };

    process(GeometryType::LINESTRING);
    process(GeometryType::POLYGON);
    process(GeometryType::RING);
    process(GeometryType::CIRCLE);

    return forms;
}

// ---------------------------------------------------------------------

void VirtualLayer::reconfigureCb(VirtualLayerConfig& config, uint32_t level)
{
    enabled_ = config.enabled;
    _base_frame = config.base_frame;
    _map_frame = config.map_frame;
}

// ---------------------------------------------------------------------

void VirtualLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!enabled_) {
        return;
    }

    std::lock_guard<std::mutex> l(_data_mutex);

    auto size = _geometries.at(GeometryType::LINESTRING).size() +
                _geometries.at(GeometryType::POLYGON).size() +
                _geometries.at(GeometryType::RING).size() +
                _geometries.at(GeometryType::CIRCLE).size();

    if (size == 0) {
        return;
    }

    *min_x = std::min(*min_x, _min_x);
    *min_y = std::min(*min_y, _min_y);
    *max_x = std::max(*max_x, _max_x);
    *max_y = std::max(*max_y, _max_y);
}

// ---------------------------------------------------------------------

void VirtualLayer::updateCosts(costmap_2d::Costmap2D& grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_) {
        return;
    }

    std::lock_guard<std::mutex> l(_data_mutex);

    // set costs of polygons
    for (const auto& pair : _geometries[GeometryType::POLYGON]) {
        setRingCost(grid, pair.second._polygon.value().outer(),
                    costmap_2d::LETHAL_OBSTACLE,
                    min_i, min_j, max_i, max_j,
                    false);

        for (const auto& inner : pair.second._polygon.value().inners()) {
            setRingCost(grid, inner,
                        costmap_2d::LETHAL_OBSTACLE,
                        min_i, min_j, max_i, max_j,
                        true);
        }
    }

    // // set costs of obstacle polygons
    // for (int i = 0; i < _obstacle_polygons.size(); ++i) {
    //     setPolygonCost(master_grid, _obstacle_polygons[i], costmap_2d::LETHAL_OBSTACLE, min_i, min_j, max_i, max_j, true);
    // }

    // // set cost of obstacle points
    // for (int i = 0; i < _obstacle_points.size(); ++i) {
    //     unsigned int mx;
    //     unsigned int my;
    //     if (master_grid.worldToMap(_obstacle_points[i].x, _obstacle_points[i].y, mx, my)) {
    //         master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
    //     }
    // }
}

// ---------------------------------------------------------------------

void VirtualLayer::setRingCost(costmap_2d::Costmap2D& grid,
                               const rgk::core::Ring& ring,
                               unsigned char cost,
                               int min_i, int min_j, int max_i, int max_j,
                               bool fill) const
{

    std::vector<PointInt> map;
    for (const auto& point : ring) {
        PointInt loc;
        grid.worldToMapNoBounds(boost::geometry::get<0>(point), boost::geometry::get<1>(point), loc.x, loc.y);
        map.push_back(loc);
    }

    std::vector<PointInt> cells;

    // get the cells
    rasterize(map, cells, fill);

    // set the cost of those cells
    for (const auto& cell : cells) {
        int mx = cell.x;
        int my = cell.y;
        // check if point is outside bounds
        if (mx < min_i || mx >= max_i) {
            continue;
        }
        if (my < min_j || my >= max_j) {
            continue;
        }
        grid.setCost(mx, my, cost);
    }
}

// ---------------------------------------------------------------------

void VirtualLayer::rasterize(const std::vector<PointInt>& ring, std::vector<PointInt>& cells, bool fill) const
{
    //we need a minimum point if the ring
    if (ring.size() < 3) {
        return;
    }

    //first get the cells that make up the outline of the polygon
    outlineCells(ring, cells);

    if (!fill) {
        return;
    }

    //quick bubble sort to sort points by x
    PointInt swap;
    unsigned int i = 0;
    while (i < cells.size() - 1) {
        if (cells[i].x > cells[i + 1].x) {
            swap = cells[i];
            cells[i] = cells[i + 1];
            cells[i + 1] = swap;

            if (i > 0) {
                --i;
            }
        } else {
            ++i;
        }
    }

    i = 0;
    PointInt min_pt;
    PointInt max_pt;
    int min_x = cells[0].x;
    int max_x = cells[(int)cells.size() - 1].x;

    //walk through each column and mark cells inside the polygon
    for (int x = min_x; x <= max_x; ++x) {
        if (i >= (int)cells.size() - 1) {
            break;
        }

        if (cells[i].y < cells[i + 1].y) {
            min_pt = cells[i];
            max_pt = cells[i + 1];
        } else {
            min_pt = cells[i + 1];
            max_pt = cells[i];
        }

        i += 2;
        while (i < cells.size() && cells[i].x == x) {
            if (cells[i].y < min_pt.y) {
                min_pt = cells[i];
            } else if (cells[i].y > max_pt.y) {
                max_pt = cells[i];
            }
            ++i;
        }

        PointInt pt;
        //loop though cells in the column
        for (int y = min_pt.y; y < max_pt.y; ++y) {
            pt.x = x;
            pt.y = y;
            cells.push_back(pt);
        }
    }
}

// ---------------------------------------------------------------------

void VirtualLayer::outlineCells(const std::vector<PointInt>& ring, std::vector<PointInt>& cells) const
{
    for (std::size_t i = 0; i < ring.size() - 1; ++i) {
        raytrace(ring[i].x, ring[i].y, ring[i + 1].x, ring[i + 1].y, cells);
    }

    if (!ring.empty()) {
        unsigned int last_index = ring.size() - 1;
        // we also need to close the polygon by going from the last point to the first
        raytrace(ring[last_index].x, ring[last_index].y, ring[0].x, ring[0].y, cells);
    }
}

// ---------------------------------------------------------------------

void VirtualLayer::raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt>& cells) const
{
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);

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

// ---------------------------------------------------------------------

void VirtualLayer::computeMapBounds()
{
    std::lock_guard<std::mutex> l(_data_mutex);

    // reset bounds
    _min_x = _min_y = _max_x = _max_y = 0;

    // iterate on polygons
    for (const auto& pair : _geometries[GeometryType::POLYGON]) {
        for (const auto& point : pair.second._polygon.value().outer()) {
            double px = boost::geometry::get<0>(point);
            double py = boost::geometry::get<1>(point);
            _min_x = std::min(px, _min_x);
            _min_y = std::min(py, _min_y);
            _max_x = std::max(px, _max_x);
            _max_y = std::max(py, _max_y);
        }
    }

    // // iterate obstacle polygons
    // for (int i = 0; i < _obstacle_polygons.size(); ++i) {
    //     for (int j = 0; j < _obstacle_polygons.at(i).size(); ++j) {
    //         double px = _obstacle_polygons.at(i).at(j).x;
    //         double py = _obstacle_polygons.at(i).at(j).y;
    //         _min_x = std::min(px, _min_x);
    //         _min_y = std::min(py, _min_y);
    //         _max_x = std::max(px, _max_x);
    //         _max_y = std::max(py, _max_y);
    //     }
    // }

    // // iterate obstacle points
    // for (int i = 0; i < _obstacle_points.size(); ++i) {
    //     double px = _obstacle_points.at(i).x;
    //     double py = _obstacle_points.at(i).y;
    //     _min_x = std::min(px, _min_x);
    //     _min_y = std::min(py, _min_y);
    //     _max_x = std::max(px, _max_x);
    //     _max_y = std::max(py, _max_y);
    // }
}

} // namespace virtual_costmap_layer