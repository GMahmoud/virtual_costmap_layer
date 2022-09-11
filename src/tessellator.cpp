// Copyright (C) Mahmoud Ghorbel - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
// Written by MG <mahmoud.ghorbel@hotmail.com>

// This class is exported from a private repository

#include <poly2tr/poly2tri.h>
#include <virtual_costmap_layer/tessellator.hpp>

#include <map>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include <boost/geometry.hpp>

namespace rgk { namespace tessellator {

    static bool isUnsafePoint(const core::Point& point, const core::Polygon& polygon)
    {
        if (!boost::geometry::within(point, polygon.outer())) {
            return true;
        }

        for (const auto& ring : polygon.inners()) {
            if (boost::geometry::covered_by(point, ring)) {
                return true;
            }
            if (boost::geometry::distance(point, ring) < 0.01) {
                return true;
            }
        }

        return false;
    }

    // ----------------------------------------------------------------------------

    Output Tessellator::process(const core::Polygon& polygon)
    {
        auto outer = polygon.outer();
        auto inners = polygon.inners();

        std::vector<p2t::Point*> polyline;

        for (std::size_t i = 0; i < outer.size() - 1; ++i) {
            polyline.push_back(new p2t::Point(boost::geometry::get<0>(outer[i]), boost::geometry::get<1>(outer[i])));
        }

        auto cdt = std::make_shared<p2t::CDT>(polyline);

        for (const auto& inner : inners) {
            std::vector<p2t::Point*> hole;
            for (size_t i = 0; i < inner.size() - 1; ++i) {
                hole.push_back(new p2t::Point(boost::geometry::get<0>(inner[i]), boost::geometry::get<1>(inner[i])));
            }
            cdt->AddHole(hole);
        }

        cdt->Triangulate();

        auto raw_triangles = cdt->GetTriangles();

        std::vector<core::Ring> triangles;
        std::map<std::size_t, std::set<std::size_t>> neighborhood;
        std::map<p2t::Triangle*, std::size_t> indexes;

        triangles.reserve(raw_triangles.size());

        // nodes.reserve(raw_triangles.size());

        std::size_t index = 0;
        std::size_t unsafe_triangles = 0;
        for (std::size_t i = 0; i < raw_triangles.size(); ++i) {
            core::Point A(raw_triangles[i]->GetPoint(0)->x, raw_triangles[i]->GetPoint(0)->y);
            core::Point B(raw_triangles[i]->GetPoint(1)->x, raw_triangles[i]->GetPoint(1)->y);
            core::Point C(raw_triangles[i]->GetPoint(2)->x, raw_triangles[i]->GetPoint(2)->y);

            core::Ring triangle;
            triangle.push_back(A);
            triangle.push_back(B);
            triangle.push_back(C);
            triangle.push_back(A);

            core::Point center((boost::geometry::get<0>(A) + boost::geometry::get<0>(B) + boost::geometry::get<0>(C)) / 3.0, (boost::geometry::get<1>(A) + boost::geometry::get<1>(B) + boost::geometry::get<1>(C)) / 3.0);

            if (!isUnsafePoint(center, polygon)) {
                indexes.insert(std::make_pair(raw_triangles[i], index));
                triangles.push_back(triangle);
                // nodes.push_back(center);
                ++index;
            } else {
                ++unsafe_triangles;
            }
        }

        if (unsafe_triangles > 0) {
            ROS_INFO_STREAM("Unsafe " << unsafe_triangles << " triangle(s) has/have been detected");
        }

        for (const auto& element : indexes) {
            std::set<std::size_t> neighbors;
            for (std::size_t i = 0; i < 3; ++i) {
                try {
                    neighbors.insert(indexes.at(element.first->GetNeighbor(i)));
                } catch (...) {
                }
            }
            neighborhood.insert(std::make_pair(element.second, neighbors));
        }

        std::vector<core::Ring> triangle_list;
        triangle_list.reserve(neighborhood.size());
        for (const auto& e : neighborhood) {
            triangle_list.push_back(triangles[e.first]);
        }

        if (triangle_list.empty()) {
            std::string error_message = "Delaunay tessallation failed (poly2tri backend)";
            ROS_ERROR_STREAM(error_message);
            throw std::runtime_error(error_message);
        }

        return triangle_list;
    }

}} // namespace rgk::tessellator
