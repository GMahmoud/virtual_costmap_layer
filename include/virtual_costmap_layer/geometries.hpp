
// Copyright (C) Mahmoud Ghorbel - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
// Written by MG <mahmoud.ghorbel@hotmail.com>

// These definitions are exported from a private repository

#pragma once

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/ring.hpp>

namespace rgk { namespace core {

    using CartesianSystem = boost::geometry::cs::cartesian;

    template <std::size_t dim, typename CoordinateSystem>
    using PointBase = boost::geometry::model::point<double, dim, CoordinateSystem>;

    template <typename CoordinateSystem>
    using PointBase2d = PointBase<2, CoordinateSystem>;

    template <typename CoordinateSystem>
    using PointBase3d = PointBase<3, CoordinateSystem>;

    template <typename Point>
    using RingBase = boost::geometry::model::ring<Point, true, true>;

    template <typename Point>
    using PolygonBase = boost::geometry::model::polygon<Point, true, true>;

    template <typename Point>
    using LineStringBase = boost::geometry::model::linestring<Point>;

    using Point = PointBase2d<CartesianSystem>;
    using Point3d = PointBase3d<CartesianSystem>;

    using Ring = RingBase<Point>;
    using Ring3d = RingBase<Point3d>;

    using Polygon = PolygonBase<Point>;
    using Polygon3d = PolygonBase<Point3d>;

    using LineString = LineStringBase<Point>;
    using LineString3d = LineStringBase<Point3d>;

}} // namespace rgk::core
