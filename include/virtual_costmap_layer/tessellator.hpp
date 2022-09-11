// Copyright (C) Mahmoud Ghorbel - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
// Written by MG <mahmoud.ghorbel@hotmail.com>

// This class is exported from a private repository

#pragma once

#include <virtual_costmap_layer/geometries.hpp>

namespace rgk { namespace tessellator {

    using Output = std::vector<core::Ring>;

    class Tessellator {
      public:
        Tessellator() = default;

        Output process(const core::Polygon& polygon);
    };

}} // namespace rgk::tessellator
