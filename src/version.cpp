// Copyright (C) Mahmoud Ghorbel - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential
// Written by MG <mahmoud.ghorbel@hotmail.com>

#include "virtual_costmap_layer_version.hpp"

#include <iostream>
#include <sstream>
#include <string>

#include <docopt.h>

#include <nlohmann/json.hpp>

namespace virtual_costmap_layer {

struct Version {
    std::string major;
    std::string minor;
    std::string patch;
};

struct ProjectVersion {
    Version version;
    std::string full_version;
    std::string release_type;
};

struct Compiler {
    std::string id;
    std::string version;
};

struct System {
    std::string system;
    std::string processor;
};

struct Git {
    std::string branch;
    std::string commit;
    bool unstaged_changes;
};

struct ProjectInfo {
    ProjectVersion project_version;
    Compiler c_compiler;
    Compiler cxx_compiler;
    System host;
    System target;
    Git git;
    std::string build_date;
};

// --------------------------------------------------------------

ProjectVersion projectVersion()
{
    return {{VIRTUAL_COSTMAP_LAYER_PROJECT_VERSION_MAJOR, VIRTUAL_COSTMAP_LAYER_PROJECT_VERSION_MINOR,
             VIRTUAL_COSTMAP_LAYER_PROJECT_VERSION_PATCH},
            VIRTUAL_COSTMAP_LAYER_PROJECT_VERSION,
            VIRTUAL_COSTMAP_LAYER_PROJECT_RELEASE_TYPE};
}

// --------------------------------------------------------------

ProjectInfo projectInfo()
{
    return {
        projectVersion(),
        {VIRTUAL_COSTMAP_LAYER_CMAKE_C_COMPILER_ID, VIRTUAL_COSTMAP_LAYER_CMAKE_C_COMPILER_VERSION},
        {VIRTUAL_COSTMAP_LAYER_CMAKE_CXX_COMPILER_ID,
         VIRTUAL_COSTMAP_LAYER_CMAKE_CXX_COMPILER_VERSION},
        {VIRTUAL_COSTMAP_LAYER_HOST_SYSTEM, VIRTUAL_COSTMAP_LAYER_CMAKE_HOST_SYSTEM_PROCESSOR},
        {VIRTUAL_COSTMAP_LAYER_TARGET_SYSTEM, VIRTUAL_COSTMAP_LAYER_CMAKE_SYSTEM_PROCESSOR},
        {VIRTUAL_COSTMAP_LAYER_GIT_BRANCH, VIRTUAL_COSTMAP_LAYER_GIT_COMMIT,
         VIRTUAL_COSTMAP_LAYER_GIT_UNSTAGED_CHANGES},
        VIRTUAL_COSTMAP_LAYER_PROJECT_BUILD_DATE};
}

// --------------------------------------------------------------

void print(const std::string& project_name, const ProjectInfo& project_info, bool with_json)
{
    if (!with_json) {
        std::stringstream ss;
        ss << "\n";
        ss << "--------------------- virtual_costmap_layer --------------------- \n";
        ss << "Version:              " << project_info.project_version.full_version
           << "\n"
           << "Version details :     "
           << "[major:   " << project_info.project_version.version.major << "] "
           << "[minor:   " << project_info.project_version.version.minor << "] "
           << "[patch:   " << project_info.project_version.version.patch << "] "
           << "\n"
           << "Release type:         " << project_info.project_version.release_type
           << "\n"
           << "\n";

        ss << "C compiler id:        " << project_info.c_compiler.id << "\n"
           << "C compiler version:   " << project_info.c_compiler.version << "\n"
           << "CXX compiler id:      " << project_info.cxx_compiler.id << "\n"
           << "CXX Compiler version: " << project_info.cxx_compiler.version << "\n"
           << "\n";

        ss << "Host system:          " << project_info.host.system << "\n"
           << "Host processor:       " << project_info.host.processor << "\n"
           << "Target system:        " << project_info.target.system << "\n"
           << "Target processor:     " << project_info.target.processor << "\n"
           << "\n";

        ss << "Git branch:           " << project_info.git.branch << "\n"
           << "Git commit:           " << project_info.git.commit << "\n"
           << "Git unstaged changes: " << project_info.git.unstaged_changes << "\n"
           << "\n";

        ss << "Build date:           " << project_info.build_date << "\n";
        ss << "------------------------------------------------------- \n";

        std::cout << ss.str() << "\n";
    } else {
        nlohmann::json json;

        json["virtual_costmap_layer"]["version"] = project_info.project_version.full_version;

        json["virtual_costmap_layer"]["version_details"]["major"] = project_info.project_version.version.major;
        json["virtual_costmap_layer"]["version_details"]["minor"] = project_info.project_version.version.minor;
        json["virtual_costmap_layer"]["version_details"]["patch"] = project_info.project_version.version.patch;

        json["virtual_costmap_layer"]["release_type"] = project_info.project_version.release_type;

        json["virtual_costmap_layer"]["c_compiler"]["id"] = project_info.c_compiler.id;
        json["virtual_costmap_layer"]["c_compiler"]["version"] = project_info.c_compiler.version;
        json["virtual_costmap_layer"]["cxx_compiler"]["id"] = project_info.cxx_compiler.id;
        json["virtual_costmap_layer"]["cxx_compiler"]["version"] = project_info.cxx_compiler.version;

        json["virtual_costmap_layer"]["host"]["system"] = project_info.host.system;
        json["virtual_costmap_layer"]["host"]["processor"] = project_info.host.processor;

        json["virtual_costmap_layer"]["target"]["system"] = project_info.target.system;
        json["virtual_costmap_layer"]["target"]["processor"] = project_info.target.processor;

        json["virtual_costmap_layer"]["git"]["branch"] = project_info.git.branch;
        json["virtual_costmap_layer"]["git"]["commit"] = project_info.git.commit;
        json["virtual_costmap_layer"]["git"]["unstaged_changes"] = project_info.git.unstaged_changes;

        json["virtual_costmap_layer"]["build_date"] = project_info.build_date;

        std::cout << json.dump() << "\n";
    }
}

} // namespace virtual_costmap_layer

// --------------------------------------------------------------

static const char usage[] =
    R"( virtual_costmap_layer 

    Usage:
      interface [--json]
      interface (-h |--help)
      interface --version

    Options:
      -h --help                 Show help
      --version                 Show version
)";

// --------------------------------------------------------------

int main(int argc, char** argv)
{
    std::stringstream ss;
    ss << usage;

    std::string project_name = "virtual_costmap_layer";

    std::map<std::string, docopt::value> args =
        docopt::docopt(ss.str(), {argv + 1, argv + argc}, true,
                       project_name + " " + VIRTUAL_COSTMAP_LAYER_PROJECT_VERSION);

    virtual_costmap_layer::print(project_name, virtual_costmap_layer::projectInfo(), args["--json"].asBool());

    return EXIT_SUCCESS;
}