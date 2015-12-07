/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Fraunhofer IPA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Fraunhofer IPA nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mathias Lüdtke */
#include <ros/ros.h>
#include <ros/package.h> // for getting file path for loadng images

#include <moveit/setup_assistant/tools/moveit_config_data.h>

#include <boost/filesystem.hpp>  // for reading folders/files
#include <boost/program_options.hpp>

#include <fstream>  // for reading in urdf
#include <streambuf>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

bool isXacroFile(const std::string& path) { return path.find(".xacro") != std::string::npos; } // TODO: implement case-insensitive search

bool loadFileToString(std::string& buffer, const std::string& path){
    if(path.empty()) return false;

    std::ifstream stream( path.c_str() );
    if( !stream.good()) return false;

    // Load the file to a string using an efficient memory allocation technique
    stream.seekg(0, std::ios::end);
    buffer.reserve(stream.tellg());
    stream.seekg(0, std::ios::beg);
    buffer.assign( (std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>() );
    stream.close();

    return true;
}

bool loadXacroFileToString(std::string& buffer, const std::string& path, const std::vector<std::string> &xacro_args){
    if(path.empty()) return false;

    std::string cmd = "rosrun xacro xacro ";
    for(std::vector<std::string>::const_iterator it = xacro_args.begin(); it != xacro_args.end(); ++it)
        cmd += *it + " ";
    cmd += path;

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return false;

    char pipe_buffer[128];
    while(!feof(pipe)){
        if(fgets(pipe_buffer, 128, pipe) != NULL)
                buffer += pipe_buffer;
    }
    pclose(pipe);

    return true;
}

bool loadXmlFileToString(std::string& buffer, const std::string& path, const std::vector<std::string> &xacro_args){
    if(isXacroFile(path)){
        return loadXacroFileToString(buffer, path, xacro_args);
    }else{
        return loadFileToString(buffer, path);
    }
}

bool loadSetupAssistantConfig(moveit_setup_assistant::MoveItConfigData &config_data, const std::string &pkg_path){

    if(!config_data.setPackagePath(pkg_path))  return false;

    std::string setup_assistant_path;
    if(!config_data.getSetupAssistantYAMLPath(setup_assistant_path)) return false;

    if(!config_data.inputSetupAssistantYAML(setup_assistant_path)) return false;

    config_data.createFullURDFPath(); // might fail at this point

    config_data.createFullSRDFPath(config_data.config_pkg_path_); // might fail at this point

    return true;
}

bool setup(moveit_setup_assistant::MoveItConfigData &config_data, bool keep_old, const std::vector<std::string> &xacro_args) {
    std::string urdf_string;
    if(!loadXmlFileToString(urdf_string, config_data.urdf_path_, xacro_args)) return false;
    if(!config_data.urdf_model_->initString( urdf_string))return false;

    std::string srdf_string;
    if(!loadXmlFileToString(srdf_string, config_data.srdf_path_, xacro_args)) return false;
    if(!config_data.srdf_->initString( *config_data.urdf_model_, srdf_string)) return false;


    if(!keep_old) config_data.srdf_->disabled_collisions_.clear();

    return true;
}

moveit_setup_assistant::LinkPairMap compute(moveit_setup_assistant::MoveItConfigData &config_data,
                                            uint32_t trials, double min_collision_fraction,  bool verbose){
    // TODO: spin thread and print progess if verbose
    unsigned int collision_progress;
    return moveit_setup_assistant::computeDefaultCollisions(config_data.getPlanningScene(),
                                                            &collision_progress,
                                                            trials > 0,
                                                            trials,
                                                            min_collision_fraction, verbose);
}

int main(int argc, char * argv[]){

    std::string config_pkg_path;
    std::string urdf_path;
    std::string srdf_path;

    std::string output_path;

    bool include_default = false, include_always = false, keep_old = false, verbose = false;

    double min_collision_fraction = 1.0;

    uint32_t never_trials = 0;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "show help")
        ("config-pkg", po::value(&config_pkg_path), "path to moveit config package")
        ("urdf", po::value(&urdf_path), "path to URDF ( or xacro)")
        ("srdf", po::value(&srdf_path), "path to SRDF ( or xacro)")
        ("output", po::value(&output_path), "output path for SRDF")

        ("xacro-args", po::value<std::vector<std::string> >()->composing(), "additional arguments for xacro")

        ("default", po::bool_switch(&include_default),  "disable default colliding pairs")
        ("always", po::bool_switch(&include_always),  "disable always colliding pairs")

        ("keep", po::bool_switch(&keep_old),  "keep disabled link from SRDF")
        ("verbose", po::bool_switch(&verbose),  "verbose output")

        ("trials", po::value(&never_trials),  "number of trials for searching never colliding pairs")
        ("min-collision-fraction", po::value(&min_collision_fraction),  "fraction of small sample size to determine links that are alwas colliding")
      ;

    po::positional_options_description pos_desc;
    pos_desc.add("xacro-args", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(pos_desc).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    moveit_setup_assistant::MoveItConfigData config_data;

    if(!config_pkg_path.empty()){
        if(!loadSetupAssistantConfig(config_data, config_pkg_path)){
            std::cerr << "Could not load config at '" << config_pkg_path << "'" << std::endl;
            return 1;
        }
    }else if (urdf_path.empty() || srdf_path.empty()){
        std::cerr << "Please provide config package or URDF and SRDF path" << std::endl;
        return 1;
    }else if(isXacroFile(srdf_path) && output_path.empty()){
        std::cerr << "Please provide a different output file for SRDF xacro input file" << std::endl;
        return 1;
    }

    // overwrite config paths if applicable
    if(!urdf_path.empty()) config_data.urdf_path_ = urdf_path;
    if(!srdf_path.empty()) config_data.srdf_path_ = srdf_path;

    std::vector<std::string> xacro_args;
    if(vm.count("xacro-args")) xacro_args = vm["xacro-args"].as<std::vector<std::string> >();

    if(!setup(config_data, keep_old, xacro_args)){
        std::cerr << "Could not setup updater" << std::endl;
        return 1;
    }

    moveit_setup_assistant::LinkPairMap link_pairs = compute(config_data, never_trials, min_collision_fraction,verbose);

    size_t skip_mask = 0;
    if(!include_default) skip_mask |= (1<<moveit_setup_assistant::DEFAULT);
    if(!include_always) skip_mask |= (1<<moveit_setup_assistant::ALWAYS);

    config_data.setCollisionLinkPairs(link_pairs, skip_mask);

    config_data.srdf_->writeSRDF(output_path.empty() ? config_data.srdf_path_ : output_path );

    return 0;
}
