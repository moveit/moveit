#include <ros/ros.h>
#include <ros/package.h> // for getting file path for loadng images

#include <moveit/setup_assistant/tools/compute_default_collisions.h>
#include <moveit/setup_assistant/tools/moveit_config_data.h>

#include <boost/filesystem.hpp>  // for reading folders/files
#include <boost/program_options.hpp>
#include <boost/algorithm/string/join.hpp>

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

class SortableDisabledCollision {
     const srdf::Model::DisabledCollision dc_;
     const std::string key_;
public:
    SortableDisabledCollision(const srdf::Model::DisabledCollision &dc)
    : dc_(dc), key_(dc.link1_ < dc.link2_ ? (dc.link1_ + "|" + dc.link2_) : (dc.link2_ + "|" + dc.link1_)) {}
    operator const srdf::Model::DisabledCollision () const { return dc_; }
    bool operator < (const SortableDisabledCollision &other) const { return key_ < other.key_; }
};

class CollisionUpdater{
    moveit_setup_assistant::MoveItConfigData config_data;
public:
    bool loadSetupAssistantConfig(const std::string &path){
        config_data.config_pkg_path_ = path;

        fs::path setup_assistant_file = config_data.config_pkg_path_;
        setup_assistant_file /= ".setup_assistant";

        if(!config_data.inputSetupAssistantYAML( setup_assistant_file.make_preferred().native().c_str() ) )
            return false;

        fs::path urdf_path;

        if( config_data.urdf_pkg_name_.empty() || config_data.urdf_pkg_name_ == "\"\"" ){
            urdf_path = config_data.urdf_pkg_relative_path_;
        }else{
            fs::path robot_desc_pkg_path = ros::package::getPath( config_data.urdf_pkg_name_ );
            if( robot_desc_pkg_path.empty() ) return false;

            urdf_path = robot_desc_pkg_path / config_data.urdf_pkg_relative_path_;
        }
        config_data.urdf_path_ = urdf_path.make_preferred().native();

        fs::path srdf_path = config_data.config_pkg_path_ ;
        srdf_path /= config_data.srdf_pkg_relative_path_;
        config_data.srdf_path_ = srdf_path.make_preferred().native();

        return true;
    }

    void setURDF(const std::string &path) {
        if(!path.empty()) config_data.urdf_path_ = path;
    }
    void setSRDF(const std::string &path) {
        if(!path.empty()) config_data.srdf_path_ = path;
    }

    bool setup(bool keep_old, const std::vector<std::string> &xacro_args){
        std::string urdf_string;
        if(!loadXmlFileToString(urdf_string, config_data.urdf_path_, xacro_args)) return false;
        if(!config_data.urdf_model_->initString( urdf_string))return false;

        std::string srdf_string;
        if(!loadXmlFileToString(srdf_string, config_data.srdf_path_, xacro_args)) return false;
        if(!config_data.srdf_->initString( *config_data.urdf_model_, srdf_string)) return false;


        if(!keep_old) config_data.srdf_->disabled_collisions_.clear();

        return true;
    }

    moveit_setup_assistant::LinkPairMap compute(uint32_t trials, double min_collision_fraction,  bool verbose){
        unsigned int collision_progress;
        return moveit_setup_assistant::computeDefaultCollisions(config_data.getPlanningScene(),
                                                                &collision_progress,
                                                                trials > 0,
                                                                trials,
                                                                min_collision_fraction, verbose);
    }

    void write(const moveit_setup_assistant::LinkPairMap &link_pairs, bool include_default, bool include_always, const std::string &output_path){
        // Create temp disabled collision
        srdf::Model::DisabledCollision dc;

        std::set<SortableDisabledCollision> disabled_collisions;
        disabled_collisions.insert(config_data.srdf_->disabled_collisions_.begin(), config_data.srdf_->disabled_collisions_.end());

        // copy the data in this class's LinkPairMap datastructure to srdf::Model::DisabledCollision format
        for ( moveit_setup_assistant::LinkPairMap::const_iterator pair_it = link_pairs.begin();
                pair_it != link_pairs.end(); ++pair_it)
        {
            // Only copy those that are actually disabled
            if( pair_it->second.disable_check ){
                if( pair_it->second.reason == moveit_setup_assistant::DEFAULT && !include_default) continue;
                if( pair_it->second.reason == moveit_setup_assistant::ALWAYS && !include_always) continue;
                dc.link1_ = pair_it->first.first;
                dc.link2_ = pair_it->first.second;
                dc.reason_ = moveit_setup_assistant::disabledReasonToString( pair_it->second.reason );
                disabled_collisions.insert(SortableDisabledCollision(dc));
            }
        }

        config_data.srdf_->disabled_collisions_.assign(disabled_collisions.begin(), disabled_collisions.end());

        // Update collision_matrix for robot pose's use
        config_data.loadAllowedCollisionMatrix();

        config_data.srdf_->writeSRDF(output_path.empty() ? config_data.srdf_path_ : output_path );

    }

};

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

    CollisionUpdater updater;

    if(!config_pkg_path.empty()){
        if(!updater.loadSetupAssistantConfig(config_pkg_path)){
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

    updater.setURDF(urdf_path);
    updater.setSRDF(srdf_path);

    std::vector<std::string> xacro_args;
    if(vm.count("xacro-args")) xacro_args = vm["xacro-args"].as<std::vector<std::string> >();

    if(!updater.setup(keep_old, xacro_args)){
        std::cerr << "Could not setup updater" << std::endl;
        return 1;
    }

    moveit_setup_assistant::LinkPairMap link_pairs = updater.compute(never_trials, min_collision_fraction,verbose);

    updater.write(link_pairs, include_default, include_always, output_path);


    return 0;
}
