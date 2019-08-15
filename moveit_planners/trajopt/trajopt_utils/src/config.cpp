#include <trajopt_utils/config.hpp>

namespace util
{
void CommandParser::read(int argc, char* argv[])
{
  // create boost options_description based on variables, parser
  po::options_description od;
  od.add_options()("help,h", "produce help message");
  for (std::size_t i = 0; i < configs.size(); ++i)
  {
    for (std::size_t j = 0; j < configs[i].params.size(); ++j)
    {
      configs[i].params[j]->addToBoost(od);
    }
  }
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(od).run(), vm);
  if (vm.count("help"))
  {
    std::cout << "usage: " << argv[0] << " [options]" << std::endl;
    std::cout << od << std::endl;
    exit(0);
  }
  po::notify(vm);
}
}
