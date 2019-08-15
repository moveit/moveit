#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_utils/stl_to_string.hpp>

namespace util
{
namespace po = boost::program_options;

struct ParameterBase
{
  std::string m_name;
  std::string m_desc;
  ParameterBase(const std::string& name, const std::string& desc) : m_name(name), m_desc(desc) {}
  virtual void addToBoost(po::options_description&) = 0;
  virtual ~ParameterBase() {}
};
typedef std::shared_ptr<ParameterBase> ParameterBasePtr;

template <typename T>
struct ParameterVec : ParameterBase
{
  std::vector<T>* m_value;
  ParameterVec(std::string name, std::vector<T>* value, std::string desc) : ParameterBase(name, desc), m_value(value) {}
  void addToBoost(po::options_description& od)
  {
    od.add_options()(
        m_name.c_str(), po::value(m_value)->default_value(*m_value, Str(*m_value))->multitoken(), m_desc.c_str());
  }
};

template <typename T>
struct Parameter : ParameterBase
{
  T* m_value;
  Parameter(std::string name, T* value, std::string desc) : ParameterBase(name, desc), m_value(value) {}
  void addToBoost(po::options_description& od)
  {
    od.add_options()(m_name.c_str(), po::value(m_value)->default_value(*m_value, Str(*m_value)), m_desc.c_str());
  }
};

struct Config
{
  std::vector<ParameterBasePtr> params;
  void add(ParameterBase* param) { params.push_back(ParameterBasePtr(param)); }
};

struct CommandParser
{
  std::vector<Config> configs;
  void addGroup(const Config& config) { configs.push_back(config); }
  CommandParser(const Config& config) { addGroup(config); }
  void read(int argc, char* argv[]);
};
}
