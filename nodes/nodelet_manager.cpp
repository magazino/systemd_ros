#include <nodelet/loader.h>
#include <ros/ros.h>

void parse_remappings(int argc, char** argv, int& i, ros::M_string& remappings)
{
  remappings.clear();
  for (; i < argc; ++i)
  {
    std::string arg = argv[i];
    size_t pos = arg.find(":=");
    if (pos != std::string::npos)
      remappings[arg.substr(0, pos)] = arg.substr(pos + 2);
    else
      break;
  }
}

int main(int argc, char** argv)
{
  ros::M_string remappings;
  int i = 1;

  parse_remappings(argc, argv, i, remappings);

  ros::init(remappings, "manager");

  nodelet::Loader loader;
  std::vector<std::string> local_args_unused;
  std::string type, name;

  while (i < argc)
  {
    if (i + 2 > argc)
    {
      ROS_ERROR("Invalid number of arguments");
      return -1;
    }

    type = argv[i++];
    name = argv[i++];
    parse_remappings(argc, argv, i, remappings);

    if (!loader.load(name, type, remappings, local_args_unused))
    {
      ROS_ERROR("Error loading nodelet");
      return -1;
    }
  }

  ros::spin();
}
