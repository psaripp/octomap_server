#include <fstream>

#include "ros/console.h"
#include "ros/ros.h"

#include <octomap/OcTreeBase.h>
#include <octomap/octomap.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

void printUsage(const char *programName) {
  ROS_ERROR("Usage: %s [-f | -b] filename", programName);
  ROS_ERROR("    -f forces to send a full map");
  ROS_ERROR("    -b forces to send a binary map");
  ROS_ERROR("    -l latch published message");
}

enum class SendType { BINARY, FULL, AUTO };

enum class TreeType { BINARY_TREE, OC_TREE, INVALID_TREE };

int main(int argc, char **argv) {
  ros::init(argc, argv, "hello_world_map_server",
            ros::init_options::AnonymousName);

  // Quick and dirty argument parser
  if (argc < 2) {
    printUsage(argv[0]);
    return -1;
  }

  std::string filename;
  bool isLatch = false;
  auto sendType = SendType::AUTO;
  auto treeType = TreeType::INVALID_TREE;
  {
    bool done = false;
    size_t index = 1;
    while (!done) {
      const auto arg = std::string(argv[index]);
      if (arg == "-l") {
        isLatch = true;
      } else if (arg == "-b") {
        sendType = SendType::BINARY;
      } else if (arg == "-f") {
        sendType = SendType::FULL;
      } else {
        filename = argv[index];
        done = true;
      }

      ++index;

      done |= index >= argc;
    };

    if (filename.empty()) {
      ROS_ERROR("No input filename was given");
      printUsage(argv[0]);
      return -1;
    }

    // pre-determine file type
    const auto extension = filename.substr(filename.find_last_of('.') + 1);

    if (extension == "bt") {
      treeType = TreeType::BINARY_TREE;
    } else if (extension == "ot") {
      treeType = TreeType::OC_TREE;
    }

    if (treeType == TreeType::INVALID_TREE) {
      ROS_ERROR("Invalid file type. Extension should be either .ot or .bt");
      printUsage(argv[0]);
      return -1;
    }
  }

  // Setup publisher, read file and send tree over
  try {
    ros::NodeHandle nodeHandle; // w/o namespace
    auto octomapPublisher =
        nodeHandle.advertise<octomap_msgs::Octomap>("octomap", 1, isLatch);

    //
    octomap_msgs::Octomap octomapMsg;

    if (treeType == TreeType::OC_TREE) {
      // 1. File type was an octree

      const auto ocTreeBase = std::shared_ptr<octomap::AbstractOcTree>(
          octomap::AbstractOcTree::read(filename));

      if (sendType == SendType::BINARY) {
        // Forced to send as binary
        const auto ocTree =
            std::dynamic_pointer_cast<octomap::OcTree>(ocTreeBase);
        if (ocTree) {
          ROS_INFO("Send occupancy tree as binary");
          octomap_msgs::binaryMapToMsg(*ocTree, octomapMsg);
        } else {
          throw std::runtime_error("Cannot publish tree");
        }

      } else {
        // Send as normally would otherwise
        ROS_INFO("Send occupancy tree as full map");
        octomap_msgs::fullMapToMsg(*ocTreeBase, octomapMsg);
      }

    } else {
      // 2. File type was a binary tree

      auto ocTree = octomap::OcTree(filename);

      if (sendType == SendType::FULL) {
        // Forced to send as full map
        ROS_INFO("Send binary tree as full map");
        octomap_msgs::fullMapToMsg(ocTree, octomapMsg);
      } else {
        // send as binary
        ROS_INFO("Send binary tree as binary");
        octomap_msgs::binaryMapToMsg(ocTree, octomapMsg);
      }
    }


    octomapMsg.header.frame_id = "base_link";
    octomapPublisher.publish(octomapMsg);

    ROS_INFO("OK. Message length = %d bytes", octomapMsg.data.size());

    ros::spin();
  } catch (std::runtime_error &e) {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
