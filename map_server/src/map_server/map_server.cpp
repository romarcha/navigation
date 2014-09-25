/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#include <map_server/map_server/map_server.h>

/**
 * \brief Returns the value of a parameter or a default value.
 *
 * \param key the key of the parameter
 * \param default_value the default value to use in case the parameter
 *      does not exist
 * \return value of the parameter
 */
template<typename T>
T get_param(std::string const& key, T default_value)
{
    ros::NodeHandle node("~");

    T value;
    node.param(key, value, default_value);
    return value;
}

MapServer::MapServer(const std::string& fname)
{
    std::string mapfname = "";
    double origin[3];
    int negate;
    double occ_th, free_th;
    double res;

    std::string frame_id = get_param<std::string>("frame_id","map");
    std::string topic_name = get_param<std::string>("topic_id","map");


    std::ifstream fin(fname.c_str());
    if (fin.fail())
    {
        ROS_ERROR("Map_server could not open %s.", fname.c_str());
        exit(-1);
    }

    // Read the .yaml file using the new yaml library.
    YAML::Node yaml_file;
    try
    {
        yaml_file = YAML::LoadFile(fname);
    }
    catch (YAML::Exception &e)
    {
        ROS_ERROR(e.what());
        exit(-1);
    }

    // Read resolution
    try
    {
        res = yaml_file["resoltuion"].as<double>();
    }
    catch (YAML::Exception &e)
    {
        ROS_ERROR("Error reading map resolution.");
        ROS_ERROR_STREAM("YAML error: " << e.what());
        exit(-1);
    }

    // Read resolution
    try
    {
        res = yaml_file["resoltuion"].as<double>();
    }
    catch (YAML::Exception &e)
    {
        ROS_ERROR("Error reading map resolution.");
        ROS_ERROR_STREAM("YAML error: " << e.what());
        exit(-1);
    }

    // Read negate
    try
    {
        negate = yaml_file["negate"].as<int>();
    }
    catch (YAML::Exception &e)
    {
        ROS_ERROR("Error reading negate.");
        ROS_ERROR_STREAM("YAML error: " << e.what());
        exit(-1);
    }

    // Read occupied threshold
    try
    {
        occ_th = yaml_file["occupied_thresh"].as<double>();
    }
    catch (YAML::Exception &e)
    {
        ROS_ERROR("Error reading occupied threshold.");
        ROS_ERROR_STREAM("YAML error: " << e.what());
        exit(-1);
    }
    /*
    try {
      doc["occupied_thresh"] >> occ_th;
    } catch (YAML::InvalidScalar) {
      ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
      exit(-1);
    }
    try {
      doc["free_thresh"] >> free_th;
    } catch (YAML::InvalidScalar) {
      ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
      exit(-1);
    }
    try {
      doc["origin"][0] >> origin[0];
      doc["origin"][1] >> origin[1];
      doc["origin"][2] >> origin[2];
    } catch (YAML::InvalidScalar) {
      ROS_ERROR("The map does not contain an origin tag or it is invalid.");
      exit(-1);
    }
    try {
      doc["image"] >> mapfname;
      // TODO: make this path-handling more robust
      if(mapfname.size() == 0)
      {
        ROS_ERROR("The image tag cannot be an empty string.");
        exit(-1);
      }
      if(mapfname[0] != '/')
      {
        // dirname can modify what you pass it
        char* fname_copy = strdup(fname.c_str());
        mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
        free(fname_copy);
      }
    }
    catch (YAML::InvalidScalar)
    {
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        exit(-1);
    }

    ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
    map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin);
    map_resp_.map.info.map_load_time = ros::Time::now();
    map_resp_.map.header.frame_id = frame_id;
    map_resp_.map.header.stamp = ros::Time::now();
    ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
           map_resp_.map.info.width,
           map_resp_.map.info.height,
           map_resp_.map.info.resolution);
    meta_data_message_ = map_resp_.map.info;

    service = n.advertiseService("static_map", &MapServer::mapCallback, this);

    // Latched publisher for metadata
    metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    metadata_pub.publish( meta_data_message_ );

    // Latched publisher for data
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    map_pub.publish( map_resp_.map );
    */
}

bool MapServer::mapCallback(nav_msgs::GetMap::Request  &req,
                 nav_msgs::GetMap::Response &res )
{
  // request is empty; we ignore it

  // = operator is overloaded to make deep copy (tricky!)
  res = map_resp_;
  ROS_INFO("Sending map");

  return true;
}
