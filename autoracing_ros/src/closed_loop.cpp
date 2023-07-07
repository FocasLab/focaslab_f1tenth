/*
 * File Name: closed_loop.cpp
 *
 * Edited: Allen Emmanuel Binny
 */
// C++ headers
#include <iostream>
#include <fstream>
#include <vector>

// ros includes
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

// rviz visualization
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

// map subscriber
std::string map_topic_name = "/map";
ros::Subscriber map_sub;

// state space
std::vector<int> map_vector;
double resolution;
int width, height;

// global variables
static const int state_dim = 3;
static const int input_dim = 2;
static constexpr double tau = 0.08;

int flag = 1;

int returnObstacle(std::vector<std::vector<int>> &maps, int i, int j, int gridl)
{
	for (int k = i; k < i + gridl; k++)
	{
		for (int l = j; l < j + gridl; l++)
		{
			if (maps[k][l])
			{
				return 1;
			}
		}
	}
	return 0;
}

void formMap(std::vector<std::vector<int>> &maps)
{
	std::fstream my_file;
	std::cout << "It has entered";
	my_file.open("/home/allemmbinn/Documents/mapData.txt", std::ios::out);
	if (!my_file)
		std::cout << "Error in file creation!";
	else
		std::cout << "File Creation successfull.";

	int count = 1;

	std::string s1 = "ob";
	std::string s3 = "{type=\"rectangle\"; h=\"{";
	std::string s4 = ",";
	std::string s5 = "} ,{";
	std::string s6 = "} ,{-3.4,3.4}\";}";

	int gridl = (0.6) / resolution;
	int y = 0; // Last location of y found
	for (int i = 0; i < width; i += gridl)
	{
		for (int j = 0; j < height; j += gridl)
		{
			if (returnObstacle(maps, i, j, gridl))
			{
				std::string s2 = s1 + std::to_string(count++) + s3 + std::to_string(i * resolution) + s4 + std::to_string((i + gridl) * resolution) + s5 + std::to_string(j * resolution) + s4 + std::to_string((j + gridl) * resolution) + s6;
				my_file << s2 << std::endl;
			}
		}
	}
}

std::vector<std::vector<int>> getMapMatrix(const std::vector<int> &map_vector, int width, int height)
{
	std::vector<std::vector<int>> map;

	for (int i = 0; i < width; i++)
	{
		std::vector<int> map_i;
		for (int j = 0; j < height; j++)
		{
			int idx = height * i + j;
			if (map_vector[idx] > 0)
				map_i.push_back(1);
			else
				map_i.push_back(0);
		}
		map.push_back(map_i);
	}
	return map;
}

void mapCallback(const nav_msgs::OccupancyGrid &msg)
{
	map_vector.clear();
	resolution = msg.info.resolution;
	width = msg.info.width;
	height = msg.info.height;

	flag = 0;

	for (int i = 0; i < width * height; i++)
		map_vector.push_back(msg.data[i]);
}

int main(int argc, char **argv)
{
	// ros node initialize
	ros::init(argc, argv, "closed_loop");
	std::cout<<"Is it working";

	ros::NodeHandle nh_;

	map_sub = nh_.subscribe(map_topic_name, 10, mapCallback);

	while(flag)
	{
		ros::Duration(1, 0).sleep();
		std::cout << "stuck";
	}
	std::cout << "Outside";
	std::vector<std::vector<int>> maps = getMapMatrix(map_vector, width, height);
	formMap(maps);

	ros::spinOnce();
	// ros::spin();

	return 0;
}