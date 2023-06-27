/*
 * File Name: closed_loop.cpp
 * 
 * Edited: Allen Emmanuel Binny
*/

// ros includes
#include <ros/ros.h>

// rviz visualization
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

// scots includes 
#include "autoracing_core/scots.hpp"
#include "autoracing_core/RungeKutta4.hpp"
#include "autoracing_core/TicToc.hpp"


class ClosedLoop{
    protected:
        ros::NodeHandle nh_;

        // state space
		std::vector<int> map_eta_vector;
		double resolution;
		int width, height;

        // global variables
		static const int state_dim = 3;
		static const int input_dim = 2;
		static constexpr double tau = 0.08;

        // Alias
        using state_type = std::array<double, state_dim>;
		using input_type = std::array<double, input_dim>;
		using abs_type = scots::abs_type;

    public:
        ClosedLoop(){
        }

		abs_type returnEta(const scots::UniformGrid &ss, std::vector<std::vector<int>> &maps, state_type x){
			// ratio of scots grid(s_eta) to map grid (resolution)
			// 0.2 is added for floating point numbers
			std::vector<int> grid_ratio{3*int((ss.get_eta()[0] / resolution) + 0.2), 3*int((ss.get_eta()[1] / resolution) + 0.2)};

			// coordinates to search in map matrix
			// 0.2 is added for floating point numbers.
			std::vector<int> cord{int((x[0] / resolution) + 0.2), int((x[1] / resolution) + 0.2)};

			for(int i = -1; i < grid_ratio[1] + 1; i++) {
				for(int j = -1; j < grid_ratio[0] + 1; j++) {
					if(cord[1] + i >= 0 && cord[1] + i< height && cord[0] + j >= 0 && cord[0] + j < width) {
						if(maps[cord[1] + i][cord[0] + j] != 0){
							return 1;
						}
					}
				}
			}
			return 0;
		}

        void formMap(const scots::UniformGrid &ss, std::vector<std::vector<int>> &maps){
            abs_type num_cell = ss.size();
			std::vector<abs_type> NN = ss.get_nn();

			std::cout << "Number of cells: " << num_cell << std::endl;

			// check for only (x, y) state space (num_grid_x * num_grid_y)
			for(abs_type i = 0; i < NN[2]; i++) {
				state_type x;
				ss.itox(i, x);

				
			}
        }
}