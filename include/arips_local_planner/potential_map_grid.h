/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
#ifndef ARIPS_PLANNER_POTENTIAL_MAP_GRID_H_
#define ARIPS_PLANNER_POTENTIAL_MAP_GRID_H_

#include <vector>
#include <iostream>
#include <limits>
#include <base_local_planner/trajectory_inc.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <base_local_planner/map_cell.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

namespace arips_local_planner{    
  /**
   * @class PotentialMapGrid
   * @brief A grid of MapCell cells that is used to propagate path and goal distances for the trajectory controller.
   */
  class PotentialMapGrid{
    public:
      /**
       * @brief  Creates a 0x0 map by default
       */
      PotentialMapGrid();

      /**
       * @brief  Creates a map of size_x by size_y
       * @param size_x The width of the map 
       * @param size_y The height of the map 
       */
      PotentialMapGrid(unsigned int size_x, unsigned int size_y);


      /**
       * @brief  Returns a map cell accessed by (col, row)
       * @param x The x coordinate of the cell 
       * @param y The y coordinate of the cell 
       * @return A reference to the desired cell
       */
      inline base_local_planner::MapCell& operator() (unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }

      /**
       * @brief  Returns a map cell accessed by (col, row)
       * @param x The x coordinate of the cell 
       * @param y The y coordinate of the cell 
       * @return A copy of the desired cell
       */
      inline base_local_planner::MapCell operator() (unsigned int x, unsigned int y) const {
        return map_[size_x_ * y + x];
      }

      inline base_local_planner::MapCell& getCell(unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }

      /**
       * @brief  Destructor for a PotentialMapGrid
       */
      ~PotentialMapGrid(){}

      /**
       * @brief  Copy constructor for a PotentialMapGrid
       * @param mg The PotentialMapGrid to copy 
       */
      PotentialMapGrid(const PotentialMapGrid& mg);

      /**
       * @brief  Assignment operator for a PotentialMapGrid
       * @param mg The PotentialMapGrid to assign from 
       */
      PotentialMapGrid& operator= (const PotentialMapGrid& mg);

      /**
       * @brief reset path distance fields for all cells
       */
      void resetPathDist();

      /**
       * @brief  check if we need to resize
       * @param size_x The desired width
       * @param size_y The desired height
       */
      void sizeCheck(unsigned int size_x, unsigned int size_y);

      /**
       * @brief Utility to share initialization code across constructors
       */
      void commonInit();

      /**
       * @brief  Returns a 1D index into the MapCell array for a 2D index
       * @param x The desired x coordinate
       * @param y The desired y coordinate
       * @return The associated 1D index 
       */
      size_t getIndex(int x, int y);

      /**
       * return a value that indicates cell is in obstacle
       */
      inline double obstacleCosts() {
        return std::numeric_limits<double>::infinity();
      }

      /**
       * returns a value indicating cell was not reached by wavefront
       * propagation of set cells. (is behind walls, regarding the region covered by grid)
       */
      inline double unreachableCellCosts() {
        return map_.size() + 1;
      }
      
      /**
       * @brief Update what cell is considered the next local goal
       */
      void setLocalGoal(const costmap_2d::Costmap2D& costmap,
            const std::vector<geometry_msgs::PoseStamped>& global_plan);

      /**
       * @brief  Used to update the distance of a cell in path distance computation
       * @param  current_cell The cell we're currently in 
       * @param  check_cell The cell to be updated
       */
      void updatePathCell(base_local_planner::MapCell* current_cell, base_local_planner::MapCell* check_cell,
          const costmap_2d::Costmap2D& costmap, float dist);

      /**
       * increase global plan resolution to match that of the costmap by adding points linearly between global plan points
       * This is necessary where global planners produce plans with few points.
       * @param global_plan_in input
       * @param global_plan_output output
       * @param resolution desired distance between waypoints
       */
      static void adjustPlanResolution(const std::vector<geometry_msgs::PoseStamped>& global_plan_in,
            std::vector<geometry_msgs::PoseStamped>& global_plan_out, double resolution);

      /**
       * @brief  Compute the distance from each cell in the local map grid to the planned path
       * @param dist_queue A queue of the initial cells on the path 
       */
      void computeTargetDistance(const costmap_2d::Costmap2D& costmap);
      

      double goal_x_, goal_y_; /**< @brief The goal distance was last computed from */

      unsigned int size_x_, size_y_; ///< @brief The dimensions of the grid

    private:

      std::vector<base_local_planner::MapCell> map_; ///< @brief Storage for the MapCells
      
      using DistQueue = std::multimap<double, base_local_planner::MapCell*>;
      
      DistQueue dist_queue_;
      std::map<base_local_planner::MapCell*, DistQueue::iterator> dist_queue_locations_;

      void insertCellIntoQueue(base_local_planner::MapCell* cell);
      
      base_local_planner::MapCell* takeFirstFromQueue();
  };
};

#endif
