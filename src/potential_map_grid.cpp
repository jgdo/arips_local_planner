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
#include <arips_local_planner/potential_map_grid.h>
#include <costmap_2d/cost_values.h>

#include <map>

using namespace std;
using base_local_planner::MapCell;

namespace arips_local_planner {

PotentialMapGrid::PotentialMapGrid()
    : size_x_(0), size_y_(0)
{
}

PotentialMapGrid::PotentialMapGrid(unsigned int size_x, unsigned int size_y)
    : size_x_(size_x), size_y_(size_y)
{
    commonInit();
}

PotentialMapGrid::PotentialMapGrid(const PotentialMapGrid& mg) {
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
}

void PotentialMapGrid::commonInit() {
    //don't allow construction of zero size grid
    ROS_ASSERT(size_y_ != 0 && size_x_ != 0);

    map_.resize(size_y_ * size_x_);

    //make each cell aware of its location in the grid
    for(unsigned int i = 0; i < size_y_; ++i) {
        for(unsigned int j = 0; j < size_x_; ++j) {
            unsigned int id = size_x_ * i + j;
            map_[id].cx = j;
            map_[id].cy = i;
        }
    }
}

size_t PotentialMapGrid::getIndex(int x, int y) {
    return size_x_ * y + x;
}

PotentialMapGrid& PotentialMapGrid::operator= (const PotentialMapGrid& mg) {
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
    return *this;
}

void PotentialMapGrid::sizeCheck(unsigned int size_x, unsigned int size_y) {
    if(map_.size() != size_x * size_y)
        map_.resize(size_x * size_y);

    if(size_x_ != size_x || size_y_ != size_y) {
        size_x_ = size_x;
        size_y_ = size_y;

        for(unsigned int i = 0; i < size_y_; ++i) {
            for(unsigned int j = 0; j < size_x_; ++j) {
                int index = size_x_ * i + j;
                map_[index].cx = j;
                map_[index].cy = i;
            }
        }
    }
}


inline void PotentialMapGrid::updatePathCell(MapCell* current_cell, MapCell* check_cell,
        const costmap_2d::Costmap2D& costmap, float dist) {
    
    if(check_cell->target_mark) {
        return;
    }

    //if the cell is an obstacle set the max path distance
    unsigned char check_cost = costmap.getCost(check_cell->cx, check_cell->cy);
    if(!getCell(check_cell->cx, check_cell->cy).within_robot &&
            (check_cost == costmap_2d::LETHAL_OBSTACLE ||
             check_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
             check_cost == costmap_2d::NO_INFORMATION)) {
        check_cell->target_dist = obstacleCosts();
        return;
    }

    double new_target_dist = current_cell->target_dist + (double)(check_cost)/150.0 + dist; // TODO values
    if (new_target_dist < check_cell->target_dist) {
        check_cell->target_dist = new_target_dist;
        
        // erase cell from queue
        auto loc_iter = dist_queue_locations_.find(check_cell);
        if(loc_iter != dist_queue_locations_.end()) {
            dist_queue_.erase(loc_iter->second);
            dist_queue_locations_.erase(loc_iter);
        }
        
        // and re-queue it with updated dist
        insertCellIntoQueue(check_cell);
    }
}


//reset the path_dist and goal_dist fields for all cells
void PotentialMapGrid::resetPathDist() {
    for(unsigned int i = 0; i < map_.size(); ++i) {
        map_[i].target_dist =  std::numeric_limits<double>::infinity(); //  unreachableCellCosts();
        map_[i].target_mark = false;
        map_[i].within_robot = false;
    }
}

void PotentialMapGrid::adjustPlanResolution(const std::vector<geometry_msgs::PoseStamped>& global_plan_in,
        std::vector<geometry_msgs::PoseStamped>& global_plan_out, double resolution) {
    if (global_plan_in.size() == 0) {
        return;
    }
    double last_x = global_plan_in[0].pose.position.x;
    double last_y = global_plan_in[0].pose.position.y;
    global_plan_out.push_back(global_plan_in[0]);

    double min_sq_resolution = resolution * resolution;

    for (unsigned int i = 1; i < global_plan_in.size(); ++i) {
        double loop_x = global_plan_in[i].pose.position.x;
        double loop_y = global_plan_in[i].pose.position.y;
        double sqdist = (loop_x - last_x) * (loop_x - last_x) + (loop_y - last_y) * (loop_y - last_y);
        if (sqdist > min_sq_resolution) {
            int steps = ceil((sqrt(sqdist)) / resolution);
            // add a points in-between
            double deltax = (loop_x - last_x) / steps;
            double deltay = (loop_y - last_y) / steps;
            // TODO: Interpolate orientation
            for (int j = 1; j < steps; ++j) {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = last_x + j * deltax;
                pose.pose.position.y = last_y + j * deltay;
                pose.pose.position.z = global_plan_in[i].pose.position.z;
                pose.pose.orientation = global_plan_in[i].pose.orientation;
                pose.header = global_plan_in[i].header;
                global_plan_out.push_back(pose);
            }
        }
        global_plan_out.push_back(global_plan_in[i]);
        last_x = loop_x;
        last_y = loop_y;
    }
}

//mark the point of the costmap as local goal where global_plan first leaves the area (or its last point)
void PotentialMapGrid::setLocalGoal(const costmap_2d::Costmap2D& costmap,
                                    const std::vector<geometry_msgs::PoseStamped>& global_plan) {
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    int local_goal_x = -1;
    int local_goal_y = -1;
    bool started_path = false;

    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());

    // skip global path points until we reach the border of the local map
    for (unsigned int i = 0; i < adjusted_global_plan.size(); ++i) {
        double g_x = adjusted_global_plan[i].pose.position.x;
        double g_y = adjusted_global_plan[i].pose.position.y;
        unsigned int map_x, map_y;
        if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
            local_goal_x = map_x;
            local_goal_y = map_y;
            started_path = true;
        } else {
            if (started_path) {
                break;
            }// else we might have a non pruned path, so we just continue
        }
    }
    if (!started_path) {
        ROS_ERROR("None of the points of the global plan were in the local costmap, global plan points too far from robot");
        return;
    }

    dist_queue_.clear();
    dist_queue_locations_.clear();

    if (local_goal_x >= 0 && local_goal_y >= 0) {
        MapCell& current = getCell(local_goal_x, local_goal_y);
        costmap.mapToWorld(local_goal_x, local_goal_y, goal_x_, goal_y_);
        current.target_dist = 0.0;
        insertCellIntoQueue(&current);
    }

    computeTargetDistance(costmap);
}



void PotentialMapGrid::computeTargetDistance(const costmap_2d::Costmap2D& costmap) {
    const auto last_col = size_x_ - 1;
    const auto last_row = size_y_ - 1;
    
    while(!dist_queue_.empty()) {
        MapCell* current_cell = takeFirstFromQueue();
        
        current_cell->target_mark = true;
        
        if(current_cell->cx > 0) {
            auto& nect_cell = getCell(current_cell->cx-1, current_cell->cy);
            updatePathCell(current_cell, &nect_cell, costmap, 1);
        }
        

        if(current_cell->cx < last_col) {
            auto& nect_cell = getCell(current_cell->cx+1, current_cell->cy);
            updatePathCell(current_cell, &nect_cell, costmap, 1);
        }

        if(current_cell->cy > 0) {
            auto& nect_cell = getCell(current_cell->cx, current_cell->cy-1);
            updatePathCell(current_cell, &nect_cell, costmap, 1);
        }

        if(current_cell->cy < last_row) {
           auto& nect_cell = getCell(current_cell->cx, current_cell->cy+1);
           updatePathCell(current_cell, &nect_cell, costmap, 1);
        }
        
        if(current_cell->cx > 0 && current_cell->cy > 0) {
           auto& nect_cell = getCell(current_cell->cx-1, current_cell->cy-1);
           updatePathCell(current_cell, &nect_cell, costmap, std::sqrt(2.0));
        }
        
        if(current_cell->cx > 0 && current_cell->cy < last_row) {
            auto& nect_cell = getCell(current_cell->cx-1, current_cell->cy+1);
            updatePathCell(current_cell, &nect_cell, costmap, std::sqrt(2.0));
        }
        
        if(current_cell->cx < last_col && current_cell->cy > 0) {
           auto& nect_cell = getCell(current_cell->cx+1, current_cell->cy-1);
           updatePathCell(current_cell, &nect_cell, costmap, std::sqrt(2.0));
        }
        
        if(current_cell->cx < last_col && current_cell->cy < last_row) {
           auto& nect_cell = getCell(current_cell->cx+1, current_cell->cy+1);
           updatePathCell(current_cell, &nect_cell, costmap, std::sqrt(2.0));
        }
        
        
    }
}

void PotentialMapGrid::insertCellIntoQueue(base_local_planner::MapCell* cell)
{
    auto iter = dist_queue_.insert({cell->target_dist, cell});
    dist_queue_locations_.insert({cell, iter});
}

base_local_planner::MapCell * PotentialMapGrid::takeFirstFromQueue()
{
    base_local_planner::MapCell* cell = dist_queue_.begin()->second;
    dist_queue_.erase(dist_queue_.begin());
    dist_queue_locations_.erase(cell);
    
    return cell;
}


};


