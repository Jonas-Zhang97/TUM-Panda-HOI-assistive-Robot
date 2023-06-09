#ifndef rviz_scene_H
#define rviz_scene_h

#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class RvizScene
{
    private:
        //
    public:
        //
        moveit_visual_tools::MoveItVisualTools visual_tools;
        moveit::planning_interface::PlanningSceneInterface PSI;
        RvizScene():visual_tools("panda_link0"){};
};

#endif