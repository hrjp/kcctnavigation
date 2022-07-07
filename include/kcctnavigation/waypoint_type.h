#pragma once
#include <string>

//addition status
enum class waypoint_type{
    skip,
    normal,
    precision,
    person_detection,
    recursion_start,
    recursion_end
};

std::string waypoint_type_str(waypoint_type status){
    switch (status){
        case(waypoint_type::skip):
            return "skip";
        case(waypoint_type::normal):
            return "normal";
        case(waypoint_type::precision):
            return "precision";
        case(waypoint_type::person_detection):
            return "person_detection";
        case(waypoint_type::recursion_start):
            return "recursion_start";
        case(waypoint_type::recursion_end):
            return "recursion_end"; 
        default:
            return "";
    }
}