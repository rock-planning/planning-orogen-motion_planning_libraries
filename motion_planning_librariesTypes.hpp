#ifndef motion_planning_libraries_TYPES_HPP
#define motion_planning_libraries_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace motion_planning_libraries {

enum TRAV_MAP_MODE {
    CLEAR,
    RANDOM_CIRCLES,
    RANDOM_RECTANGLES,
    SMALL_OPENING,
    PARKING_SPACE
};

}

#endif

