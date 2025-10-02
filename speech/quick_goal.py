import sys
from llm_streaming import start_ros_background, _nav_node, WAYPOINTS  

if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "Fites"
    start_ros_background()
    wp = WAYPOINTS[name]
    print(_nav_node.set_goal(
        frame_id=wp.get("frame_id","map"),
        x=wp["x"], y=wp["y"], ox=wp["ox"], oy=wp["oy"], oz=wp["oz"], ow=wp["ow"],
        location_name=name
    ))
    input("Press Enter to cancel...\n")
    print(_nav_node.cancel_goal())
