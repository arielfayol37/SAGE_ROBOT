
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class Nav2AsyncBridge(Node):
    def __init__(self, enqueue_arrival):
        super().__init__('nav2_llm_bridge_async')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None
        self.status = "idle"          # idle | navigating | arrived | failed | cancelling
        self.current_target = None
        self.last_feedback = {}
        self.enqueue_arrival = enqueue_arrival

    def set_goal(self, *, frame_id, x, y, ox, oy, oz, ow, location_name):
        self.current_target = location_name
        self.status = "navigating"

        # Preempt old goal if any
        if self.goal_handle:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass

        if not self.client.wait_for_server(timeout_sec=0.5):
            self.status = "failed"
            return "Nav2 action server not available."

        pose = PoseStamped()
        pose.header.frame_id = frame_id or 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.x = float(ox)
        pose.pose.orientation.y = float(oy)
        pose.pose.orientation.z = float(oz)
        pose.pose.orientation.w = float(ow)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        def _feedback_cb(fb):
            try:
                f = fb.feedback
                # available: current_pose, navigation_time, number_of_recoveries, distance_remaining
                self.last_feedback = {
                    "distance_remaining": getattr(f, "distance_remaining", 0.0),
                    "recoveries": getattr(f, "number_of_recoveries", 0),
                }
            except Exception:
                pass

        send_fut = self.client.send_goal_async(goal, feedback_callback=_feedback_cb)

        def _after_send(fut):
            try:
                gh = fut.result()
                if not gh or not gh.accepted:
                    self.status = "failed"
                    return
                self.goal_handle = gh
                res_fut = gh.get_result_async()

                def _after_result(rf):
                    try:
                        res = rf.result()                    # GetResult.Response
                        st = res.status                      # int (GoalStatus.*)
                        if st == GoalStatus.STATUS_SUCCEEDED:
                            self.status = "arrived"
                            try:
                                self.enqueue_arrival(self.current_target)
                            except Exception:
                                pass
                        elif st == GoalStatus.STATUS_CANCELED:
                            self.status = "idle"
                        else:
                            # ABORTED or others
                            self.status = "failed"
                    except Exception:
                        self.status = "failed"

                res_fut.add_done_callback(_after_result)
            except Exception:
                self.status = "failed"

        send_fut.add_done_callback(_after_send)
        return "Goal accepted (navigating)."

    def cancel_goal(self):
        if not self.goal_handle:
            return "No active goal."
        self.status = "cancelling"
        fut = self.goal_handle.cancel_goal_async()
        def _after_cancel(_):
            self.status = "idle"
            self.goal_handle = None
        fut.add_done_callback(_after_cancel)
        return "Cancel requested."
