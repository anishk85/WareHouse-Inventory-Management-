# 🗺️ Nav2 Waypoint Following: Two Approaches Compared

## Overview

We have implemented **two methods** for autonomous waypoint navigation with inventory tasks:

1. **Manual NavigateToPose** (`waypoint_follower_node.py`)  
2. **Nav2 FollowWaypoints** (`nav2_waypoint_follower.py`) ← **RECOMMENDED**

---

## 📊 Comparison

| Feature | Manual NavigateToPose | Nav2 FollowWaypoints |
|---------|---------------------|---------------------|
| **Complexity** | Higher (manual state machine) | Lower (Nav2 handles it) |
| **Code Lines** | ~285 lines | ~320 lines (but simpler logic) |
| **Reliability** | Manual error handling | Built-in Nav2 recovery |
| **Waypoint Handling** | One at a time | All at once |
| **Task Execution** | Custom implementation | Can use Nav2 task executor plugins |
| **Progress Feedback** | Custom status messages | Built-in feedback |
| **Recovery** | Manual implementation needed | Automatic via Nav2 |
| **Flexibility** | Full control | Some constraints |
| **Best For** | Custom behaviors | Standard waypoint following |

---

## 🔧 Method 1: Manual NavigateToPose

### How It Works

```python
# Send one goal at a time
goal = NavigateToPose.Goal()
goal.pose = waypoint_1
nav_client.send_goal_async(goal, callback=on_reached)

def on_reached(result):
    # Do inventory task
    lift_actuator()
    scan_qr_codes()
    lower_actuator()
    
    # Send next goal
    goal.pose = waypoint_2
    nav_client.send_goal_async(goal, callback=on_reached)
```

### Flow Diagram

```
IDLE
  ↓
START QR DETECTION (once, runs forever)
  ↓
SEND GOAL → waypoint_1
  ↓
NAVIGATING (Nav2 controls robot)
  ↓
REACHED waypoint_1 (callback triggered)
  ↓
LIFT ACTUATOR (5s)
  ↓
WAIT & SCAN (50s)
  ↓
LOWER ACTUATOR (5s)
  ↓
SEND GOAL → waypoint_2
  ↓
(repeat for each waypoint)
```

### Pros
- ✅ Full control over every step
- ✅ Custom task execution timing
- ✅ Can modify behavior between waypoints
- ✅ Clear state machine logic

### Cons
- ❌ More code to maintain
- ❌ Manual error handling
- ❌ Must handle nav failures ourselves
- ❌ Callback-based (can be tricky)

---

## 🚀 Method 2: Nav2 FollowWaypoints (RECOMMENDED)

### How It Works

```python
# Send ALL waypoints at once
goal = FollowWaypoints.Goal()
goal.poses = [waypoint_1, waypoint_2, waypoint_3, ...]
follow_waypoints_client.send_goal_async(goal)

# Nav2 navigates through all waypoints sequentially
# At each waypoint, Nav2 can trigger a task executor plugin
# OR we subscribe to waypoint arrival events
```

### Flow Diagram

```
IDLE
  ↓
START QR DETECTION (once, runs forever)
  ↓
SEND ALL WAYPOINTS → Nav2
  ↓
Nav2 NAVIGATING
  ├─> waypoint_1
  │     ↓
  │   ARRIVED (Nav2 signals)
  │     ↓
  │   EXECUTE TASK (lift→scan→lower)
  │     ↓
  ├─> waypoint_2
  │     ↓
  │   ARRIVED (Nav2 signals)
  │     ↓
  │   EXECUTE TASK
  │     ↓
  ├─> waypoint_3
  └─> ...
        ↓
      COMPLETE (Nav2 signals with result)
```

### Pros
- ✅ **Simpler code** - Nav2 handles navigation
- ✅ **Built-in recovery** - Nav2 retries failures
- ✅ **Send all waypoints once** - no manual sequencing
- ✅ **Feedback built-in** - current waypoint index
- ✅ **Industry standard** - widely used pattern
- ✅ **Plugin support** - can use Nav2 task executor plugins

### Cons
- ❌ Less granular control between waypoints
- ❌ Harder to do complex conditional logic
- ❌ Must work within Nav2's framework

---

## 🎯 Which One to Use?

### Use **Nav2 FollowWaypoints** (`nav2_waypoint_follower.py`) if:
- ✅ Standard waypoint following with simple tasks
- ✅ You want robustness and automatic recovery
- ✅ You're deploying in production
- ✅ You want less code to maintain
- ✅ **This is the recommended approach for most use cases**

### Use **Manual NavigateToPose** (`waypoint_follower_node.py`) if:
- ✅ You need complex logic between waypoints
- ✅ Waypoints are conditional/dynamic
- ✅ You need fine-grained control
- ✅ You're prototyping/experimenting

---

## 🔄 How They Handle the Inventory Task

Both approaches execute the same inventory task at each waypoint, but trigger it differently:

### Manual Approach
```python
def nav_result_callback(self, future):
    """Called when Nav2 reaches waypoint"""
    if result.success:
        self.state = TaskState.REACHED_WAYPOINT

def state_machine(self):
    if self.state == TaskState.REACHED_WAYPOINT:
        self.execute_inventory_task()  # Lift, scan, lower
```

### Nav2 Approach
```python
def feedback_callback(self, feedback_msg):
    """Nav2 reports current waypoint"""
    current_wp = feedback_msg.feedback.current_waypoint
    if current_wp != self.last_waypoint:
        # New waypoint reached!
        # But Nav2 is still navigating...
        
# Better: Use waypoint task executor callback
def waypoint_task_callback(self, msg):
    """Nav2's task executor signals arrival"""
    if msg.data == "arrived":
        self.execute_inventory_task()  # Lift, scan, lower
```

---

## 📝 Nav2 Waypoint Follower Architecture

```
┌─────────────────────────────────────────────┐
│  Your Node                                  │
│  (nav2_waypoint_follower.py)               │
│                                             │
│  - Loads waypoints from YAML               │
│  - Enables QR detection                     │
│  - Sends ALL waypoints to Nav2             │
│  - Listens for task executor events        │
│  - Executes inventory tasks                │
└──────────────┬──────────────────────────────┘
               │
               │ FollowWaypoints action
               ▼
┌─────────────────────────────────────────────┐
│  Nav2 waypoint_follower Server             │
│  (built into Nav2)                          │
│                                             │
│  - Receives list of waypoints               │
│  - Navigates to each sequentially           │
│  - Reports progress via feedback            │
│  - Triggers task executor at each waypoint  │
│  - Handles recovery from failures           │
└──────────────┬──────────────────────────────┘
               │
               │ NavigateToPose actions
               ▼
┌─────────────────────────────────────────────┐
│  Nav2 bt_navigator Server                   │
│  (behavior tree navigation)                 │
│                                             │
│  - Path planning                            │
│  - Obstacle avoidance                       │
│  - Recovery behaviors                       │
│  - Controller execution                     │
└─────────────────────────────────────────────┘
```

---

## 🚦 Nav2 Waypoint Task Executor Plugin

Nav2's waypoint_follower can execute tasks at each waypoint using plugins:

```yaml
# waypoint_follower_params.yaml
waypoint_follower:
  ros__parameters:
    waypoint_task_executor_plugin: "wait_at_waypoint"
    
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 60000  # 60 seconds
```

**Available Plugins:**
1. **WaitAtWaypoint** - Pause for X milliseconds
2. **PhotoAtWaypoint** - Take a photo
3. **Custom Plugin** - You can create your own!

**For our use case:**
- We use `WaitAtWaypoint` to pause for ~60 seconds
- During this pause, we execute the inventory task
- QR detection runs continuously in background

---

## 🎮 Launch Commands

### Method 1: Manual NavigateToPose
```bash
ros2 launch mecanum_hardware inventory_mission.launch.py \
    waypoint_follower:=manual \
    waypoints:=/tmp/waypoints.yaml
```

### Method 2: Nav2 FollowWaypoints
```bash
ros2 launch mecanum_hardware inventory_mission.launch.py \
    waypoint_follower:=nav2 \
    waypoints:=/tmp/waypoints.yaml
```

---

## 📊 Performance Comparison

| Metric | Manual | Nav2 |
|--------|--------|------|
| Lines of Code | 285 | 320 |
| State Machine Complexity | High | Low |
| Error Recovery | Manual | Automatic |
| Time to Implement | Longer | Shorter |
| Maintenance | More effort | Less effort |
| Flexibility | High | Medium |
| Robustness | Manual | High |

---

## 🎓 Recommendation

**For the warehouse inventory system, use Nav2 FollowWaypoints** because:

1. ✅ **More reliable** - Built-in recovery behaviors
2. ✅ **Less code** - Nav2 does the heavy lifting
3. ✅ **Industry standard** - Well-tested and documented
4. ✅ **Easier maintenance** - Less custom code to debug
5. ✅ **Better monitoring** - Built-in feedback and status

The manual approach is great for learning and understanding how navigation works,
but Nav2's waypoint follower is designed exactly for this use case.

---

## 🔗 Resources

- [Nav2 Waypoint Follower Docs](https://navigation.ros.org/tutorials/docs/navigation2_with_keepout_filter.html)
- [Nav2 Action Servers](https://navigation.ros.org/behavior_trees/index.html)
- [Writing Custom Task Executor Plugins](https://navigation.ros.org/plugin_tutorials/docs/writing_new_waypoint_plugin.html)

---

**Bottom Line:** Both work, but `nav2_waypoint_follower.py` is the better choice for production! 🚀
