# This module is for the controller
# By Adam Gardner, St. Peter's College. 2024

# Change as required
Kp = 0.15
Kd = 0.6

def controller(depth, depth_LastTime, time_step, reference_depth):
    error = reference_depth[time_step] - depth
    error_LastTime = reference_depth[time_step - 1] - depth_LastTime

    controller_output = Kp * error + Kd * error_LastTime

    return controller_output