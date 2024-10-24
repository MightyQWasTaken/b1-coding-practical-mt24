# This module is for the controller
# By Adam Gardner, St. Peter's College. 2024

# Change as required
Kp = 0.17
Kd = 2.75

def pdcontroller(depth, depth_LastTime, time_step, reference_depth):
    error = reference_depth[time_step] - depth
    error_LastTime = reference_depth[time_step - 1] - depth_LastTime

    controller_output = Kp * error + Kd * (error - error_LastTime)

    return controller_output