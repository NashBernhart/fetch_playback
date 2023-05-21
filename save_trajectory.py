# This is just a snippet for testing purposes to be included in other playback files
# The purpose is to extract the message to be saved in a JSON file and later reassmbled into the message when it is to be replayed


from moveit_msgs.msg import RobotTrajectory

# arrays to hold each portion of each message

# arrs to hold jt portions of message
jt_names = []
# jt.points portions
jt_p_pos = []
jt_p_vel = []
jt_p_acc = []
jt_p_eff = []
jt_p_tfs = []
# header portions
jt_h_seq = []
jt_h_ts =[]
jt_h_frame = []

# arrs to hold multi dof portions



(fraction, plan) = group.compute_cartesian_path([waypoints])

# get portions of message

# jt message
jt = plan.joint_trajectory
jt_points = jt.points
jt_header = jt.header

# multidof message
multi_dof = plan.multi_dof_joint_trajectory

# appending to arrs
jt_h_seq.append(jt_header.seq)
jt_h_frame.append(jt_header.frame_id)
jt_h_ts.append(jt_header.stamp)
jt_p_pos.append(jt_points.positions)
jt_p_vel.append(jt_points.velocities)
jt_p_acc.append(jt_points.accelerations)
jt_p_eff.append(jt_points.effort)
jt_p_tfs.append(jt_points.time_from_start)






