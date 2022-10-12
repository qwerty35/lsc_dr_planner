{
  "quadrotors": {
    "crazyflie": {
      "max_vel": [1.0, 1.0, 1.0], # Maximum velocity of the crazyflie [v_min_x, v_min_y, v_min_z], [m/s]
      "max_acc": [2.0, 2.0, 2.0], # Maximum acceleration of the crazyflie [a_min_x, a_min_y, a_min_z], [m/s^2]
      "radius": 0.15, # Radius of the crazyflie, [m]
      "nominal_velocity": 1.0, # Not used in this work
      "downwash": 2.0}, # Downwash coefficient of the crazyflie
    "default": {
      "max_vel": [1.0, 1.0, 1.0],
      "max_acc": [2.0, 2.0, 2.0],
      "radius": 0.15,
      "nominal_velocity": 1.0,
      "downwash": 2.0}
  },

  "world": [
    {"dimension": [-2.0, -0.3, 0.0, 6.0, 4.3, 2.5]} # Size of the world [x_min, y_min, z_min, x_max, y_max, z_max], [m]
  ],

  "agents": [
    # type: quadrotor type (crazyflie or default), cid: crazyflie id for experiment (Not equal to the agent id used in the planner and rviz)
    # start: start point of the agent [x,y,z] [m], goal: goal point of the agent [x,y,z] [m]
     {"type": "crazyflie", "cid": 1, "start": [-1, 3.0, 1], "goal": [5, 1.0, 1]},
     {"type": "crazyflie", "cid": 2, "start": [-1, 2.5, 1], "goal": [5, 1.5, 1]},
     {"type": "crazyflie", "cid": 3, "start": [-1, 2.0, 1], "goal": [5, 2.0, 1]},
     {"type": "crazyflie", "cid": 4, "start": [-1, 1.5, 1], "goal": [5, 2.5, 1]},
     {"type": "crazyflie", "cid": 5, "start": [-1, 1.0, 1], "goal": [5, 3.0, 1]},
     {"type": "crazyflie", "cid": 6, "start": [5, 3.0, 1], "goal": [-1, 1.0, 1]},
     {"type": "crazyflie", "cid": 7, "start": [5, 2.5, 1], "goal": [-1, 1.5, 1]},
     {"type": "crazyflie", "cid": 8, "start": [5, 2.0, 1], "goal": [-1, 2.0, 1]},
     {"type": "crazyflie", "cid": 9, "start": [5, 1.5, 1], "goal": [-1, 2.5, 1]},
     {"type": "crazyflie", "cid": 10, "start": [5, 1.0, 1], "goal": [-1, 3.0, 1]}
  ],

  # not used in this package
  "obstacles": [
  ]
}