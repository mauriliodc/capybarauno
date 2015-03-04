# CapybaraUno
## Capybara Robot ros nodes for Indigo 

### Inside:
* capybarauno_node

  the main node, it will subscribe to a capybarauno::ticks topic and communicate with the robot base requested a specific velocity (requested as left/right ticks) and publishes the robot ticks
* capybarauno_joy2ticks

  the node will subscribe to joy topic and convert the data from two axii into ticks
* capybarauno_twist2ticks

  the node will subscribe to a cmv_vel topic and convert the data into ticks
* capybarauno_ticks2odo

  the node subscribes to the published ticks from the main node and computes/publises the odometry
