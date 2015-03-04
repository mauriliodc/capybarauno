# CapybaraUno
## Capybara Robot ros nodes for Indigo 

#### The capybara robot
the capybara robot is an open source and open hardware differential drive robot. Powered by a dspic33fj128mc802 it provides an highly customizable firmware, a full extensible serial commands stack and an high speed PID control loop up to 1khz for each motor.

The firmware source code is available https://github.com/mauriliodc/capybara

#### Nodes provided

* capybarauno_node

  the main node, it will subscribe to a capybarauno::ticks topic and communicate with the robot base requested a specific velocity (requested as left/right ticks) and publishes the robot ticks
* capybarauno_joy2ticks

  the node will subscribe to joy topic and convert the data from two axii into ticks
* capybarauno_twist2ticks

  the node will subscribe to a cmv_vel topic and convert the data into ticks
* capybarauno_ticks2odo

  the node subscribes to the published ticks from the main node and computes/publises the odometry
