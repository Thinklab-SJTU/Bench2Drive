## Latest changes

* Added support to ROS agents, which are meant to inherit from the new `ROS1Agent` and `ROS2Agent` agents. The old `RosAgent` has been deleted.
* Improved the format of the printed runtime information
* Added optional side mirrors to the hyman agent
* Improved the performance of the routes by initializing the scenarios during runtime
* Improved result writer output, in the same wasy as ScenarioRunner's one.
* Improved the initialization and cleanup of the Leaderboard.
* Improved the robustness of the resuming functionality
* Improved the example Dockerfile and added new example for ROS based agents
* Added new utility scripts:
    - merge_statistics.py: join two or more json results into one
    - route_creator.py: simplifies the creation of new routes
    - route_displayer.py: parse and debug route xml files from inside the simulator
    - route_summarizer.py: parses the route xml file into a table.
    - scenario_creator.py: uses the spectator and terminal inputs to automatically add scenarios to a route
    - scenario_orderer.py: modifies the scenarios part of the route file to be ordered according to their route's position
    - weather_creator.py: gets the current simulation's weather in the route format for easy copy.
* The `StatisticsManager` class has been remade to add robustness, remove unneeded complexity and hardcoded values. Its interaction with other classes has remained unchanged
* Routes have had the same changed as the ones in ScenarioRunner
* Added parked vehicles to the routes. These are parsed from a file with all their possible positions.
* Added new arguments to the leaderboard:
    - `routes-subset` allows to run part of the routes. Use `-` to run groups of routes (i.e `0-4`), `,` to run specific routes (i.e `1,6,8,14`), or a combination of the two (i.e `0-2,5,8-10`).
    - `debug-checkpoint` defines the endpoint of the live results, created when the `debug` argument is 2 or higher.
* Added support for traffic manager hybrid mode.
* Added a new attribute to the global statistics, *scores_std_dev*, which calculates the standard deviation of the scores done throughout the simulation.
* Fixed bug causing the global infractions to not be correctly calculated

* Creating stable version for the CARLA online leaderboard
* Initial creation of the repository
