# Create a new scenario tutorial

This tutorial describes how you can create and run a new scenario using the
ScenarioRunner and the ScenarioManager suite.

Let us call the new scenario _NewScenario_. To create it, there are only few
steps required.

## Creating an empty Python class
Go to the Scenarios folder and create a new Python class with the name
_NewScenario_ in a new Python file (_new_scenario.py_). The class should be
derived from the _BasicScenario_ class. As a result, the class should look as
follows:

   ```
   class NewScenario(BasicScenario):
       """
       Some documentation on NewScenario
       :param world is the CARLA world
       :param ego_vehicles is a list of ego vehicles for this scenario
       :param config is the scenario configuration (ScenarioConfiguration)
       :param randomize can be used to select parameters randomly (optional, default=False)
       :param debug_mode can be used to provide more comprehensive console output (optional, default=False)
       :param criteria_enable can be used to disable/enable scenario evaluation based on test criteria (optional, default=True)
       :param timeout is the overall scenario timeout (optional, default=60 seconds)
       """

       # some ego vehicle parameters
       # some parameters for the other vehicles

       def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                    timeout=60):
           """
           Initialize all parameters required for NewScenario
           """

           # Call constructor of BasicScenario
           super(NewScenario, self).__init__(
             "NewScenario",
             ego_vehicles,
             config,
             world,
             debug_mode,
             criteria_enable=criteria_enable)


       def _create_behavior(self):
           """
           Setup the behavior for NewScenario
           """

       def _create_test_criteria(self):
           """
           Setup the evaluation criteria for NewScenario
           """
   ```

## Filling the Python class

In the NewScenario class, you have to define the following methods mentioned
in the code example.

### Initialize Method
The initialize method is intended to setup all parameters required
for the scenario and all vehicles. This includes selecting the correct vehicles,
spawning them at the correct location, etc. To simplify this, you may want to
use the _setup_vehicle()_ function defined in basic_scenario.py

### CreateBehavior method
This method should setup the behavior tree that contains the behavior of all
non-ego vehicles during the scenario. The behavior tree should use py_trees and
the atomic behaviors defined in _atomic_scenario_behavior.py_

### CreateTestCriteria method
This method should setup a list with all evaluation criteria for the scenario.
The criteria should be based on the atomic criteria defined in
_atomic_scenario_criteria.py_.

Note: From this list a parallel py_tree will be created automatically!

## Adding the scenario configuration
Finally the scenario configuration should be added to the examples/ folder. If you
extend an already existing scenario module, you can simply extend the corresponding
XML, otherwise add a new XML file. In this case you can use any of the existing
XML files as blueprint.

If you want to add multiple ego vehicles for a scenario, make sure that they use different
role names, e.g.
```
    <scenario name="MultiEgoTown03" type="FreeRide" town="Town03">
        <ego_vehicle x="207" y="59" z="0" yaw="180" model="vehicle.lincoln.mkz_2017" rolename="hero"/>
        <ego_vehicle x="237" y="-95.0754252474" z="0" yaw="90" model="vehicle.tesla.model3" rolename="hero2"/>
    </scenario>
```
