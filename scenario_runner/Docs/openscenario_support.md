## OpenSCENARIO Support

The scenario_runner provides support for the [OpenSCENARIO](http://www.openscenario.org/) 1.0 standard.
The current implementation covers initial support for maneuver Actions, Conditions, Stories and the Storyboard.
If you would like to use evaluation criteria for a scenario to evaluate pass/fail results, these can be implemented
as StopTriggers (see below). However, not all features for these elements are yet available. If in doubt, please see the
module documentation in srunner/tools/openscenario_parser.py

An example for a supported scenario based on OpenSCENARIO is available [here](https://github.com/carla-simulator/scenario_runner/blob/master/srunner/examples/FollowLeadingVehicle.xosc)

In addition, it is recommended to take a look into the official documentation available [here](https://releases.asam.net/OpenSCENARIO/1.0.0/Model-Documentation/index.html) and [here](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html#_foreword).

### Migrating OpenSCENARIO 0.9.x to 1.0
The easiest way to convert old OpenSCENARIO samples to the official standard 1.0 is to use _xsltproc_ and the migration scheme located in the openscenario folder.
Example:

```bash
xsltproc -o newScenario.xosc migration0_9_1to1_0.xslt oldScenario.xosc
```


### Level of support
In the following the OpenSCENARIO attributes are listed with their current support status.

#### General OpenSCENARIO setup

This covers all part that are defined outside the OpenSCENARIO Storyboard

| Attribute                          | Support                            | Notes                              |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| `FileHeader`                       | &#9989;                              | Use "CARLA:" at the beginning of the description to use the CARLA coordinate system.                                |
| `CatalogLocations`<br>`ControllerCatalog`                                      | &#9989;                              | While the catalog is supported, the reference/usage may not work.                |
| `CatalogLocations`<br>`EnvironmentCatalog`                                     | &#9989;                              |                                    |
| `CatalogLocations`<br>`ManeuverCatalog`                                        | &#9989;                              |                                    |
| `CatalogLocations`<br>`MiscObjectCatalog`                                      | &#9989;                              |                                    |
| `CatalogLocations`<br>`PedestrianCatalog`                                      | &#9989;                              |                                    |
| `CatalogLocations`<br>`RouteCatalog`                                           | &#9989;                              | While the catalog is supported, the reference/usage may not work.                |
| `CatalogLocations`<br>`TrajectoryCatalog`                                      | &#9989;                              | While the catalog is supported, the reference/usage may not work.                |
| `CatalogLocations`<br>`VehicleCatalog`                                         | &#9989;                              |                                    |
| `ParameterDeclarations`            | &#9989;                              |                                    |
| `RoadNetwork`<br>`LogicFile`                                                   | &#9989;                              | The CARLA level can be used directly (e.g. LogicFile=Town01). Also any OpenDRIVE path can be provided.              |
| `RoadNetwork`<br>`SceneGraphFile`                                              | &#10060;                               | The provided information is not used.                                            |
| `RoadNetwork`<br>`TafficSignals`                                               | &#10060;                               | The provided information is not used.                                            |
| `Entities`<br>`EntitySelection`                                                | &#10060;                               | The provided information is not used.                                            |
| `Entities` `ScenarioObject`<br>`CatalogReference`                               | &#9989;                              | The provided information is not used.                                            |
| `Entities` `ScenarioObject`<br>`MiscObject`                                     | &#9989;                              | The name should match a CARLA vehicle model, otherwise a default vehicle based on the vehicleCategory is used. BoundingBox entries are ignored.                   |
| `Entities` `ScenarioObject`<br>`ObjectController`                               | &#10060;                               | The provided information is not used.                                            |
| `Entities` `ScenarioObject`<br>`Pedestrian`                                     | &#9989;                              | The name should match a CARLA vehicle model, otherwise a default vehicle based on the vehicleCategory is used. BoundingBox entries are ignored.                   |
| `Entities` `ScenarioObject`<br>`Vehicle`                                        | &#9989;                              | The name should match a CARLA vehicle model, otherwise a default vehicle based on the vehicleCategory is used. The color can be set via properties ('Property name="color" value="0,0,255"'). Axles, Performance, BoundingBox entries are ignored. |

<br>


#### OpenSCENARIO Storyboard

##### OpenSCENARIO Actions

The OpenSCENARIO Actions can be used for two different purposes. First, Actions can be used to
define the initial behavior of something, e.g. a traffic participant. Therefore, Actions can be
used within the OpenSCENARIO Init. In addition, Actions are also used within the OpenSCENARIO
story. In the following, the support status for both application areas is listed. If an action
contains of submodules, which are not listed, the support status applies to all submodules.

###### GlobalAction



| GlobalAction                          | Init  support                            | Story support                              | Notes                              |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| `EntityAction`                | &#10060;                          | &#10060;                          |                               |
| `EnvironmentAction`           | &#9989;                         | &#10060;                          |                               |
| `ParameterAction`             | &#9989;                          | &#9989;                          |                               |
| `InfrastructureAction` `TrafficSignalAction`<br>`TrafficAction`                                               | &#10060;                          | &#10060;                          |                               |
| `InfrastructureAction` `TrafficSignalAction`<br>`TrafficSignalControllerAction`                               | &#10060;                          | &#10060;                          |                               |
| `InfrastructureAction` `TrafficSignalAction`<br>`TrafficSignalStateAction`                                    | &#10060;                          | &#9989;                         | As traffic signals in CARLA towns have no unique ID, they have to be set by providing their position (Example: TrafficSignalStateAction name="pos=x,y" state="green"). The id can also be used for none CARLA town (Example: TrafficSignalStateAction name="id=n" state="green") |

<br>



###### UserDefinedAction



| UserDefinedAction                          | Init  support                            | Story support                              | Notes                              |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| `CustomCommandAction`                       | &#10060;                                        | &#9989;                                       | This action is currently used to trigger the execution of an additional script. Example: type="python /path/to/script args". |

<br>


###### PrivateAction


| PrivateAction                          | Init  support                            | Story support                              | Notes                              |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| `ActivateControllerAction`                          | &#10060;             | &#9989;            | Can be used to activate/deactive the CARLA autopilot.                                             |
| `ControllerAction`                                  | &#9989;            | &#9989;            | AssignControllerAction is supported, but a Python module has to be provided for the controller implementation, and in OverrideControllerValueAction all values need to be `False`. |
| `LateralAction`<br> `LaneChangeAction`             | &#10060;             | &#9989;            | Currently all lane changes have a linear dynamicShape, the dynamicDimension is defined as the distance and are relative to the actor itself (RelativeTargetLane).                  |
| `LateralAction`<br>`LaneOffsetAction`             | &#10060;             | &#9989;             |  Currently all type of dynamicShapes are ignored and depend on the controller. This action might not work as intended if the offset is high enough to make the vehicle exit its lane  |
| `LateralAction`<br> `LateralDistanceAction`        | &#10060;             | &#10060;             |                  |
| `LongitudinalAction`<br> `LongitudinalDistanceAction`| &#10060;             | &#9989;              |`timeGap` attribute is not supported              |
| `LongitudinalAction`<br> `SpeedAction`             | &#9989;            | &#9989;            |                  |
| `SynchronizeAction`                                 | &#10060;             | &#10060;             |                  |
| `TeleportAction`                                    | &#9989;            | &#9989;            |                  |
| `VisibilityAction`                                  | &#10060;             | &#10060;             |                  |
| `RoutingAction`<br> `AcquirePositionAction`        | &#10060;             | &#9989;            |                  |
| `RoutingAction`<br> `AssignRouteAction`            | &#10060;             | &#9989;            | Route Options (shortest/fastest/etc) are supported. Shortests means direct path between A and B, all other will use the shortest path along the road network between A and B       |
| `RoutingAction`<br> `FollowTrajectoryAction`       | &#10060;             | &#10060;             |                  |

<br>



##### OpenSCENARIO Conditions

Conditions in OpenSCENARIO can be defined either as ByEntityCondition or as ByValueCondition.
Both can be used for StartTrigger and StopTrigger conditions.
The following two tables list the support status for each.

###### ByEntityCondition


| EntityCondition                                | Support                                        | Notes                                          |
| ---------------------------------------------- | ---------------------------------------------- | ---------------------------------------------- |
| `AccelerationCondition`                        | &#9989;                                          |                                                |
| `CollisionCondition`                           | &#9989;                                          |                                                |
| `DistanceCondition`                            | &#9989;                                          | \*freespace\* attribute is still not supported |
| `EndOfRoadCondition`                           | &#9989;                                          |                                                |
| `OffroadCondition`                             | &#9989;                                          |                                                |
| `ReachPositionCondition`                       | &#9989;                                          |                                                |
| `RelativeDistanceCondition`                    | &#9989;                                          |                                                |
| `RelativeSpeedCondition`                       | &#9989;                                          |                                                |
| `SpeedCondition`                               | &#9989;                                          |                                                |
| `StandStillCondition`                          | &#9989;                                          |                                                |
| `TimeHeadwayCondition`                         | &#9989;                                          |                                                |
| `TimeToCollisionCondition`                     | &#9989;                                          |                                                |
| `TraveledDistanceCondition`                    | &#9989;                                          |                                                |

<br>

###### ByValueCondition


| ValueCondition            | Support                   | Notes                     |
| -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| `ParameterCondition`         | &#9989;                     | The level of support depends on the parameter. It is recommended to use other conditions if possible. Please also consider the note below.                                                  |
| `SimulationTimeCondition`                                    | &#9989;                     |                           |
| `StoryboardElementStateCondition`                            | &#9989;                     | startTransition, stopTransition, endTransition and completeState are currently supported.                  |
| `TimeOfDayCondition`         | &#9989;                     |                           |
| `TrafficSignalCondition`                                     | &#9989;                     | As traffic signals in CARLA towns have no unique ID, they have to be set by providing their position (Example: TrafficSignalCondition name="pos=x,y" state="green"). The id can also be used for none CARLA town (Example: TrafficSignalCondition name="id=n" state="green") |
| `TrafficSignalControllerCondition`                           | &#10060;                      |                           |
| `UserDefinedValueCondition`                                  | &#10060;                      |                           |

<br>

!!! Note
     In the OpenSCENARIO 1.0 standard, a definition of test / evaluation criteria is not
     defined. For this purpose, you can re-use StopTrigger conditions with CARLA. The following
     StopTrigger conditions for evaluation criteria are supported through ParameterConditions by
     providing the criteria name for the condition:

     * criteria_RunningStopTest
     * criteria_RunningRedLightTest
     * criteria_WrongLaneTest
     * criteria_OnSideWalkTest
     * criteria_KeepLaneTest
     * criteria_CollisionTest
     * criteria_DrivenDistanceTest


##### OpenSCENARIO Positions

There are several ways of defining positions in OpenSCENARIO. In the following we list the
current support status for each definition format.

| Position                          | Support                            | Notes                              |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| `LanePosition`           | &#9989;                    |                          |
| `RelativeLanePosition`   | &#9989;                    |                          |
| `RelativeObjectPosition` | &#9989;                    |                          |
| `RelativeRoadPosition`   | &#9989;                     |                          |
| `RelativeWorldPosition`  | &#9989;                    |                          |
| `RoadPosition`           | &#9989;                     |                          |
| `RoutePosition`          | &#10060;                     |                          |
| `WorldPosition`          | &#9989;                    |                          |

<br>
