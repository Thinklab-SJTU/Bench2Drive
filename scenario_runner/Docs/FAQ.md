# Frequently Asked Questions

## I receive the error "TypeError: 'instancemethod' object has no attribute '__getitem__'" in the agent navigation

This issue is most likely caused by an outdated version of the Python Networkx package. Please remove the current installation
(e.g. sudo apt-get remove python-networkx) and install it using "pip install --user networkx==2.2".

## No scenario visible and I receive the message "No more scenarios .... Exiting"

In case you receive the following output
```
Preparing scenario: FollowLeadingVehicle_1
ScenarioManager: Running scenario FollowVehicle
Resetting ego-vehicle!
Failure!
Resetting ego-vehicle!
ERROR: failed to destroy actor 527 : unable to destroy actor: not found
No more scenarios .... Exiting
```
and you see nothing happening, it is most likely due to the fact, that you did not launch a program to control
the ego vehicle. Run for example manual_control.py, and you should now see something happening.


## Scenario Runner exits with error when using --debug commandline parameter

In case you receive the following output
```
UnicodeEncodeError: 'ascii' codec can't encode character '\u2713' in position 58: ordinal not in range(128)
```
Please set environment variable
```
PYTHONIOENCODING=utf-8
```

