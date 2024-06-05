# Get ScenarioRunner

This tutorial explains how to download ScenarioRunner and run a simple example to test it. ScenarioRunner needs CARLA in order to run, and must match the CARLA version being used. If the CARLA being used is a build from source, download ScenarioRunner from source. If the CARLA being used is a package, download the corresponding version of ScenarioRunner.  

*   __[Installation summary](#installation-summary)__  
*   __[A. Download a ScenarioRunner release](#a.-download-a-scenariorunner-release)__  
	*   [Update the release](#update-the-release)  
*   __[B. Download ScenarioRunner from source](#b.-build-scenariorunner-from-source)__  
	*   [Update the build from source](#update-the-build-from-source)  
*   __[Run a test](#run-a-test)__  

---
## Installation summary

<details>
   <summary>
    Show command line summary for the quick start installation
   </summary>

```sh
# Decide whether to use a package or make the build from source


# Option A) Use a ScenarioRunner package
   # 1. Install a CARLA package: 
      https://carla.readthedocs.io/en/latest/start_quickstart/
   # 2. Download the matching ScenarioRunner package: 
      https://github.com/carla-simulator/scenario_runner/releases
   # 3. Extract the content wherever needed. 

   # Update the release: 
   # 1. Delete previous CARLA and ScenarioRunner versions.
   # 2. Download the latest CARLA release. 
   # 3. Download the matching ScenarioRunner release.


# Option B) Download ScenarioRunner from source
   # 1. Build CARLA from source:
      https://carla.readthedocs.io/en/latest/build_linux/
   # 2. Clone the ScenarioRunner repository: 
git clone https://github.com/carla-simulator/scenario_runner.git
   # 3. Install requirements according to the Python version to be used: 
   # For Python 2.x:
sudo apt remove python-networkx #if installed, remove old version of networkx
pip2 install --user -r requirements.txt
   # For Python 3.x: 
sudo apt remove python3-networkx #if installed, remove old version of networkx
pip3 install --user -r requirements.txt

   # To update ScenarioRunner from source:
   # 1. Update CARLA: 
      https://carla.readthedocs.io/en/latest/build_update/
   # 2. Go to the ScenarioRunner repository, master branch
cd ~/scenario_runner
git branch master
   # 3. Pull the latest changes from the repository
git pull 

```
</details>


---
## A. Download a ScenarioRunner release

The releases of ScenarioRunner are packages containing:  
*   A ScenarioRunner version tied to a specific CARLA release.  
*   A few example scenarios written in Python.  

The process to run a ScenarioRunner release is quite straightforward.  

__1. Download a CARLA release.__ Follow the process in the [CARLA quick start](https://github.com/carla-simulator/carla/releases).  
__2. Download the matching ScenarioRunner release.__ All of the releases are listed [here](https://github.com/carla-simulator/scenario_runner/releases).

!!! Important
    Both versions have to match. If the CARLA release is *0.9.9*, use also ScenarioRunner *0.9.9*. [Here](https://github.com/carla-simulator/scenario_runner) is a brief list of compatibilities between CARLA and ScenarioRunner.  

__3. Extract the content.__ The directory does not matter.  


### Update the release

The packaged version requires no updates. The content is bundled and thus, tied to a specific release of CARLA. Everytime there is a new CARLA release, there will be a matching one for ScenarioRunner. All the releases are listed here:  

*   [CARLA releases](https://github.com/carla-simulator/carla/releases)  
*   [Scenario Runner releases](https://github.com/carla-simulator/scenario_runner/releases)  

To run the latest or any other release, delete the previous and install the one desired.  

---
## B. Download ScenarioRunner from source

The ScenarioRunner source repository contains the most experimental features that run with the latest development version of CARLA. It requires no build, as it only contains Python code for the ScenarioRunner module.  

__1. Build CARLA from source.__ Follow the docs to build on [Linux](https://carla.readthedocs.io/en/latest/build_linux/) or [Windows](https://carla.readthedocs.io/en/latest/build_windows/).  

!!! Important
    ScenarioRunner needs CARLA to run, so the minimum requirements for CARLA stated in the docs are also necessary to run ScenarioRunner.  

__2. Clone the ScenarioRunner repository.__

```sh
git clone https://github.com/carla-simulator/scenario_runner.git
```

__3. Install the requirements according to the Python version to be used.__  First go to the main ScenarioRunner directory

```sh
cd ~/scenario_runner/
```

*   __For Python 2.x.__  

```sh
sudo apt remove python-networkx #if installed, remove old version of networkx
pip2 install --user -r requirements.txt
```

*   __For Python 3.x__  
```sh
sudo apt remove python3-networkx #if installed, remove old version of networkx
pip3 install --user -r requirements.txt
```

!!! Warning
    py-trees newer than v0.8 are __not__ supported.


### Update from source  

__1. Update the CARLA build__ Follow the [docs](https://carla.readthedocs.io/en/latest/build_update/) to update CARLA.  

__2. Go to the main ScenarioRunner directory.__ Make sure to be in the local master branch.  

```sh
cd ~/scenario_runner
git branch master
```
__3. Pull the latest changes from the repository.__  

```sh
git pull
```

__4. Add environment variables and Python paths__ These are necessary for the system to find CARLA, and add the PythonAPI to the Python path.

*   __For Linux__

```sh
# ${CARLA_ROOT} is the CARLA installation directory
# ${SCENARIO_RUNNER} is the ScenarioRunner installation directory
# <VERSION> is the correct string for the Python version being used
# In a build from source, the .egg files may be in: ${CARLA_ROOT}/PythonAPI/dist/ instead of ${CARLA_ROOT}/PythonAPI
export CARLA_ROOT=/path/to/your/carla/installation
export SCENARIO_RUNNER_ROOT=/path/to/your/scenario/runner/installation
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-<VERSION>.egg
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
```

!!! Note
    Change the command lines with the proper paths, according to the comments. 

*   __For Windows__


```sh
# %CARLA_ROOT% is the CARLA installation directory
# %SCENARIO_RUNNER% is the ScenarioRunner installation directory
# <VERSION> is the correct string for the Python version being used
# In a build from source, the .egg files may be in: ${CARLA_ROOT}/PythonAPI/dist/ instead of ${CARLA_ROOT}/PythonAPI
set CARLA_ROOT=\path\to\your\carla\installation
set SCENARIO_RUNNER_ROOT=\path\to\your\scenario\runner\installation
set PYTHONPATH=%PYTHONPATH%;%CARLA_ROOT%\PythonAPI\carla\dist\carla-<VERSION>.egg
set PYTHONPATH=%PYTHONPATH%;%CARLA_ROOT%\PythonAPI\carla
```

!!! Note
    Change the command lines with the proper paths, according to the comments. 

To permanently set the environment variables, go to *edit the environment variables of this account*. This can be quickly accessed by writing *env* on the Windows' search panel.

---
## Run a test 

Running the follow vehicle example
First of all, you need to get latest master branch from CARLA. Then you have to
include CARLA Python API to the Python path:

!!! Important
    If working with builds from source, make sure to upload these. Download the latest content in the master branches.  


__1. Run the CARLA server.__

*   __A) In a build from source__ go to the CARLA directory and launch the server in the editor. 

```sh
cd ~/carla # Change the path accordingly
make launch
# Press Play in the UE Editor
```

*   __B) In a CARLA package__ run the server directly. 

```sh
./CarlaUE4.sh
```

__2. Start an example scenario.__ Open another terminal and go to the directory where ScenarioRunner is downloaded. For the sake of this test, the follow leading vehicle scenario will be used. 

```sh
# Inside the ScenarioRunner root directory
python scenario_runner.py --scenario FollowLeadingVehicle_1 --reloadWorld
```

!!! Note
    If using a Python 3.x version, run the command with `python3`. 

__3. Test the scenario with manual control.__ Open a new terminal and run the `manual_control.py`. A new window should pop up, with an ego vehicle in the middle of the street. Move forward and the leading vehicle will appear. 

```sh
# Inside the ScenarioRunner root directory
python manual_control.py
```

The scenarios have a timeout of one minute approximately, for the agent to be launched. If the timeout appears, the follow leading vehicle example should be launched again.  

!!! Warning
    Run the `manual_control.py` found in the ScenarioRunner package/repository, __not CARLA__. 

__4. Explore other options.__ Run the Scenario Runner with the flag `--help` to explore other command line parameters and some basic descriptions. For example, to avoid automatically (re-)load the CARLA world, skip the command line option `--reloadWorld`.

```sh
python scenario_runner.py --help
```

---

Thus concludes the installation process for ScenarioRunner. In case any unexpected error or issue occurs, the [CARLA forum](https://forum.carla.org/c/using-carla/scenario-runner) is open to everybody. There is an _ScenarioRunner_ category to post problems and doubts regarding this module. 

