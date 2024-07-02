#!/bin/bash

# Check if huggingface-hub is installed
if ! python -m pip show huggingface-hub > /dev/null 2>&1; then
  echo "huggingface-hub is not installed. Installing now..."
  python -m pip install huggingface-hub
else
  echo "huggingface-hub is already installed."
fi

mkdir Bench2Drive-mini

huggingface-cli download --resume-download --repo-type dataset rethinklab/Bench2Drive --include "HardBreakRoute_Town01_Route30_Weather3.tar.gz" --local-dir Bench2Drive-mini --local-dir-use-symlinks False
huggingface-cli download --resume-download --repo-type dataset rethinklab/Bench2Drive --include "DynamicObjectCrossing_Town02_Route13_Weather6.tar.gz" --local-dir Bench2Drive-mini --local-dir-use-symlinks False
huggingface-cli download --resume-download --repo-type dataset rethinklab/Bench2Drive --include "Accident_Town03_Route156_Weather0.tar.gz" --local-dir Bench2Drive-mini --local-dir-use-symlinks False
huggingface-cli download --resume-download --repo-type dataset rethinklab/Bench2Drive --include "YieldToEmergencyVehicle_Town04_Route165_Weather7.tar.gz" --local-dir Bench2Drive-mini --local-dir-use-symlinks False
huggingface-cli download --resume-download --repo-type dataset rethinklab/Bench2Drive --include "ConstructionObstacle_Town05_Route68_Weather8.tar.gz" --local-dir Bench2Drive-mini --local-dir-use-symlinks False
huggingface-cli download --resume-download --repo-type dataset rethinklab/Bench2Drive --include "ParkedObstacle_Town10HD_Route371_Weather7.tar.gz" --local-dir Bench2Drive-mini --local-dir-use-symlinks False
huggingface-cli download --resume-download --repo-type dataset rethinklab/Bench2Drive --include "ControlLoss_Town11_Route401_Weather11.tar.gz" --local-dir Bench2Drive-mini --local-dir-use-symlinks False
huggingface-cli download --resume-download --repo-type dataset rethinklab/Bench2Drive --include "AccidentTwoWays_Town12_Route1444_Weather0.tar.gz" --local-dir Bench2Drive-mini --local-dir-use-symlinks False
huggingface-cli download --resume-download --repo-type dataset rethinklab/Bench2Drive --include "OppositeVehicleTakingPriority_Town13_Route600_Weather2.tar.gz" --local-dir Bench2Drive-mini --local-dir-use-symlinks False
huggingface-cli download --resume-download --repo-type dataset rethinklab/Bench2Drive --include "VehicleTurningRoute_Town15_Route443_Weather1.tar.gz" --local-dir Bench2Drive-mini --local-dir-use-symlinks False
