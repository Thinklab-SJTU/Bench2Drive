from ubuntu:18.04

# Install base libs
run apt-get update && apt-get install --no-install-recommends -y libpng16-16=1.6.34-1ubuntu0.18.04.2 \
libtiff5=4.0.9-5ubuntu0.3 libjpeg8=8c-2ubuntu8 build-essential=12.4ubuntu1 wget=1.19.4-1ubuntu2.2 git=1:2.17.1-1ubuntu0.7 \
 python3.6 python3.6-dev python3-pip libxerces-c-dev \
 && rm -rf /var/lib/apt/lists/* 

# Install python requirements
run pip3 install --user setuptools==46.3.0 wheel==0.34.2 && pip3 install py_trees==0.8.3 networkx==2.2 pygame==1.9.6 \
    six==1.14.0 numpy==1.18.4 psutil==5.7.0 shapely==1.7.0 xmlschema==1.1.3 ephem==3.7.6.0 tabulate==0.8.7\
&& mkdir -p /app/scenario_runner 

# Install scenario_runner 
copy . /app/scenario_runner

# setup environment :
# 
#   CARLA_HOST :    uri for carla package without trailing slash. 
#                   For example, "https://carla-releases.s3.eu-west-3.amazonaws.com/Linux".
#                   If this environment is not passed to docker build, the value
#                   is taken from CARLA_VER file inside the repository.
#
#   CARLA_RELEASE : Name of the package to be used. For example, "CARLA_0.9.9".
#                   If this environment is not passed to docker build, the value
#                   is taken from CARLA_VER file inside the repository.
# 
#
#  It's expected that $(CARLA_HOST)/$(CARLA_RELEASE).tar.gz is a downloadable resource.
#

env CARLA_HOST ""
env CARLA_RELEASE ""

# Extract and install python API and resources from CARLA
run export DEFAULT_CARLA_HOST="$(sed -e 's/^\s*HOST\s*=\s*//;t;d' /app/scenario_runner/CARLA_VER)" && \
    echo "$DEFAULT_CARLA_HOST" && \
    export CARLA_HOST="${CARLA_HOST:-$DEFAULT_CARLA_HOST}" && \
    export DEFAULT_CARLA_RELEASE="$(sed -e 's/^\s*RELEASE\s*=\s*//;t;d' /app/scenario_runner/CARLA_VER)" && \
    export CARLA_RELEASE="${CARLA_RELEASE:-$DEFAULT_CARLA_RELEASE}" && \
    echo "$CARLA_HOST/$CARLA_RELEASE.tar.gz" && \
    wget -qO- "$CARLA_HOST/$CARLA_RELEASE.tar.gz" | tar -xzv PythonAPI/carla -C / && \
    mv /PythonAPI/carla /app/ && \
    python3 -m easy_install --no-find-links --no-deps "$(find /app/carla/ -iname '*py3.*.egg' )"


# Setup working environment
workdir /app/scenario_runner
env PYTHONPATH "${PYTHONPATH}:/app/carla/agents:/app/carla"
entrypoint ["/bin/sh" ]

