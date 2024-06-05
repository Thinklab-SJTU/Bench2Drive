#!/bin/bash
killall -9 -r CarlaUE4-Linux
ps -ef | grep "carla-rpc-port" | awk '{print $2}' | xargs kill > /dev/null 2>&1 &
ps -ef | grep "run_evaluation" | awk '{print $2}' | xargs kill > /dev/null 2>&1 &
ps -ef | grep "leaderboard_evaluator" | awk '{print $2}' | xargs kill > /dev/null 2>&1 &
wait
