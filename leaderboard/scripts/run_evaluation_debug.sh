#!/bin/bash
BASE_PORT=30000
BASE_TM_PORT=50000
IS_BENCH2DRIVE=True
BASE_ROUTES=leaderboard/data/bench2drive220
TEAM_AGENT=leaderboard/team_code/your_team_agent.py
TEAM_CONFIG=your_team_agent_ckpt.pth   # for TCP and ADMLP
# TEAM_CONFIG=your_team_agent_config.py+your_team_agent_ckpt.pth # for UniAD and VAD
BASE_CHECKPOINT_ENDPOINT=eval
SAVE_PATH=./eval_v1/
PLANNER_TYPE=only_traj

GPU_RANK=3
PORT=$BASE_PORT
TM_PORT=$BASE_TM_PORT
ROUTES="${BASE_ROUTES}.xml"
CHECKPOINT_ENDPOINT="${BASE_CHECKPOINT_ENDPOINT}.json"
bash leaderboard/scripts/run_evaluation.sh $PORT $TM_PORT $IS_BENCH2DRIVE $ROUTES $TEAM_AGENT $TEAM_CONFIG $CHECKPOINT_ENDPOINT $SAVE_PATH $PLANNER_TYPE $GPU_RANK
