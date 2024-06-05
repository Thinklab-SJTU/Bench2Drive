#!/bin/bash
BASE_PORT=30000
BASE_TM_PORT=50000
IS_BENCH2DRIVE=True
BASE_ROUTES=leaderboard/data/bench2drive220
TEAM_AGENT=team_code/your_team_agent.py
TEAM_CONFIG=your_team_agent_ckpt.pth
BASE_CHECKPOINT_ENDPOINT=eval_bench2drive220
SAVE_PATH=./eval_bench2drive220/

TASK_NUM=4
GPU_RANK_LIST=(0 1)
TASK_LIST=(0 1 2 3)
python tools/split_xml.py $BASE_ROUTES $TASK_NUM
length=${#GPU_RANK_LIST[@]}
for ((i=0; i<$length; i++ )); do
    PORT=$((BASE_PORT + i * 150))
    TM_PORT=$((BASE_TM_PORT + i * 150))
    ROUTES="${BASE_ROUTES}_${TASK_LIST[$i]}.xml"
    CHECKPOINT_ENDPOINT="uniad_0526/${BASE_CHECKPOINT_ENDPOINT}_${TASK_LIST[$i]}.json"
    GPU_RANK=${GPU_RANK_LIST[$i]}
    echo "INDEX: $i"
    echo "PORT: $PORT"
    echo "TM_PORT: $TM_PORT"
    echo "CHECKPOINT_ENDPOINT: $CHECKPOINT_ENDPOINT"
    echo "GPU_RANK: $GPU_RANK"
    echo "bash leaderboard/scripts/run_evaluation.sh $PORT $TM_PORT $IS_BENCH2DRIVE $ROUTES $TEAM_AGENT $TEAM_CONFIG $CHECKPOINT_ENDPOINT $SAVE_PATH $PLANNER_TYPE $GPU_RANK"
    bash leaderboard/scripts/run_evaluation.sh $PORT $TM_PORT $IS_BENCH2DRIVE $ROUTES $TEAM_AGENT $TEAM_CONFIG $CHECKPOINT_ENDPOINT $SAVE_PATH $PLANNER_TYPE $GPU_RANK 2>&1 > ${BASE_ROUTES}_${TASK_LIST[$i]}_${BASE_CHECKPOINT_ENDPOINT}.log &
    sleep 5
done
wait