#!/bin/bash

autopep8_params="--in-place --max-line-length=120"
pylint_params="--rcfile=.pylintrc"

files=`find leaderboard/ -type f -name "*.py"`
for file in ${files[@]}
do
  autopep8 $file ${autopep8_params}
  pylint --rcfile=.pylintrc ${pylint_params} $file
done
