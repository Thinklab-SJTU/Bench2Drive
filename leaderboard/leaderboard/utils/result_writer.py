#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module contains the result gatherer and write for CARLA scenarios.
It shall be used from the ScenarioManager only.
"""

from __future__ import print_function

import time
from collections import OrderedDict
from tabulate import tabulate


COLORED_STATUS = {
    "FAILURE": '\033[91mFAILURE\033[0m',
    "SUCCESS": '\033[92mSUCCESS\033[0m',
    "ACCEPTABLE": '\033[93mACCEPTABLE\033[0m',
}

STATUS_PRIORITY = {
    "FAILURE": 0,
    "ACCEPTABLE": 1,
    "SUCCESS": 2,
}  # Lower number is higher priority


class ResultOutputProvider(object):

    """
    This module contains the _result gatherer and write for CARLA scenarios.
    It shall be used from the ScenarioManager only.
    """

    def __init__(self, data):
        """
        - data contains all scenario-related information
        - global_result is overall pass/fail info
        """
        self._data = data
        self._start_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self._data.start_system_time))
        self._end_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self._data.end_system_time))

        self._global_result = '\033[92m'+'SUCCESS'+'\033[0m'
        for criterion in self._data.scenario.get_criteria():
            if criterion.test_status != "SUCCESS":
                self._global_result = '\033[91m'+'FAILURE'+'\033[0m'
        if self._data.scenario.timeout_node.timeout:
            self._global_result = '\033[91m'+'FAILURE'+'\033[0m'

        print(self.create_output_text())

    def create_output_text(self):
        """
        Creates the output message
        """

        # Create the title
        output = "\n"
        output += "\033[1m========= Results of {} (repetition {}) ------ {} \033[1m=========\033[0m\n".format(
            self._data.scenario_tree.name, self._data.repetition_number, self._global_result)
        output += "\n"

        # Simulation part
        system_time = round(self._data.scenario_duration_system, 2)
        game_time = round(self._data.scenario_duration_game, 2)
        ratio = round(self._data.scenario_duration_game / self._data.scenario_duration_system, 3)

        list_statistics = [["Start Time", "{}".format(self._start_time)]]
        list_statistics.extend([["End Time", "{}".format(self._end_time)]])
        list_statistics.extend([["System Time", "{}s".format(system_time)]])
        list_statistics.extend([["Game Time", "{}s".format(game_time)]])
        list_statistics.extend([["Ratio (Game / System)", "{}".format(ratio)]])

        output += tabulate(list_statistics, tablefmt='fancy_grid')
        output += "\n\n"

        # Criteria part
        header = ['Criterion', 'Result', 'Value']
        list_statistics = [header]
        criteria_data = OrderedDict()

        for criterion in self._data.scenario.get_criteria():

            name = criterion.name

            if name in criteria_data:
                result = criterion.test_status
                if STATUS_PRIORITY[result] < STATUS_PRIORITY[criteria_data[name]['result']]:
                    criteria_data[name]['result'] = result
                criteria_data[name]['actual_value'] += criterion.actual_value

            else:
                criteria_data[name] = {
                    'result': criterion.test_status,
                    'actual_value': criterion.actual_value,
                    'units': criterion.units
                }

        for criterion_name in criteria_data:
            criterion = criteria_data[criterion_name]

            result = criterion['result']
            if result in COLORED_STATUS:
                result = COLORED_STATUS[result]

            if criterion['units'] is None:
                actual_value = ""
            else:
                actual_value = str(criterion['actual_value']) + " " + criterion['units']

            list_statistics.extend([[criterion_name, result, actual_value]])

        # Timeout
        name = "Timeout"

        actual_value = self._data.scenario_duration_game

        if self._data.scenario_duration_game < self._data.scenario.timeout:
            result = '\033[92m'+'SUCCESS'+'\033[0m'
        else:
            result = '\033[91m'+'FAILURE'+'\033[0m'

        list_statistics.extend([[name, result, '']])

        output += tabulate(list_statistics, tablefmt='fancy_grid')
        output += "\n"

        return output
