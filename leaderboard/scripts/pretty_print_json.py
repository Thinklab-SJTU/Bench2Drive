# !/usr/bin/env python
# Copyright (c) 2020 Intel Corporation.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Create a human readable version of the scores provided by the leaderboard.
"""

from __future__ import print_function

import argparse
from argparse import RawTextHelpFormatter
from dictor import dictor
import json
from tabulate import tabulate


def prettify_json(args):
    with open(args.file) as fd:
        json_dict = json.load(fd)

    if not json_dict:
        print('[Error] The file [{}] could not be parsed.'.format(args.file))
        return -1

    progress = dictor(json_dict, '_checkpoint.progress')
    records_table = dictor(json_dict, '_checkpoint.records')
    sensors = dictor(json_dict, 'sensors')
    labels_scores = dictor(json_dict, 'labels')
    scores = dictor(json_dict, 'values')

    # compose output
    output = ""

    if progress:
        output += '* {}% ({}/{}) routes completed\n'.format(100.0*progress[0]/float(progress[1]),
                                                            progress[0],
                                                            progress[1])
    if sensors:
        output += '* The agent used the following sensors: {}\n\n'.format(', '.join(sensors))

    if scores and labels_scores:
        metrics = list(zip(*[labels_scores[0:3], scores[0:3]]))
        infractions = list(zip(*[labels_scores[3:], scores[3:]]))

        output += '=== Global average metrics: ===\n'
        output += tabulate(metrics, tablefmt=args.format)
        output += '\n\n'
        output += '=== Total infractions: ===\n'
        output += tabulate(infractions, tablefmt=args.format)
        output += '\n\n'

    if records_table:
        header = ['Metric', 'Value', 'Additional information']
        list_statistics = [header]
        total_duration_game = 0
        total_duration_system = 0
        total_route_length = 0
        for route in records_table:
            route_completed_kms = 0.01 * route['scores']['Route completion'] * route['meta']['Route length'] / 1000.0
            metrics_route = [[key, '{:.3f}'.format(values), ''] for key, values in route['scores'].items()]
            infractions_route = [[key, '{:.3f} ({} occurrences)'.format(len(values)/route_completed_kms, len(values)),
                                 '\n'.join(values)] for key, values in route['infractions'].items()]

            times = [['Game duration', '{:.3f}'.format(route['meta']['Game duration']), 'seconds'],
                     ['System duration', '{:.3f}'.format(route['meta']['System duration']), 'seconds']]

            route_completed_length = [ ['distance driven', '{:.3f}'.format(route_completed_kms), 'Km']]

            total_duration_game += route['meta']['Game duration']
            total_duration_system += route['meta']['System duration']
            total_route_length += route_completed_kms

            list_statistics.extend([['{}'.format(route['route_id']), '', '']])
            list_statistics.extend([*metrics_route, *infractions_route, *times, *route_completed_length])
            list_statistics.extend([['', '', '']])

        list_statistics.extend([['Total game duration', '{:.3f}'.format(total_duration_game), 'seconds']])
        list_statistics.extend([['Total system duration', '{:.3f}'.format(total_duration_system), 'seconds']])
        list_statistics.extend([['Total distance driven', '{:.3f}'.format(total_route_length), 'Km']])

        output += '==== Per-route analysis: ===\n'.format()
        output += tabulate(list_statistics, tablefmt=args.format)

    if args.output:
        with open(args.output, 'w') as fd:
            fd.write(output)
    else:
        print(output)

    return 0


def main():
    description = 'Create a human readable version of the scores provided by the leaderboard.\n'
    # general parameters
    parser = argparse.ArgumentParser(description=description, formatter_class=RawTextHelpFormatter)
    parser.add_argument('-f', '--file', help='JSON file containing the results of the leaderboard', required=True)
    parser.add_argument('--format', default='fancy_grid',
                        help='Format in which the table will be printed, e.g.: fancy_grid, latex, github, html, jira')
    parser.add_argument('-o', '--output', help='Output file to print the results into')
    arguments = parser.parse_args()

    return prettify_json(arguments)


if __name__ == '__main__':
    main()
