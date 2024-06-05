import argparse

from leaderboard.utils.checkpoint_tools import fetch_dict
from leaderboard.utils.statistics_manager import StatisticsManager


def check_duplicates(route_ids):
    """Checks that all route ids are present only once in the files"""
    for id in route_ids:
        if route_ids.count(id) > 1:
            raise ValueError(f"Stopping. Found that the route {id} has more than one record")


def check_missing_data(route_ids):
    """Checks that there is no missing data, by changing their route id to an integer"""
    rep_num = 1
    prev_rep_int = 0
    prev_total_int = 0
    prev_id = ""

    for id in route_ids:
        route_int = int(id.split('_')[1])
        rep_int = int(id.split('_rep')[-1])

        # Get the amount of repetitions. Done when a reset of the repetition number is found
        if rep_int < prev_rep_int:
            rep_num = prev_rep_int + 1

        # Missing data will create a jump of 2 units
        # (i.e if 'Route0_rep1' is missing, 'Route0_rep0' will be followed by 'Route0_rep2', which are two units)
        total_int = route_int * rep_num + rep_int
        if total_int - prev_total_int > 1: 
            raise ValueError(f"Stopping. Missing some data as the ids jumped from {prev_id} to {id}")

        prev_rep_int = rep_int
        prev_total_int = total_int
        prev_id = id


def main():
    """
    Utility script to merge two or more statistics into one.
    While some checks are done, it is best to ensure that merging all files makes sense
    """
    argparser = argparse.ArgumentParser()
    argparser.add_argument('-f', '--file-paths', nargs="+", required=True, help='path to all the files containing the partial results')
    argparser.add_argument('-e', '--endpoint', required=True, help='path to the endpoint containing the joined results')
    args = argparser.parse_args()

    # Initialize the statistics manager
    statistics_manager = StatisticsManager(args.endpoint, 0)

    # Make sure that the data is correctly formed
    sensors = []
    route_ids = []
    total_routes = 0
    total_progress = 0

    for file in args.file_paths:
        data = fetch_dict(file)
        if not data:
            continue

        route_ids.extend([x['route_id'] for x in data['_checkpoint']['records']])
        total_routes += len(data['_checkpoint']['records'])
        total_progress += data['_checkpoint']['progress'][1]

        if data['sensors']:
            if not sensors:
                sensors = data['sensors']
            elif data['sensors'] != sensors:
                raise ValueError("Stopping. Found two files with different sensor configurations")

    route_ids.sort(key=lambda x: (
        int(x.split('_')[1]),
        int(x.split('_rep')[-1])
    ))

    global_statistics = total_progress != 0 and total_routes == total_progress

    if global_statistics:
        check_duplicates(route_ids)
        check_missing_data(route_ids)

    # All good, join the data and get the global results
    for file in args.file_paths:
        statistics_manager.add_file_records(file)

    statistics_manager.sort_records()
    statistics_manager.save_sensors(sensors)
    statistics_manager.save_progress(total_routes, total_progress)
    statistics_manager.save_entry_status('Started')
    if global_statistics:
        statistics_manager.compute_global_statistics()
        statistics_manager.validate_and_write_statistics(True, False)
    else:
        statistics_manager.write_statistics()

if __name__ == '__main__':
    main()
