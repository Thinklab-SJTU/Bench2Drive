import xml.etree.ElementTree as ET


def split_list_into_n_parts(lst, n):
    k, m = divmod(len(lst), n)
    return (lst[i * k + min(i, m):(i + 1) * k + min(i + 1, m)] for i in range(n))

def main(base_route, task_num, algo, planner_type):
    tree = ET.parse(f'{base_route}.xml')
    root = tree.getroot()
    case = root.findall('route')
    results = split_list_into_n_parts(case, task_num)
    for index, re in enumerate(results):
        new_root = ET.Element("routes")
        for x in re:
            new_root.append(x)
        new_tree = ET.ElementTree(new_root)
        new_tree.write(f'{base_route}_{index}_{algo}_{planner_type}.xml', encoding='utf-8', xml_declaration=True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("base_route", type=str)
    parser.add_argument("task_num", type=int)
    parser.add_argument("algo", type=str)
    parser.add_argument("planner_type", type=str)
    args = parser.parse_args()
    main(args.base_route, args.task_num, args.algo, args.planner_type)