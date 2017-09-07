from __future__ import division
from __future__ import print_function

import os
import math


def radians(yaw):
    """Convert from degrees to radians in the range [-PI, PI]"""
    yaw_radians = math.radians(yaw)
    if yaw > 180.:
        yaw_radians -= 2 * math.pi
    elif yaw < -180.:
        yaw_radians += 2 * math.pi
    return yaw_radians

def degrees(yaw):
    """Convert from radians to degrees in the range [0, 360)"""
    return math.degrees(yaw) % 360.

def convert_file(filepath, yaw_column, conversion='radians', delimiter=','):
    if conversion == 'radians':
        convert_f = radians
        prec = 12  # 9 is already enough
    elif conversion == 'degrees':
        convert_f = degrees
        prec = 6
    else:
        raise ValueError("Invalid conversion: `{}`".format(conversion))

    name, ext = os.path.splitext(filepath)
    output_path = "%s_%s%s" % (name, conversion, ext)

    with open(filepath, 'r') as fp_input:
        with open(output_path, 'w') as fp_output:
            for line in fp_input:
                row = line.rstrip().split(delimiter)
                yaw_org = float(row[yaw_column])
                yaw = convert_f(yaw_org)
                row[yaw_column] = round(yaw, prec)
                print(*row, sep=',', file=fp_output)

    return output_path


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description='Convert yaw column from degrees to radians')
    parser.add_argument('file', help='filename to convert');
    parser.add_argument('-column', type=int, default=4,
                        help='column containing the yaw (default=4)')
    parser.add_argument('-degrees', action='store_true', default=False,
                        help='convert from radians to degrees')

    args = parser.parse_args()

    filepath = args.file
    column = max(0, args.column - 1)
    conversion = 'degrees' if args.degrees else 'radians'

    output_path = convert_file(filepath, column, conversion)
    print(output_path)
