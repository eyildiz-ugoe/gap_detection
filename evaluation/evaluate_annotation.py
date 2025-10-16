import numpy as np
import collections
import sys


if(len(sys.argv) == 1):
    print("Please enter the path to the annotation as an argument!")
else:
    path = sys.argv[1]
    annotation = open(path)
    pcd_data = annotation.read()
    point_data = pcd_data.split("ascii", 1)[1]
    rows = point_data.split("\n")
    rows.pop(0)

    row_data = []
    for row in rows:
        r = row.split(" ")

        if(len(r) == 5):
            row_data.append(map(float, r))

    labels = []
    for row in row_data:
        if(row[3] != 0.0):
            labels.append(row[3])

    results = collections.Counter(labels)

    # write to file
    file = open("annotation_evaluation.txt", "w")
    for gap, number_of_points in results.items():
        file.write("Label " + str(int(gap)) + " - " +
                   str(number_of_points) + " points\n")

    file.close()
    print("Saved evaluation report of annotation to annotation_evaluation.txt")
