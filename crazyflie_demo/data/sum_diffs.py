import csv
from csv import reader
import math

sum_all = 0
count_all = 0
sum_xy = 0
count_xy = 0
worst_error = 0
with open("diff_nums.csv", 'rb') as csvfile:
    f = csv.reader(csvfile, delimiter=',')
    for row in f:
        sum_xy += float(row[0]) + float(row[1])
        sum_all += float(row[0]) + float(row[1]) + float(row[2])
        count_all += 3
        count_xy += 2
        worst_error = max(worst_error, abs(float(row[0])), abs(float(row[1])), abs(float(row[2])))

print("average over all:")
print(1.0*sum_all/count_all)
print("average over xy:")
print(1.0*sum_xy/count_xy)
print("worst error:")
print(worst_error)
