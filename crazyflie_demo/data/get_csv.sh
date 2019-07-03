grep "diff" square_setpoint > diff.txt
cut -c 45- diff.txt > dxdydz.txt
cat dxdydz.txt | tr  " dx:" " " | tr "yz" "," | tr -d " " > diff_nums.csv
python ../sum_diffs.py