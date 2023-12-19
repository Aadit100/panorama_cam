#!/bin/bash
num=9
i=0
while [[$i -le $num]]; 
do
	filename="imu_data_"+i+".bag"
	scp nvidia@10.42.0.51:/home/nvidia/caesar2020/src/navigation/scripts/$filename  ~/bagfiles
	(( i += 1 ))
done
source /home/aadit/caesar/devel/setup.bash
cd ~/bagfiles
j=0
while [[$j -le $num]];
do
	filename="imu_data_"+j+".bag"
	out_filename="output_"+j+".txt"
	rostopic echo /imu_angles/Pitch > $out_filename & rosbag play -r 15 $filename
	(( j += 1 ))
done
python3 ./final_dist_plt.py & python3 ./init_dist_plt.py
