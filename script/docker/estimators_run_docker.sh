#!/bin/bash

# Copyright (C) 2021 Alessandro Fornasier, Control of Networked Systems,
# University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available in the
# LICENSE file. No license in patents is granted.
#
# You can contact the author at alessandro.fornasier@ieee.org


# *******************************************************************
# Remember to change the estimators parameters like camera intrinsic,
# camera extrinsics, imu and camera rates, imu noise values, etc...
# accordingly to the generated data
# *******************************************************************

clear

echo "------------------------------------------------------------------------"
echo "  _____            __________                                           "
echo "  \    \          /          |   - VINSEVAL:                        -   "
echo "   \    \        /  _________|   - EVALUATION FRAMEWORK FOR UNIFIED -   "
echo "    \    \      /  |             - TESTING OF CONSISTENCY AND       -   "
echo "     \    \    /   |_________    - ROBUSTNESS OF VISUAL-INERTIAL    -   "
echo "      \    \  /              |   - NAVIGATION SYSTEMS ALGORITHMS    -   "
echo "       \    \/     __________|                                          "
echo "        \    \    |              -- ESTIMATOR RUN --                    "
echo "         \    \   |__________                                           "
echo "         /\    \             |                                          "
echo "        /__\____\____________|                                          "
echo "------------------------------------------------------------------------"
echo ""

# **************************** GLOBAL VARIABLES ****************************

# Bagfile play rate
BAG_RATE=1

# Estimators
OPENVINS=1
VINSMONO=2

# Absolute Path to cws and to bagfiles
PATH_TO_CWS=${HOME}/${CATKIN_WS}
PATH_TO_BAG=/mnt/host/data

# Attributes dictionary
# ATTR_1 = features
# ATTR_2 = illumination
# ATTR_3 = clutter (not fully implemented yet)
# ATTR_4 = motionblur (not fully implemented yet)
# ATTR_5 = imunoise
# ATTR_6 = timedelay

# Number of attributes (see dictionary above)
ATTRIBUTES=8
EXLUDE_ATTRIBUTES=(3,4,7,8)

# Number of levels per attribute
LEVELS=2

# Number of runs for a single level
RUNS=1

# IMU noise and delay multiplier
IMU_NOISE_MUL=$( echo "scale=9; l(200)/9" | bc -l )
DELAY_MUL=$( echo "scale=4; (5*0.001)/3" | bc -l )

# Fixed optimal imu noise values (2nd batch correspond to imu lvl=5)
init_acc_noise_density=$( echo "scale=5; 1.0e-4" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )
init_gyro_noise_density=$( echo "scale=6; 1.0e-5" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )
init_acc_random_walk=$( echo "scale=6; 1.0e-5" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )
init_gyro_random_walk=$( echo "scale=7; 1.0e-6" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )

fix_acc_noise_density=${init_acc_noise_density}
fix_gyro_noise_density=${init_gyro_noise_density}
fix_acc_random_walk=${init_acc_random_walk}
fix_gyro_random_walk=${init_gyro_random_walk}

# **************************************************************************

# ******************************* FUNCTIONS ********************************

set_vinsmono_params () {
	sed -i -e 's/\bacc_n:.*/acc_n: '${1}'/' ${PATH_TO_CWS}/src/vinseval/vinseval_bringup/config/vinsmono/vinseval.yaml
	sed -i -e 's/\bgyr_n:.*/gyr_n: '${2}'/' ${PATH_TO_CWS}/src/vinseval/vinseval_bringup/config/vinsmono/vinseval.yaml
	sed -i -e 's/\bacc_w:.*/acc_w: '${3}'/' ${PATH_TO_CWS}/src/vinseval/vinseval_bringup/config/vinsmono/vinseval.yaml
	sed -i -e 's/\bgyr_w:.*/gyr_w: '${4}'/' ${PATH_TO_CWS}/src/vinseval/vinseval_bringup/config/vinsmono/vinseval.yaml
	sed -i -e 's/\btd:.*/td: '${5}'/' ${PATH_TO_CWS}/src/vinseval/vinseval_bringup/config/vinsmono/vinseval.yaml
}

# **************************************************************************

trap "exit" INT

# Create folders if they do not exist
for ((attr=1;attr<=$ATTRIBUTES;attr++))
do
   for ((lvl=1;lvl<=$LEVELS;lvl++))
   do
      for ((run=1;run<=$RUNS;run++))
      do
          mkdir -p ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}
      done
   done
done

echo "[WARNING] Deleting previous data"
echo ""
read -n 1 -s -r -p "Press any key to continue"
echo ""
echo ""

# cleanup previous data
rm ${PATH_TO_BAG}/EVAL/*/*/*/*.bag &>/dev/null
rm ${PATH_TO_BAG}/EVAL/*/*/*/*.active &>/dev/null
rm -r ${PATH_TO_BAG}/EVAL/*/*/*/RESULTS &>/dev/null

# Launch the roscore and wait for 5 secs
roscore &>/dev/null &
sleep 5.0

# run the odom2posewithcovariancestamped converter in background
./Odometry2PoseWithCovarianceStamped.py &>/dev/null &

for ((attr=1;attr<=$ATTRIBUTES;attr++))
do
   if [[ "${EXLUDE_ATTRIBUTES[@]}" =~ ${attr} ]]
   then
      continue
   fi
   for ((lvl=1;lvl<=$LEVELS;lvl++))
   do
      for ((run=1;run<=$RUNS;run++))
      do
       echo "Running of attribute: <$attr> level: <$lvl> run: <$run>"
       echo ""
       case $attr in
           5)
               # multiply the default value by IMU_NOISE_MUL*level
               acc_noise_density=$( echo "scale=7; ${lvl}*e(${IMU_NOISE_MUL}*(${lvl}-1))*${init_acc_noise_density}" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )
               gyro_noise_density=$( echo "scale=8; ${lvl}*e(${IMU_NOISE_MUL}*(${lvl}-1))*${init_gyro_noise_density}" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )
               acc_random_walk=$( echo "scale=8; ${lvl}*e(${IMU_NOISE_MUL}*(${lvl}-1))*${init_acc_random_walk}" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )
               gyro_random_walk=$( echo "scale=9; ${lvl}*e(${IMU_NOISE_MUL}*(${lvl}-1))*${init_gyro_random_walk}" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )

	       # OpenVins (change imu init threshold accordigly)
	       if (($lvl < 4))
               then
                 rosbag record -O ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}/EVAL_ATTR_${attr}_LVL_${lvl}_RUN_${run}_EST_${OPENVINS}.bag /pose_est /pose_gt __name:=recordnode &>/dev/null &
                 roslaunch vinseval_bringup vinseval_ov.launch acc_noise_density:=${acc_noise_density} gyro_noise_density:=${gyro_noise_density} acc_random_walk:=${acc_random_walk} gyro_random_walk:=${gyro_random_walk} init_imu_thresh:=0.05 bag:=${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag bag_rate:=${BAG_RATE} &>run_output.log
               elif (($lvl == 4))
               then
                 rosbag record -O ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}/EVAL_ATTR_${attr}_LVL_${lvl}_RUN_${run}_EST_${OPENVINS}.bag /pose_est /pose_gt __name:=recordnode &>/dev/null &
                 roslaunch ov_msckf vinseval.launch acc_noise_density:=${acc_noise_density} gyro_noise_density:=${gyro_noise_density} acc_random_walk:=${acc_random_walk} gyro_random_walk:=${gyro_random_walk} init_imu_thresh:=0.065 bag:=${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag bag_rate:=${BAG_RATE} &>run_output.log
               elif (($lvl == 5))
               then
                 rosbag record -O ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}/EVAL_ATTR_${attr}_LVL_${lvl}_RUN_${run}_EST_${OPENVINS}.bag /pose_est /pose_gt __name:=recordnode &>/dev/null &
                 roslaunch vinseval_bringup vinseval_ov.launch acc_noise_density:=${acc_noise_density} gyro_noise_density:=${gyro_noise_density} acc_random_walk:=${acc_random_walk} gyro_random_walk:=${gyro_random_walk} init_imu_thresh:=0.14 bag:=${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag bag_rate:=${BAG_RATE} &>run_output.log
               elif (($lvl == 6))
               then
                 rosbag record -O ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}/EVAL_ATTR_${attr}_LVL_${lvl}_RUN_${run}_EST_${OPENVINS}.bag /pose_est /pose_gt __name:=recordnode &>/dev/null &
                 roslaunch ov_msckf vinseval.launch acc_noise_density:=${acc_noise_density} gyro_noise_density:=${gyro_noise_density} acc_random_walk:=${acc_random_walk} gyro_random_walk:=${gyro_random_walk} init_imu_thresh:=0.3 bag:=${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag bag_rate:=${BAG_RATE} &>run_output.log
               elif (($lvl == 7))
               then
                 rosbag record -O ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}/EVAL_ATTR_${attr}_LVL_${lvl}_RUN_${run}_EST_${OPENVINS}.bag /pose_est /pose_gt __name:=recordnode &>/dev/null &
                 roslaunch vinseval_bringup vinseval_ov.launch acc_noise_density:=${acc_noise_density} gyro_noise_density:=${gyro_noise_density} acc_random_walk:=${acc_random_walk} gyro_random_walk:=${gyro_random_walk} init_imu_thresh:=0.62 bag:=${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag bag_rate:=${BAG_RATE} &>run_output.log
               elif (($lvl == 8))
               then
                 rosbag record -O ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}/EVAL_ATTR_${attr}_LVL_${lvl}_RUN_${run}_EST_${OPENVINS}.bag /pose_est /pose_gt __name:=recordnode &>/dev/null &
                 roslaunch vinseval_bringup vinseval_ov.launch acc_noise_density:=${acc_noise_density} gyro_noise_density:=${gyro_noise_density} acc_random_walk:=${acc_random_walk} gyro_random_walk:=${gyro_random_walk} init_imu_thresh:=1.26 bag:=${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag bag_rate:=${BAG_RATE} &>run_output.log
               elif (($lvl == 9))
               then
                 rosbag record -O ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}/EVAL_ATTR_${attr}_LVL_${lvl}_RUN_${run}_EST_${OPENVINS}.bag /pose_est /pose_gt __name:=recordnode &>/dev/null &
                 roslaunch vinseval_bringup vinseval_ov.launch acc_noise_density:=${acc_noise_density} gyro_noise_density:=${gyro_noise_density} acc_random_walk:=${acc_random_walk} gyro_random_walk:=${gyro_random_walk} init_imu_thresh:=2.52 bag:=${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag bag_rate:=${BAG_RATE} &>run_output.log
               elif (($lvl == 10))
               then
                 rosbag record -O ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}/EVAL_ATTR_${attr}_LVL_${lvl}_RUN_${run}_EST_${OPENVINS}.bag /pose_est /pose_gt __name:=recordnode &>/dev/null &
                 roslaunch vinseval_bringup vinseval_ov.launch acc_noise_density:=${acc_noise_density} gyro_noise_density:=${gyro_noise_density} acc_random_walk:=${acc_random_walk} gyro_random_walk:=${gyro_random_walk} init_imu_thresh:=5.29 bag:=${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag bag_rate:=${BAG_RATE} &>run_output.log
               fi
	       rosnode kill /recordnode &>/dev/null
               sleep .5

	       # VinsMono
	       set_vinsmono_params ${acc_noise_density} ${gyro_noise_density} ${acc_random_walk} ${gyro_random_walk} 0
	       rosbag record -O ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}/EVAL_ATTR_${attr}_LVL_${lvl}_RUN_${run}_EST_${VINSMONO}.bag /pose_est /pose_gt __name:=recordnode &>/dev/null &
	       roslaunch vinseval_bringup vinseval_vinsmono.launch bag:=${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag &>run_output.log
	       rosnode kill /recordnode &>/dev/null
               sleep .5
               ;;
           6)
	       # add a delay equal to (5/3)*lvl*[1+lvl]*0.001 s
               delay=$( echo "scale=4; ${DELAY_MUL}*${lvl}*(${lvl}-1)" | bc -l )

               # OpenVins (timu = tcam + delay --> (-) in seconds)
	       rosbag record -O ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}/EVAL_ATTR_${attr}_LVL_${lvl}_RUN_${run}_EST_${OPENVINS}.bag /pose_est /pose_gt __name:=recordnode &>/dev/null &
               roslaunch vinseval_bringup vinseval_ov.launch online_calib_time:=true acc_noise_density:=${fix_acc_noise_density} gyro_noise_density:=${fix_gyro_noise_density} acc_random_walk:=${fix_acc_random_walk} gyro_random_walk:=${fix_gyro_random_walk} dt:=-${delay} bag:=${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag bag_rate:=${BAG_RATE} &>run_output.log
	       rosnode kill /recordnode &>/dev/null
               sleep .5

               # VinsMono (timu = tcam + delay --> (-) in seconds)
	       set_vinsmono_params ${fix_acc_noise_density} ${fix_gyro_noise_density} ${fix_acc_random_walk} ${fix_gyro_random_walk} -${delay}
	       rosbag record -O ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}/EVAL_ATTR_${attr}_LVL_${lvl}_RUN_${run}_EST_${VINSMONO}.bag /pose_est /pose_gt __name:=recordnode &>/dev/null &
	       roslaunch vinseval_bringup vinseval_vinsmono.launch bag:=${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag &>run_output.log
	       rosnode kill /recordnode &>/dev/null
               sleep .5
               ;;
           *)
               # OpenVins
	       rosbag record -O ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}/EVAL_ATTR_${attr}_LVL_${lvl}_RUN_${run}_EST_${OPENVINS}.bag /pose_est /pose_gt __name:=recordnode &>/dev/null &
               roslaunch vinseval_bringup vinseval_ov.launch acc_noise_density:=${fix_acc_noise_density} gyro_noise_density:=${fix_gyro_noise_density} acc_random_walk:=${fix_acc_random_walk} gyro_random_walk:=${fix_gyro_random_walk} bag:=${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag bag_rate:=${BAG_RATE} &>run_output.log
	       rosnode kill /recordnode &>/dev/null
               sleep .5

               # VinsMono
               set_vinsmono_params ${fix_acc_noise_density} ${fix_gyro_noise_density} ${fix_acc_random_walk} ${fix_gyro_random_walk} 0.0
	       rosbag record -O ${PATH_TO_BAG}/EVAL/ATTR_${attr}/LVL_${lvl}/RUN_${run}/EVAL_ATTR_${attr}_LVL_${lvl}_RUN_${run}_EST_${VINSMONO}.bag /pose_est /pose_gt __name:=recordnode &>/dev/null &
	       roslaunch vinseval_bringup vinseval_vinsmono.launch bag:=${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag &>run_output.log
	       rosnode kill /recordnode &>/dev/null
               sleep .5
	       ;;
        esac
      done
   done
done
