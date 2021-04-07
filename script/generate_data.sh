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

clear

echo "------------------------------------------------------------------------"
echo "  _____            __________                                           "
echo "  \    \          /          |   - VINSEVAL:                        -   "
echo "   \    \        /  _________|   - EVALUATION FRAMEWORK FOR UNIFIED -   "
echo "    \    \      /  |             - TESTING OF CONSISTENCY AND       -   "
echo "     \    \    /   |_________    - ROBUSTNESS OF VISUAL-INERTIAL    -   "
echo "      \    \  /              |   - NAVIGATION SYSTEMS ALGORITHMS    -   "
echo "       \    \/     __________|                                          "
echo "        \    \    |              -- DATA GENERATION --                  "
echo "         \    \   |__________                                           "
echo "         /\    \             |                                          "
echo "        /__\____\____________|                                          "
echo "------------------------------------------------------------------------"
echo ""

# Absolute Path to cws and to bagfiles
PATH_TO_BAG=~/Catkin_ws/vinseval_cws/bagfiles

# Scene and binary name
SCENE_NAME=gen3dscene
BINARY_NAME=VINSEval_v0.2.1-b1_amd_x86_64

# Render Quality
RENDER=Fastest

# Attributes dictionary
# ATTR_1 = features
# ATTR_2 = illumination
# ATTR_3 = clutter (not fully implemented yet)
# ATTR_4 = motionblur (not fully implemented yet)
# ATTR_5 = imunoise
# ATTR_6 = timedelay
# ATTR_7 = fisheye
# ATTR_8 = resolution

# Number of attributes (see dictionary above)
ATTRIBUTES=8
EXLUDE_ATTRIBUTES=(3,4,7,8)

# Number of levels per attribute
LEVELS=2

# Number of runs for a single level
RUNS=3

# Option for synched rendering (IMU_RATE mod CAM_RATE must be zero)
SYNCH=true
CAM_RATE=40

# Image dimension
IMG_WIDTH=640
IMG_HEIGHT=480

# Camera diagonal FOV
FOV=70

# Image colormap (true, false)
GRAY=true

# Illumination changes direction (up, down, random)
ILL_DIR=down

# IMU noise and delay multiplier
IMU_NOISE_MUL=$( echo "scale=7; l(200)/9" | bc -l )
DELAY_MUL=$( echo "scale=4; (5*0.001)/3" | bc -l )

# Fisheye coefficient step and starting threshold
FISH_THRESH=0.6
FISH_STEP=0.08

# Imu rate
IMU_RATE=200

# Imu params
IS_IMU_NOISY=false
IS_IMU_BODY_REF=false
IS_GRAVITY_ADDED_IMU=false

# Fixed optimal imu noise values
fix_acc_noise_density=$( echo "scale=5; 1.0e-4" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )
fix_gyro_noise_density=$( echo "scale=6; 1.0e-5" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )
fix_acc_random_walk=$( echo "scale=6; 1.0e-5" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )
fix_gyro_random_walk=$( echo "scale=7; 1.0e-6" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )

echo "Selected Scene: ${SCENE_NAME}"
echo "Selected render quality: ${RENDER}"
echo "Synchronized rendering: ${SYNCH}"
echo "Image dimension: ${IMG_WIDTH} x ${IMG_HEIGHT}"
echo "Camera FOV: ${FOV}"
echo "Image grayscale: ${GRAY}"
echo "Number of attributes: ${ATTRIBUTES}"
echo "Exluded attribute from data generation: " ${EXLUDE_ATTRIBUTES}
echo "Number of levels per attribute: ${LEVELS}"
echo "Number of runs per level: ${RUNS}"
echo "Selected render quality: ${RENDER}"
echo ""

trap "exit" INT

# Create folder if they do not exist
for ((attr=1;attr<=$ATTRIBUTES;attr++))
do
   for ((lvl=1;lvl<=$LEVELS;lvl++))
   do
      mkdir -p ${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}
   done
done

echo "[WARNING] Deleting previous data"
echo ""
read -n 1 -s -r -p "Press any key to continue"
echo ""
echo ""

# cleanup previous data
rm ${PATH_TO_BAG}/INPUT/*/*/*.bag &>/dev/null
rm ${PATH_TO_BAG}/INPUT/*/*/*.active &>/dev/null

for ((attr=1;attr<=$ATTRIBUTES;attr++))
do

   for ((lvl=1;lvl<=$LEVELS;lvl++))
   do
       if [[ "${EXLUDE_ATTRIBUTES[@]}" =~ ${attr} ]]
       then
           continue
       fi
       echo "Data generation, attribute: <$attr> level: <$lvl>"
       echo ""

       case $attr in
           5)
               # multiply the default value by IMU_NOISE_MUL*level
               acc_noise_density=$( echo "scale=7; ${lvl}*e(${IMU_NOISE_MUL}*(${lvl}-1))*${fix_acc_noise_density}" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )
               gyro_noise_density=$( echo "scale=8; ${lvl}*e(${IMU_NOISE_MUL}*(${lvl}-1))*${fix_gyro_noise_density}" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )
               acc_random_walk=$( echo "scale=8; ${lvl}*e(${IMU_NOISE_MUL}*(${lvl}-1))*${fix_acc_random_walk}" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )
               gyro_random_walk=$( echo "scale=9; ${lvl}*e(${IMU_NOISE_MUL}*(${lvl}-1))*${fix_gyro_random_walk}" | sed 's/\([+-]\{0,1\}[0-9]*\.\{0,1\}[0-9]\{1,\}\)[eE]+\{0,1\}\(-\{0,1\}\)\([0-9]\{1,\}\)/(\1*10^\2\3)/g' | bc -l )

               roslaunch logichandler logichandler.launch scene_name:=${SCENE_NAME} render_quality:=${RENDER} render_binary_name:=${BINARY_NAME} synch:=${SYNCH} image_width:=${IMG_WIDTH} image_height:=${IMG_HEIGHT} fov:=${FOV} grayscale:=${GRAY} illumination_changes_direction:=${ILL_DIR} attribute:=${attr} attribute_level:=${lvl} number_of_runs:=${RUNS} is_imu_noisy:=${IS_IMU_NOISY} is_imu_body_ref:=${IS_IMU_BODY_REF} is_gravity_added_imu:=${IS_GRAVITY_ADDED_IMU} acc_noise_density:=${acc_noise_density} gyro_noise_density:=${gyro_noise_density} acc_random_walk:=${acc_random_walk} gyro_random_walk:=${gyro_random_walk} imu_rate:=${IMU_RATE} cam_rate:=${CAM_RATE} bagpath:="${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}" &>generate_data_output.log
               ;;
           6)
	       # add a delay equal to (5/3)*lvl*[1+lvl]*0.001 s
               delay=$( echo "scale=4; ${DELAY_MUL}*${lvl}*(${lvl}-1)" | bc -l )
               roslaunch logichandler logichandler.launch scene_name:=${SCENE_NAME} render_quality:=${RENDER} render_binary_name:=${BINARY_NAME} synch:=${SYNCH} image_width:=${IMG_WIDTH} image_height:=${IMG_HEIGHT} fov:=${FOV} grayscale:=${GRAY} illumination_changes_direction:=${ILL_DIR} attribute:=${attr} attribute_level:=${lvl} number_of_runs:=${RUNS} time_delay_s:=${delay} is_imu_noisy:=${IS_IMU_NOISY} is_imu_body_ref:=${IS_IMU_BODY_REF} is_gravity_added_imu:=${IS_GRAVITY_ADDED_IMU} acc_noise_density:=${fix_acc_noise_density} gyro_noise_density:=${fix_gyro_noise_density} acc_random_walk:=${fix_acc_random_walk} gyro_random_walk:=${fix_gyro_random_walk} imu_rate:=${IMU_RATE} cam_rate:=${CAM_RATE} bagpath:="${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}" &>generate_data_output.log
               ;;
           7)
	       # compute fisheye coefficient
               fisheye=$( echo "scale=2; ${FISH_THRESH}+${lvl}*${FISH_STEP}" | bc -l )
               roslaunch logichandler logichandler.launch scene_name:=${SCENE_NAME} render_quality:=${RENDER} render_binary_name:=${BINARY_NAME} synch:=${SYNCH} image_width:=${IMG_WIDTH} image_height:=${IMG_HEIGHT} fov:=${FOV} grayscale:=${GRAY} illumination_changes_direction:=${ILL_DIR} attribute:=${attr} attribute_level:=${lvl} number_of_runs:=${RUNS} fisheye:=${fisheye} is_imu_noisy:=${IS_IMU_NOISY} is_imu_body_ref:=${IS_IMU_BODY_REF} is_gravity_added_imu:=${IS_GRAVITY_ADDED_IMU} acc_noise_density:=${fix_acc_noise_density} gyro_noise_density:=${fix_gyro_noise_density} acc_random_walk:=${fix_acc_random_walk} gyro_random_walk:=${fix_gyro_random_walk} imu_rate:=${IMU_RATE} cam_rate:=${CAM_RATE} bagpath:="${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}" &>generate_data_output.log
               ;;
           8)
	       # compute fisheye coefficient
               res_x=$( echo "scale=2; ${IMG_WIDTH}*${lvl}" | bc -l )
               res_y=$( echo "scale=2; ${IMG_HEIGHT}*${lvl}" | bc -l )
               echo "Resolution: ${res_x}x${res_y}"
               roslaunch logichandler logichandler.launch scene_name:=${SCENE_NAME} render_quality:=${RENDER} render_binary_name:=${BINARY_NAME} synch:=${SYNCH} image_width:=${res_x} image_height:=${res_y} fov:=${FOV} grayscale:=${GRAY} illumination_changes_direction:=${ILL_DIR} attribute:=${attr} attribute_level:=${lvl} number_of_runs:=${RUNS} is_imu_noisy:=${IS_IMU_NOISY} is_imu_body_ref:=${IS_IMU_BODY_REF} is_gravity_added_imu:=${IS_GRAVITY_ADDED_IMU} acc_noise_density:=${fix_acc_noise_density} gyro_noise_density:=${fix_gyro_noise_density} acc_random_walk:=${fix_acc_random_walk} gyro_random_walk:=${fix_gyro_random_walk} imu_rate:=${IMU_RATE} cam_rate:=${CAM_RATE} bagpath:="${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}" &>generate_data_output.log
               ;;
           *)
               roslaunch logichandler logichandler.launch scene_name:=${SCENE_NAME} render_quality:=${RENDER} render_binary_name:=${BINARY_NAME} synch:=${SYNCH} image_width:=${IMG_WIDTH} image_height:=${IMG_HEIGHT} fov:=${FOV} grayscale:=${GRAY} illumination_changes_direction:=${ILL_DIR} attribute:=${attr} attribute_level:=${lvl} number_of_runs:=${RUNS} is_imu_noisy:=${IS_IMU_NOISY} is_imu_body_ref:=${IS_IMU_BODY_REF} is_gravity_added_imu:=${IS_GRAVITY_ADDED_IMU} acc_noise_density:=${fix_acc_noise_density} gyro_noise_density:=${fix_gyro_noise_density} acc_random_walk:=${fix_acc_random_walk} gyro_random_walk:=${fix_gyro_random_walk} imu_rate:=${IMU_RATE} cam_rate:=${CAM_RATE} bagpath:="${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}" &>generate_data_output.log
               ;;
        esac

      sleep 1.0

      # Check if there are active bagfiles
      while [ -f ${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/*.active ]
      do
         sleep 0.5
      done

      sleep 5.0

      # Realign all the bagfiles
      for ((run=1;run<=$RUNS;run++))
      do
         mv ${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag ${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.origin.bag &>generate_data_output.log
         python3 switch_bag_time.py -i ${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.origin.bag -o ${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.bag &>generate_data_output.log
      	 rm ${PATH_TO_BAG}/INPUT/ATTR_${attr}/LVL_${lvl}/INPUT_ATTR_${attr}_LVL_${lvl}_RUN_${run}.origin.bag &>generate_data_output.log
      done
   done
done
