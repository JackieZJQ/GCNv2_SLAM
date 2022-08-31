#! /bin/bash

for ((i = 1; i <= 20; i++))
do
  FULL_RESOLUTION=1 ./Build/Release/Source/Examples/Monocular/mono_tum TUM1.yaml ../Datasets/TUM/rgbd_dataset_freiburg1_xyz result.txt >> rpe-20-fr1-xyz-method-1.txt && evo_rpe tum ../Datasets/TUM/rgbd_dataset_freiburg1_xyz/groundtruth.txt result.txt -a -s >> rpe-20-fr1-xyz-method-1.txt
done

for ((i = 1; i <= 20; i++))
do
	FULL_RESOLUTION=1 ./Build/Release/Source/Examples/Monocular/mono_tum TUM1.yaml ../Datasets/TUM2/fr3/rgbd_dataset_freiburg3_nostructure_notexture_far result.txt >> rpe-20-fr3-nnf-method-1.txt && evo_rpe tum ../Datasets/TUM2/fr3/rgbd_dataset_freiburg3_nostructure_notexture_far/groundtruth.txt result.txt -a -s >> rpe-20-fr3-nnf-method-1.txt
done

for ((i = 1; i <= 20; i++))
do
	FULL_RESOLUTION=1 ./Build/Release/Source/Examples/Monocular/mono_tum TUM1.yaml ../Datasets/TUM2/fr1/rgbd_dataset_freiburg1_floor result.txt >> rpe-20-fr1-floor-method-1.txt && evo_rpe tum ../Datasets/TUM2/fr1/rgbd_dataset_freiburg1_floor/groundtruth.txt result.txt -a -s >> rpe-20-fr1-floor-method-1.txt
done