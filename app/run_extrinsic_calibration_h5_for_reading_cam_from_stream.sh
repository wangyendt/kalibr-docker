cam_imu_kalibr_app_dir="$1"
cam_imu_kalibr_ws_dir="$2"
cam_imu_kalibr_calibrator_dir="$3"
dataset_root="$4"
main_config_yaml_file="$5"
# echo "kalibr_dir: " ${kalibr_dir}
# echo "dataset_root: " ${dataset_root}
# echo "imu_collector_path: " ${imu_collector_path}
# echo "common_py_dir: " ${common_py_dir}

start_time=$(date +%s)

# echo "processing $dataset_root"
device=taurus
which_cam=0
resolution='640x400'

python3 -u ./update_cam-cam-config.py $main_config_yaml_file

if [[ $device == mercury ]]; then
    cp configs/cam-camchain-mercury-${resolution}_cam0.yaml $dataset_root/cam0-camchain-${resolution}.yaml
    cp configs/cam-camchain-mercury-${resolution}_cam1.yaml $dataset_root/cam1-camchain-${resolution}.yaml
    cp configs/imu_mercury.yaml $dataset_root/imu.yaml
elif [[ $device == taurus ]]; then
    cp configs/cam-camchain-taurus-${resolution}_cam0.yaml $dataset_root/cam0-camchain-${resolution}.yaml
    cp configs/imu_taurus.yaml $dataset_root/imu.yaml
    # python3 ./change_imu_format_from_taurus_txt_to_euroc_data_csv.py "$dataset_root"
else
    echo "Unidentified device: " $device
fi
cp configs/april_6x6.yaml $dataset_root/aprilgrid.yaml

# python3 ./sync_imu_cam_timestamp_h5.py "$dataset_root" "$which_cam"

export KALIBR_MANUAL_FOCAL_LENGTH_INIT=500
machine_type=$(uname -m)
if [[ $machine_type == "aarch64" ]]; then
    # 如果是 ARM 架构，导出 LD_PRELOAD 环境变量
    export LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so
fi
# else
#     # 如果不是 ARM 架构，不导出 LD_PRELOAD 环境变量
#     echo "System is not ARM architecture. LD_PRELOAD is not set."
# fi
# export LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so

cd $cam_imu_kalibr_ws_dir
source devel/setup.bash
root=$dataset_root
target=$root/aprilgrid.yaml
camchain_yaml=$root/cam${which_cam}-camchain-${resolution}.yaml
imu_yaml=$root/imu.yaml
# h5_file=$root/${which_cam}_img_${resolution}.h5
image_timestamp_file=$root/${which_cam}_save_timestamp.txt
# imu_data_file=$root/data.txt
imu_data_root=$root
corner_file=$root/cam${which_cam}_${resolution}_corners.pkl

# --target $target --cam $camchain_yaml --imu_yaml $imu_yaml \
# export PYTHONUNBUFFERED=1
rosrun kalibr kalibr_read_data_from_stream \
    --dataset_root $dataset_root \
    --cam_imu_kalibr_app_dir $cam_imu_kalibr_app_dir \
    --cam_imu_kalibr_ws_dir $cam_imu_kalibr_ws_dir \
    --cam_imu_kalibr_calibrator_dir $cam_imu_kalibr_calibrator_dir \
    --target $target --cams $camchain_yaml --imu_yaml $imu_yaml \
    --main_config_yaml_file "$main_config_yaml_file" \
    --image_timestamp_file $image_timestamp_file \
    --imu_data_root $imu_data_root --timeoffset-padding 0.1 \
    --corner_file $corner_file # --no-time-calibration

end_time=$(date +%s)
elapsed_time=$((end_time - start_time))
echo "cam-imu collection任务耗时：$elapsed_time 秒"
