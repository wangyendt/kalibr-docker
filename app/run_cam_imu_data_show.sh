dataset_root=${1:-/media/psf/work/code/project/ffalcon/production_calibration/data/2024_01_01_00_00_00/cam_imu}
kalibr_dir=${2:-/media/psf/work/code/project/ffalcon/production_calibration/module/cam_imu}
fixture_id=$3
config_yaml_file=$4
# kalibr_dir=/home/wayne/work/code/catkin_wss/kalibr_ws

echo $dataset_root
echo $kalibr_dir
echo $fixture_id
echo $config_yaml_file

device=taurus
# device=mercury
which_cam=0
# resolution='1920x1200'
resolution='640x400'

start_time=$(date +%s)


# 设备配置函数
configure_device() {
    local device=$1
    local resolution=$2
    echo 'Show data start...'

    case $device in
    mercury)
        # cp configs/cam-camchain-mercury-${resolution}_cam1.yaml $dataset_root/cam1-camchain-${resolution}.yaml
        cp configs/cam-camchain-mercury-${resolution}_cam0.yaml $dataset_root/cam0-camchain-${resolution}.yaml
        cp configs/imu_mercury.yaml $dataset_root/imu.yaml
        ;;
    taurus)
        # cp configs/cam-camchain-taurus-${resolution}_cam0.yaml $dataset_root/cam0-camchain-${resolution}.yaml
        # cp configs/imu_taurus.yaml $dataset_root/imu.yaml
        # python3 ./change_imu_format_from_taurus_txt_to_euroc_data_csv.py "$dataset_root" --interpolation
        python3 -u ./change_imu_format_from_taurus_txt_to_euroc_data_csv.py "$dataset_root" "$fixture_id" "$config_yaml_file"
        ;;
    *)
        echo "Unidentified device: $device"
        exit 1
        ;;
    esac

    cp configs/april_6x6.yaml $dataset_root/aprilgrid.yaml
    python3 -u ./sync_imu_cam_timestamp_h5.py "$dataset_root" "$which_cam" "$fixture_id"
}

# 运行kalibr的函数
run_cam_imu_show() {
    echo "Ready to run Kalibr..."
    cd $kalibr_dir || exit
    export KALIBR_MANUAL_FOCAL_LENGTH_INIT=500
    source devel/setup.bash
    machine_type=$(uname -m)
    if [[ $machine_type == "aarch64" ]]; then
        # 如果是 ARM 架构，导出 LD_PRELOAD 环境变量
        export LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so
    fi

    local root=$dataset_root
    local target=$root/aprilgrid.yaml
    local camchain_yaml=$root/cam${which_cam}-camchain-${resolution}.yaml
    local imu_yaml=$root/imu.yaml
    local corner_file=$root/cam${which_cam}_${resolution}_corners.pkl
    local image_timestamp_file=$root/${which_cam}_save_timestamp.txt
    local imu_data_file=$root/data"$fixture_id".csv

    rosrun kalibr kalibr_show_cam_imu_data \
        --target $target --cam $camchain_yaml --imu $imu_yaml \
        --corner_file $corner_file --image_timestamp_file $image_timestamp_file \
        --imu_data_file $imu_data_file --timeoffset-padding 0.04 \
        --fixture_id $fixture_id \
        --dont-show-report | tee $root/cam2imu_calibration.log
        # --export-poses | tee $root/cam2imu_calibration.log
}

# 主逻辑
configure_device $device $resolution
run_cam_imu_show

end_time=$(date +%s)
elapsed_time=$((end_time - start_time))
echo "cam-imu calibration任务耗时：$elapsed_time 秒"
