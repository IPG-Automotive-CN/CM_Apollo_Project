# Example of the command to run that generates the c++ files for gnss_best_pose
# Install protobuf-dev
# Go to the cpp_generated folder
protoc -I=../ --cpp_out=../src ../modules/drivers/gnss/proto/gnss_best_pose.proto 

# To generate cpp for all files in the folder 
find .. -name "*.proto" -exec protoc -I=../ --cpp_out=../src {}  \;


./modules/transform/proto/transform.proto
./modules/transform/proto/static_transform_conf.proto
./modules/perception/lidar/lib/segmentation/ncut/proto/ncut_param.proto
./modules/perception/lidar/lib/segmentation/cnnseg/proto/spp_engine_config.proto
./modules/perception/lidar/lib/segmentation/cnnseg/proto/cnnseg_param.proto
./modules/perception/lidar/lib/segmentation/cnnseg/proto/cnnseg_config.proto
./modules/perception/lidar/lib/object_filter_bank/proto/filter_bank_config.proto
./modules/perception/lidar/lib/pointcloud_preprocessor/proto/pointcloud_preprocessor_config.proto
./modules/perception/lidar/lib/ground_detector/ground_service_detector/proto/ground_service_detector_config.proto
./modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector/proto/spatio_temporal_ground_detector_config.proto
./modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/proto/hdmap_roi_filter.proto
./modules/perception/lidar/lib/scene_manager/roi_service/proto/roi_service.proto
./modules/perception/lidar/lib/scene_manager/ground_service/proto/ground_service_config.proto
./modules/perception/lidar/lib/scene_manager/proto/scene_manager_config.proto
./modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.proto
./modules/perception/lidar/app/proto/lidar_obstacle_tracking_config.proto
./modules/perception/lidar/app/proto/lidar_obstacle_segmentation_config.proto
./modules/perception/camera/lib/feature_extractor/tfe/tracking_feature.proto
./modules/perception/camera/lib/lane/postprocessor/denseline/denseline_postprocessor.proto
./modules/perception/camera/lib/lane/postprocessor/darkSCNN/darkSCNN_postprocessor.proto
./modules/perception/camera/lib/lane/common/darkSCNN.proto
./modules/perception/camera/lib/lane/common/denseline.proto
./modules/perception/camera/lib/obstacle/postprocessor/location_refiner/location_refiner.proto
./modules/perception/camera/lib/obstacle/detector/yolo/proto/yolo.proto
./modules/perception/camera/lib/obstacle/transformer/multicue/multicue.proto
./modules/perception/camera/lib/obstacle/tracker/omt/omt.proto
./modules/perception/camera/lib/traffic_light/preprocessor/tl_preprocess.proto
./modules/perception/camera/lib/traffic_light/detector/detection/detection.proto
./modules/perception/camera/lib/traffic_light/detector/recognition/recognition.proto
./modules/perception/camera/lib/traffic_light/tracker/semantic.proto
./modules/perception/camera/common/proto/object_template_meta_schema.proto
./modules/perception/camera/app/perception.proto
./modules/perception/proto/rt.proto
./modules/perception/proto/dst_type_fusion_config.proto
./modules/perception/proto/probabilistic_fusion_config.proto
./modules/perception/proto/map_manager_config.proto
./modules/perception/proto/traffic_light_detection.proto
./modules/perception/proto/pbf_tracker_config.proto
./modules/perception/proto/perception_obstacle.proto
./modules/perception/proto/perception_camera.proto
./modules/perception/proto/dst_existance_fusion_config.proto
./modules/perception/proto/perception_config_schema.proto
./modules/perception/proto/hm_tracker_config.proto
./modules/perception/proto/ccrf_type_fusion_config.proto
./modules/perception/proto/perception_ultrasonic.proto
./modules/perception/proto/tracker_config.proto
./modules/perception/proto/fused_classifier_config.proto
./modules/perception/proto/roi_boundary_filter_config.proto
./modules/perception/proto/sensor_meta_schema.proto
./modules/perception/proto/perception_lane.proto
./modules/perception/proto/motion_service.proto
./modules/perception/onboard/proto/radar_component_config.proto
./modules/perception/onboard/proto/trafficlights_perception_component.proto
./modules/perception/onboard/proto/fusion_camera_detection_component.proto
./modules/perception/onboard/proto/lidar_component_config.proto
./modules/perception/onboard/proto/fusion_component_config.proto
./modules/perception/onboard/proto/lane_perception_component.proto
./modules/perception/onboard/proto/motion_service.proto
./modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper/proto/pbf_gatekeeper_config.proto
./modules/v2x/proto/v2x_service_obu_to_car.proto
./modules/v2x/proto/v2x_service_car_to_obu.proto
./modules/v2x/proto/v2x_carstatus.proto
./modules/v2x/proto/v2x_traffic_light.proto
./modules/canbus/proto/lexus.proto
./modules/canbus/proto/vehicle_parameter.proto
./modules/canbus/proto/canbus_conf.proto
./modules/canbus/proto/ge3.proto
./modules/canbus/proto/wey.proto
./modules/canbus/proto/chassis_detail.proto
./modules/canbus/proto/chassis.proto
./modules/canbus/proto/zhongyun.proto
./modules/canbus/proto/transit.proto
./modules/canbus/proto/ch.proto
./modules/localization/proto/gnss_pnt_result.proto
./modules/localization/proto/imu.proto
./modules/localization/proto/sins_pva.proto
./modules/localization/proto/gps.proto
./modules/localization/proto/localization.proto
./modules/localization/proto/localization_config.proto
./modules/localization/proto/localization_status.proto
./modules/localization/proto/measure.proto
./modules/localization/proto/rtk_config.proto
./modules/localization/proto/pose.proto
./modules/common/configs/proto/vehicle_config.proto
./modules/common/adapters/proto/adapter_config.proto
./modules/common/vehicle_state/proto/vehicle_state.proto
./modules/common/vehicle_model/proto/vehicle_model_config.proto
./modules/common/proto/drive_event.proto
./modules/common/proto/drive_state.proto
./modules/common/proto/pnc_point.proto
./modules/common/proto/vehicle_signal.proto
./modules/common/proto/error_code.proto
./modules/common/proto/header.proto
./modules/common/proto/geometry.proto
./modules/common/monitor_log/proto/monitor_log.proto
./modules/common/util/testdata/simple.proto
./modules/prediction/proto/fnn_vehicle_model.proto
./modules/prediction/proto/fnn_model_base.proto
./modules/prediction/proto/feature.proto
./modules/prediction/proto/prediction_output_evaluation.proto
./modules/prediction/proto/offline_features.proto
./modules/prediction/proto/prediction_obstacle.proto
./modules/prediction/proto/prediction_conf.proto
./modules/prediction/proto/lane_graph.proto
./modules/prediction/proto/scenario.proto
./modules/prediction/proto/network_model.proto
./modules/prediction/proto/semantic_map_config.proto
./modules/prediction/proto/network_layers.proto
./modules/prediction/proto/prediction_point.proto
./modules/control/proto/pid_conf.proto
./modules/control/proto/lon_controller_conf.proto
./modules/control/proto/control_conf.proto
./modules/control/proto/mpc_controller_conf.proto
./modules/control/proto/lat_controller_conf.proto
./modules/control/proto/control_cmd.proto
./modules/control/proto/gain_scheduler_conf.proto
./modules/control/proto/calibration_table.proto
./modules/control/proto/pad_msg.proto
./modules/control/proto/leadlag_conf.proto
./modules/dreamview/proto/hmi_mode.proto
./modules/dreamview/proto/hmi_config.proto
./modules/dreamview/proto/hmi_status.proto
./modules/dreamview/proto/simulation_world.proto
./modules/dreamview/proto/data_collection_table.proto
./modules/dreamview/proto/chart.proto
./modules/dreamview/proto/point_cloud.proto
./modules/map/relative_map/proto/navigator_config.proto
./modules/map/relative_map/proto/relative_map_config.proto
./modules/map/relative_map/proto/navigation.proto
./modules/map/proto/map_lane.proto
./modules/map/proto/map.proto
./modules/map/proto/map_speed_control.proto
./modules/map/proto/map_yield_sign.proto
./modules/map/proto/map_id.proto
./modules/map/proto/map_junction.proto
./modules/map/proto/map_stop_sign.proto
./modules/map/proto/map_overlap.proto
./modules/map/proto/map_signal.proto
./modules/map/proto/map_geometry.proto
./modules/map/proto/map_pnc_junction.proto
./modules/map/proto/map_parking_space.proto
./modules/map/proto/map_speed_bump.proto
./modules/map/proto/map_road.proto
./modules/map/proto/map_crosswalk.proto
./modules/map/proto/map_clear_area.proto
./modules/bridge/proto/udp_bridge_remote_info.proto
./modules/third_party_perception/proto/radar_obstacle.proto
./modules/drivers/canbus/proto/can_card_parameter.proto
./modules/drivers/canbus/proto/sensor_canbus_conf.proto
./modules/drivers/gnss/proto/gnss_status.proto
./modules/drivers/gnss/proto/config.proto
./modules/drivers/gnss/proto/imu.proto
./modules/drivers/gnss/proto/gnss.proto
./modules/drivers/gnss/proto/gnss_best_pose.proto
./modules/drivers/gnss/proto/gnss_raw_observation.proto
./modules/drivers/gnss/proto/heading.proto
./modules/drivers/gnss/proto/ins.proto
./modules/drivers/camera/proto/config.proto
./modules/drivers/velodyne/proto/config.proto
./modules/drivers/velodyne/proto/velodyne.proto
./modules/drivers/radar/racobit_radar/proto/racobit_radar_conf.proto
./modules/drivers/radar/conti_radar/proto/conti_radar_conf.proto
./modules/drivers/radar/ultrasonic_radar/proto/ultrasonic_radar_conf.proto
./modules/drivers/proto/sensor_image.proto
./modules/drivers/proto/radar.proto
./modules/drivers/proto/racobit_radar.proto
./modules/drivers/proto/conti_radar.proto
./modules/drivers/proto/ultrasonic_radar.proto
./modules/drivers/proto/mobileye.proto
./modules/drivers/proto/pointcloud.proto
./modules/drivers/proto/delphi_esr.proto
./modules/drivers/video/proto/video_h265cfg.proto
./modules/drivers/tools/image_decompress/proto/config.proto
./modules/guardian/proto/guardian_conf.proto
./modules/guardian/proto/guardian.proto
./modules/routing/proto/routing.proto
./modules/routing/proto/topo_graph.proto
./modules/routing/proto/routing_config.proto
./modules/routing/proto/poi.proto
./modules/tools/navigator/dbmap/proto/dbmap.proto
./modules/tools/prediction/data_pipelines/proto/fnn_model.proto
./modules/tools/prediction/data_pipelines/proto/cruise_model.proto
./modules/tools/fuel_proxy/proto/job_config.proto
./modules/tools/sensor_calibration/proto/extractor_config.proto
./modules/data/proto/frame.proto
./modules/data/proto/static_info.proto
./modules/data/tools/smart_recorder/proto/smart_recorder_triggers.proto
./modules/data/tools/smart_recorder/proto/smart_recorder_status.proto
./modules/planning/proto/open_space_fallback_decider_config.proto
./modules/planning/proto/cos_theta_smoother_config.proto
./modules/planning/proto/piecewise_jerk_speed_config.proto
./modules/planning/proto/spiral_curve_config.proto
./modules/planning/proto/reference_line_smoother_config.proto
./modules/planning/proto/navi_speed_decider_config.proto
./modules/planning/proto/lattice_structure.proto
./modules/planning/proto/piecewise_jerk_path_config.proto
./modules/planning/proto/lattice_sampling_config.proto
./modules/planning/proto/open_space_roi_decider_config.proto
./modules/planning/proto/traffic_rule_config.proto
./modules/planning/proto/planner_open_space_config.proto
./modules/planning/proto/planning_stats.proto
./modules/planning/proto/lane_change_decider_config.proto
./modules/planning/proto/open_space_trajectory_provider_config.proto
./modules/planning/proto/decider_config.proto
./modules/planning/proto/open_space_pre_stop_decider_config.proto
./modules/planning/proto/planning_config.proto
./modules/planning/proto/planning_status.proto
./modules/planning/proto/open_space_trajectory_partition_config.proto
./modules/planning/proto/path_assessment_decider_config.proto
./modules/planning/proto/planning.proto
./modules/planning/proto/navi_obstacle_decider_config.proto
./modules/planning/proto/path_bounds_decider_config.proto
./modules/planning/proto/speed_bounds_decider_config.proto
./modules/planning/proto/auto_tuning_model_input.proto
./modules/planning/proto/dp_st_speed_config.proto
./modules/planning/proto/path_decider_info.proto
./modules/planning/proto/navi_path_decider_config.proto
./modules/planning/proto/path_lane_borrow_decider_config.proto
./modules/planning/proto/proceed_with_caution_speed_config.proto
./modules/planning/proto/auto_tuning_raw_feature.proto
./modules/planning/proto/qp_problem.proto
./modules/planning/proto/sl_boundary.proto
./modules/planning/proto/rule_based_stop_decider_config.proto
./modules/planning/proto/decision.proto
./modules/planning/proto/pad_msg.proto
./modules/planning/proto/fem_pos_deviation_smoother_config.proto
./modules/planning/proto/planning_internal.proto
./modules/monitor/proto/system_status.proto
./cyber/examples/proto/examples.proto
./cyber/proto/cyber_conf.proto
./cyber/proto/transport_conf.proto
./cyber/proto/choreography_conf.proto
./cyber/proto/dag_conf.proto
./cyber/proto/role_attributes.proto
./cyber/proto/run_mode_conf.proto
./cyber/proto/unit_test.proto
./cyber/proto/parameter.proto
./cyber/proto/proto_desc.proto
./cyber/proto/topology_change.proto
./cyber/proto/component_conf.proto
./cyber/proto/scheduler_conf.proto
./cyber/proto/record.proto
./cyber/proto/perception.proto
./cyber/proto/qos_profile.proto
./cyber/proto/classic_conf.proto