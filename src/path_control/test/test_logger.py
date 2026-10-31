import csv
import json
import time

from path_control.log_store import LogStore


def rows(directory, name):
    with (directory / f'{name}.csv').open() as handle:
        return list(csv.DictReader(handle))


def test_segments_missing_stale_and_raw_samples(tmp_path):
    store = LogStore(tmp_path, {}, stale_after_sec=.5)
    store.timeline(1, 1)
    for tick, a in enumerate([(1, 2), (1, 2), None, (1, 2)], 2):
        values = dict(state='INIT', start_x=a[0] if a else None,
                      start_y=a[1] if a else None, raw_json='{}')
        store.record('status', values, tick, tick)
    store.record('cmd_vel', dict(linear_x=.1, linear_y=0, angular_z=.2), 8, 8)
    store.timeline(9, 600000009)
    store.close()
    assert [row['event'] for row in rows(store.directory, 'events')] == [
        'start_acquired', 'start_cleared', 'start_acquired']
    assert [row['segment_id'] for row in rows(store.directory, 'status')] == ['1', '1', '1', '2']
    timeline = rows(store.directory, 'timeline')
    assert timeline[0]['optitrack_received'] == 'False'
    assert timeline[0]['optitrack_x'] == ''
    assert timeline[1]['cmd_vel_stale'] == 'True'
    assert timeline[1]['cmd_vel_segment_id'] == '2'
    assert len(rows(store.directory, 'cmd_vel')) == 1


def test_ros_logging_and_parameter_snapshot(tmp_path):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, Twist
    from std_msgs.msg import String
    from uwb_interfaces.msg import UwbPosition
    from path_control.logger import PathControlLogger

    rclpy.init(args=['--ros-args', '-p', f'output_dir:={tmp_path}'])
    logger = probe = None
    try:
        logger = PathControlLogger()
        # Parameter service fixture also verifies asynchronous metadata collection.
        probe = Node('path_control')
        probe.declare_parameter('goal_x', 2.5)
        pubs = [probe.create_publisher(kind, topic, 10) for kind, topic in (
            (String, '/path_control/status'), (Twist, '/rov_cmd_vel'),
            (PoseStamped, '/vrpn_mocap/RigidBody_1/pose'),
            (UwbPosition, '/uwb/position'), (PoseStamped, '/uwb/robot_pose'))]

        def spin_until(condition):
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                rclpy.spin_once(logger, timeout_sec=.01)
                rclpy.spin_once(probe, timeout_sec=.01)
                if condition():
                    return
            raise AssertionError('ROS logger condition timed out')

        spin_until(lambda: all(pub.get_subscription_count() for pub in pubs))
        status = String(data=json.dumps(dict(state='DRIVE', start=[1, 2], goal=[3, 4])))
        pose = PoseStamped()
        pose.header.frame_id = 'mocap'
        pose.header.stamp = probe.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.
        position = UwbPosition(x_m=1.2, y_m=2.3, valid=True)
        for pub, msg in zip(pubs, [status, Twist(), pose, position, pose]):
            pub.publish(msg)
        spin_until(lambda: len(logger.store.latest) == 5 and
                   '/path_control' in logger.store.metadata['parameter_snapshots'])
        assert logger.store.latest['optitrack']['frame_id'] == 'mocap'
        assert logger.store.latest['uwb_position']['value_valid']
        assert logger.store.metadata['parameter_snapshots']['/path_control']['parameters']['goal_x'] == 2.5
        logger.status(String(data='broken JSON'))
        assert not logger.store.latest['status']['value_valid']
        directory = logger.store.directory
        logger.destroy_node()
        logger = None
        assert len(rows(directory, 'status')) == 2
        assert rows(directory, 'cmd_vel')[0]['source_stamp_ns'] == ''
        assert (directory / 'metadata.yaml').is_file()
    finally:
        if logger is not None:
            logger.destroy_node()
        if probe is not None:
            probe.destroy_node()
        rclpy.shutdown()
