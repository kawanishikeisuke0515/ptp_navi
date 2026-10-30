import math

import pytest

from zed_heading_publisher.calibrate_zed_heading import estimate_soft_iron_calibration
from zed_heading_publisher.calibrate_zed_heading import get_angle_bin_index
from zed_heading_publisher.calibrate_zed_heading import quaternion_to_yaw_rad
from zed_heading_publisher.calibrate_zed_heading import read_calibration_samples_csv
from zed_heading_publisher.zed_heading_publisher import calculate_heading


def _variance_ratio(samples):
    mean_x = sum(x for x, _ in samples) / len(samples)
    mean_z = sum(z for _, z in samples) / len(samples)
    var_x = sum((x - mean_x) ** 2 for x, _ in samples) / len(samples)
    var_z = sum((z - mean_z) ** 2 for _, z in samples) / len(samples)
    return var_x / var_z


def test_axis_aligned_soft_iron_estimate_equalizes_variance():
    raw_samples = [
        (10.0 + 4.0 * math.sin(angle), -3.0 + 2.0 * math.cos(angle))
        for angle in [2.0 * math.pi * index / 180 for index in range(180)]
    ]

    calibration = estimate_soft_iron_calibration(raw_samples)

    corrected_samples = [
        (
            calibration.soft_iron_matrix_00 * (raw_x - calibration.center_x)
            + calibration.soft_iron_matrix_01 * (raw_z - calibration.center_z),
            calibration.soft_iron_matrix_10 * (raw_x - calibration.center_x)
            + calibration.soft_iron_matrix_11 * (raw_z - calibration.center_z),
        )
        for raw_x, raw_z in raw_samples
    ]
    assert calibration.center_x == pytest.approx(10.0)
    assert calibration.center_z == pytest.approx(-3.0)
    assert _variance_ratio(corrected_samples) == pytest.approx(1.0, abs=1e-6)


def test_calculate_heading_applies_soft_iron_matrix_before_heading():
    sample = calculate_heading(
        raw_x=3.0,
        raw_z=4.0,
        center_x=1.0,
        center_z=1.0,
        zero_heading_deg=0.0,
        soft_iron_matrix_00=2.0,
        soft_iron_matrix_01=0.0,
        soft_iron_matrix_10=0.0,
        soft_iron_matrix_11=1.0,
    )

    assert sample.corrected_x == 4.0
    assert sample.corrected_z == 3.0
    assert sample.magnetic_heading_deg == pytest.approx(
        math.degrees(math.atan2(4.0, 3.0))
    )


def test_angle_bin_uses_absolute_wrapped_vio_rotation():
    assert get_angle_bin_index(0.0, 2.0) == 0
    assert get_angle_bin_index(3.9, 2.0) == 1
    assert get_angle_bin_index(-3.9, 2.0) == 1
    assert get_angle_bin_index(361.0, 2.0) == 0


def test_quaternion_to_yaw_rad_extracts_planar_yaw():
    yaw_rad = math.radians(90.0)
    assert quaternion_to_yaw_rad(
        0.0,
        0.0,
        math.sin(yaw_rad / 2.0),
        math.cos(yaw_rad / 2.0),
    ) == pytest.approx(yaw_rad)


def test_read_calibration_samples_csv_supports_vio_yaw_column(tmp_path):
    csv_path = tmp_path / 'samples.csv'
    csv_path.write_text(
        'stamp_sec,raw_x,raw_z,accumulated_vio_yaw_deg\n'
        '1.5,2.0,3.0,45.0\n'
    )

    samples = read_calibration_samples_csv(csv_path)

    assert len(samples) == 1
    assert samples[0].stamp_sec == 1.5
    assert samples[0].raw_x == 2.0
    assert samples[0].raw_z == 3.0
    assert samples[0].accumulated_vio_yaw_deg == 45.0


def test_read_calibration_samples_csv_supports_legacy_raw_columns(tmp_path):
    csv_path = tmp_path / 'samples.csv'
    csv_path.write_text('raw_x,raw_z\n2.0,3.0\n')

    samples = read_calibration_samples_csv(csv_path)

    assert len(samples) == 1
    assert samples[0].stamp_sec == 0.0
    assert samples[0].raw_x == 2.0
    assert samples[0].raw_z == 3.0
    assert samples[0].accumulated_vio_yaw_deg is None


def test_config_update_follows_symlink_and_preserves_other_parameters(tmp_path):
    import yaml
    from zed_heading_publisher.calibrate_zed_heading import update_calibration_config

    config = tmp_path / 'source.yaml'
    config.write_text('zed_heading_publisher:\n  ros__parameters:\n    center_x: 0.0\n    publish_rate_hz: 17.0\nother_node:\n  ros__parameters:\n    enabled: true\n')
    installed = tmp_path / 'installed.yaml'
    installed.symlink_to(config)
    update_calibration_config(installed, {'center_x': 2.5})
    data = yaml.safe_load(config.read_text())
    assert installed.is_symlink()
    assert data['zed_heading_publisher']['ros__parameters'] == {
        'center_x': 2.5, 'publish_rate_hz': 17.0}
    assert data['other_node']['ros__parameters']['enabled'] is True


def test_invalid_config_is_not_overwritten(tmp_path):
    from zed_heading_publisher.calibrate_zed_heading import update_calibration_config

    config = tmp_path / 'invalid.yaml'
    original = 'other_node: {}\n'
    config.write_text(original)
    with pytest.raises(KeyError):
        update_calibration_config(config, {'center_x': 1.0})
    assert config.read_text() == original


@pytest.mark.parametrize('save_config,min_samples,updated', [
    (True, 20, True), (False, 20, False), (True, 100, False),
])
def test_csv_calibration_config_save(tmp_path, save_config, min_samples, updated):
    import os
    import subprocess
    import sys
    import yaml

    config = tmp_path / 'heading.yaml'
    config.write_text('zed_heading_publisher:\n  ros__parameters:\n    center_x: 999.0\n    publish_rate_hz: 17.0\n')
    samples = tmp_path / 'samples.csv'
    samples.write_text('raw_x,raw_z\n' + ''.join(
        f'{10 + 4 * math.cos(i * math.tau / 40)},{-3 + 2 * math.sin(i * math.tau / 40)}\n'
        for i in range(40)))
    result = subprocess.run([
        sys.executable, '-m', 'zed_heading_publisher.calibrate_zed_heading',
        '--ros-args', '-p', f'input_csv_path:={samples}',
        '-p', f'output_config_path:={config}',
        '-p', f'save_config:={str(save_config).lower()}',
        '-p', f'min_samples:={min_samples}',
    ], env={**os.environ, 'ROS_LOG_DIR': str(tmp_path / 'logs'),
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'},
        capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    params = yaml.safe_load(config.read_text())['zed_heading_publisher']['ros__parameters']
    assert params['center_x'] == pytest.approx(10.0 if updated else 999.0)
    assert params['publish_rate_hz'] == 17.0


@pytest.mark.parametrize('merged', [False, True])
def test_source_config_selected_from_regular_install(tmp_path, merged):
    from zed_heading_publisher.calibrate_zed_heading import find_source_config

    source = tmp_path / 'src/zed_heading_publisher/config/zed_heading_publisher.yaml'
    source.parent.mkdir(parents=True)
    source.write_text('source')
    prefix = tmp_path / 'install'
    if not merged:
        prefix /= 'zed_heading_publisher'
    share = prefix / 'share/zed_heading_publisher'
    installed = share / 'config/zed_heading_publisher.yaml'
    installed.parent.mkdir(parents=True)
    installed.write_text('installed')
    module = prefix / 'lib/python3.12/site-packages/zed_heading_publisher/calibrate_zed_heading.py'
    assert find_source_config(module, share) == source
    source.unlink()
    with pytest.raises(FileNotFoundError, match='output_config_path'):
        find_source_config(module, share)


def test_source_config_selected_from_symlink_install(tmp_path):
    from zed_heading_publisher.calibrate_zed_heading import find_source_config

    package = tmp_path / 'src/zed_heading_publisher'
    config = package / 'config/zed_heading_publisher.yaml'
    config.parent.mkdir(parents=True)
    config.write_text('source')
    (package / 'package.xml').touch()
    module = package / 'zed_heading_publisher/calibrate_zed_heading.py'
    module.parent.mkdir()
    module.touch()
    link = tmp_path / 'installed_module.py'
    link.symlink_to(module)
    assert find_source_config(link, tmp_path / 'install/share/zed_heading_publisher') == config
