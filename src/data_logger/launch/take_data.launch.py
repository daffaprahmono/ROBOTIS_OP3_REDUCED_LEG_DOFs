from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ===================== Launch Arguments =====================
    # --- umum ---
    output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value='~/data_pengujian',
        description='Folder output CSV data_logger'
    )
    output_filename = DeclareLaunchArgument(
        'output_filename',
        default_value='',
        description='Nama file CSV (kosong = auto timestamp)'
    )

    # --- data_logger ---
    log_hz = DeclareLaunchArgument(
        'log_hz',
        default_value='10.0',
        description='Frekuensi logging data_logger (Hz). Contoh: 10 untuk stand/push, 20 untuk walk.'
    )
    flush_every_n = DeclareLaunchArgument(
        'flush_every_n',
        default_value='50',
        description='Flush file setiap N baris'
    )
    include_battery = DeclareLaunchArgument(
        'include_battery',
        default_value='true',
        description='Log battery_voltage (true/false)'
    )
    current_scale = DeclareLaunchArgument(
        'current_scale',
        default_value='0.00269',
        description='Skala konversi raw current -> Ampere (default XM: 0.00269 A/LSB)'
    )
    volt_scale = DeclareLaunchArgument(
        'volt_scale',
        default_value='0.1',
        description='Skala konversi raw voltage -> Volt (kamu bilang raw perlu dikali 0.1)'
    )

    # --- action_sequencer ---
    walk_duration = DeclareLaunchArgument(
        'walk_duration',
        default_value='10.0',
        description='Durasi jalan (detik)'
    )
    step_interval = DeclareLaunchArgument(
        'step_interval',
        default_value='0.05',
        description='Interval publish page 90 (detik). Kecil supaya tidak ada gap yang bikin jatuh.'
    )
    start_page = DeclareLaunchArgument(
        'start_page',
        default_value='90',
        description='Page untuk jalan'
    )
    stop_page = DeclareLaunchArgument(
        'stop_page',
        default_value='91',
        description='Page untuk berhenti/berdiri'
    )
    warmup_sec = DeclareLaunchArgument(
        'warmup_sec',
        default_value='2.0',
        description='Delay awal sebelum set module'
    )
    post_module_wait_sec = DeclareLaunchArgument(
        'post_module_wait_sec',
        default_value='2.0',
        description='Delay setelah action_module aktif'
    )
    stop_repeat = DeclareLaunchArgument(
        'stop_repeat',
        default_value='3',
        description='Berapa kali publish stop_page (biar yakin masuk)'
    )
    stop_repeat_interval = DeclareLaunchArgument(
        'stop_repeat_interval',
        default_value='0.2',
        description='Interval antar publish stop_page'
    )

    # --- event_marker (opsional) ---
    run_event_marker = DeclareLaunchArgument(
        'run_event_marker',
        default_value='false',
        description='Jalankan event_marker (true/false). Dipakai untuk pengujian push.'
    )

    # ===================== Nodes =====================

    # 1) Data Logger
    data_logger_node = Node(
        package='data_logger',
        executable='data_logger_node',
        name='data_logger_node',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'output_filename': LaunchConfiguration('output_filename'),

            'log_hz': ParameterValue(LaunchConfiguration('log_hz'), value_type=float),
            'flush_every_n': ParameterValue(LaunchConfiguration('flush_every_n'), value_type=int),
            'include_battery': ParameterValue(LaunchConfiguration('include_battery'), value_type=bool),

            'current_scale': ParameterValue(LaunchConfiguration('current_scale'), value_type=float),
            'volt_scale': ParameterValue(LaunchConfiguration('volt_scale'), value_type=float),
        }]
    )

    # 2) Action Sequencer (walk test)
    action_sequencer_node = Node(
        package='data_logger',
        executable='action_sequencer',
        name='action_sequencer',
        output='screen',
        parameters=[{
            'walk_duration': ParameterValue(LaunchConfiguration('walk_duration'), value_type=float),
            'step_interval': ParameterValue(LaunchConfiguration('step_interval'), value_type=float),
            'start_page': ParameterValue(LaunchConfiguration('start_page'), value_type=int),
            'stop_page': ParameterValue(LaunchConfiguration('stop_page'), value_type=int),
            'warmup_sec': ParameterValue(LaunchConfiguration('warmup_sec'), value_type=float),
            'post_module_wait_sec': ParameterValue(LaunchConfiguration('post_module_wait_sec'), value_type=float),
            'stop_repeat': ParameterValue(LaunchConfiguration('stop_repeat'), value_type=int),
            'stop_repeat_interval': ParameterValue(LaunchConfiguration('stop_repeat_interval'), value_type=float),
        }]
    )

    # 3) Event Marker (opsional, untuk push test)
    event_marker_node = Node(
        package='data_logger',
        executable='event_marker',
        name='event_marker',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_event_marker'))
    )

    # ===================== LaunchDescription =====================
    return LaunchDescription([
        # args
        output_dir, output_filename,
        log_hz, flush_every_n, include_battery, current_scale, volt_scale,
        walk_duration, step_interval, start_page, stop_page, warmup_sec, post_module_wait_sec,
        stop_repeat, stop_repeat_interval,
        run_event_marker,

        # nodes
        data_logger_node,
        action_sequencer_node,
        event_marker_node,
    ])
