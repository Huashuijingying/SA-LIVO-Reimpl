#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Integration regression: bursty ROS replay and a bounded direct feeder.

Supply a short, raw-image ROS2 bag compatible with the two parameter files.
The shell must have ROS2 and the Livox/vikit dependencies sourced.
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import sqlite3
import subprocess
import time


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def run(args, mode, expected):
    logdir = args.output / mode
    logdir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=str(30 + os.getpid() % 150),
               ROS_LOG_DIR=str(logdir / 'ros'), OMP_NUM_THREADS='1')
    command = [str(args.binary), '--ros-args', '--params-file', str(args.params),
               '--params-file', str(args.camera), '-p', 'common.img_buffer_max:=2',
               '-p', 'common.lossless_replay_qos:=true', '-p', 'evo.pose_output_en:=false',
               '-p', 'pcd_save.pcd_save_en:=false', '-p', 'pcd_save.colmap_output_en:=false']
    if mode == 'direct':
        command += ['-p', 'common.direct_bag_path:=' + str(args.bag)]
    node = player = None
    with (logdir / 'node.log').open('w') as nl, (logdir / 'player.log').open('w') as pl:
        try:
            node = subprocess.Popen(command, env=env, stdout=nl, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            if mode == 'direct':
                node.wait(timeout=args.timeout)
            else:
                time.sleep(2)
                assert node.poll() is None, 'mapper exited during startup'
                qos = logdir / 'qos.yaml'
                qos.write_text(''.join(f'{topic}:\n  history: keep_all\n  reliability: reliable\n'
                                       '  durability: volatile\n' for topic in args.topics))
                player = subprocess.Popen(
                    ['ros2', 'bag', 'play', str(args.bag), '--rate', '20',
                     '--disable-keyboard-controls', '--qos-profile-overrides-path', str(qos),
                     '--wait-for-all-acked', '10000', '--topics', *args.topics],
                    env=env, stdout=pl, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic() + args.timeout
                while time.monotonic() < deadline:
                    assert node.poll() is None, 'mapper exited while processing burst'
                    s = (logdir / 'node.log').read_text(errors='replace')
                    counts = [s.count(t) for t in ('Get image, its header time:',
                              'Get LiDAR, its header time:', 'get imu at time:')]
                    if player.poll() == 0 and counts == expected:
                        # Let processing finish before testing normal shutdown.
                        time.sleep(1)
                        break
                    time.sleep(.1)
                else:
                    raise AssertionError(f'burst stalled or lost input: {counts} != {expected}')
                stop(node)
            assert node.returncode == 0, f'mapper exit status {node.returncode}'
        finally:
            stop(player)
            stop(node)
    text = (logdir / 'node.log').read_text(errors='replace')
    final = re.findall(r'live_image_drops=(\d+) peak_image_buffer=(\d+)', text)
    assert final, 'missing final queue diagnostics'
    drops, peak = map(int, final[-1])
    assert peak <= 2, f'queue exceeded its cap: {peak}'
    if mode == 'live':
        assert drops > 0, 'burst did not exercise queue overflow; use a denser short bag'
    else:
        counts = re.findall(r'\[ DIRECT BAG \] finished: imu=(\d+) lidar=(\d+) img=(\d+)', text)
        assert counts and list(map(int, counts[-1])) == [expected[2], expected[1], expected[0]], counts
        assert drops == 0, 'direct feeder must preserve all input'
        assert text.count('SA-LIVO Joint Update Time') >= max(1, expected[0] - 4), 'direct replay stopped early'
    result = dict(mode=mode, drops=drops, peak_image_buffer=peak, input_counts=expected)
    (logdir / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ['binary', 'params', 'camera', 'bag', 'output']:
        p.add_argument('--' + key, type=Path, required=True)
    p.add_argument('--mode', choices=['both', 'live', 'direct'], default='both')
    p.add_argument('--timeout', type=float, default=60)
    p.add_argument('--topics', nargs=3, default=['/left_camera/image', '/livox/lidar', '/livox/imu'])
    args = p.parse_args()
    for key in ['binary', 'params', 'camera', 'bag', 'output']:
        setattr(args, key, getattr(args, key).resolve())
    dbs = list(args.bag.glob('*.db3')) if args.bag.is_dir() else [args.bag]
    counts = {}
    for db in dbs:
        with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as c:
            for topic, count in c.execute('select t.name,count(*) from messages m join topics t on m.topic_id=t.id group by t.name'):
                counts[topic] = counts.get(topic, 0) + count
    expected = [counts[t] for t in args.topics]
    for mode in (['live', 'direct'] if args.mode == 'both' else [args.mode]):
        run(args, mode, expected)


if __name__ == '__main__':
    main()
