"""Synthetic 5B1 reader/profile cost and baseline bounds reproduction.

No mmap connection, game access or installation writes. Times are machine-local
measurements, not a hard realtime guarantee. Baseline source is read from Git.
"""
import argparse
import json
from pathlib import Path
import pickle
import statistics
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.sdk.scs_sdk import SCSTelemetry
from core.sdk.vehicle_observation import capture_vehicle_observation
from core.vehicle_profile import VehicleProfileProvider
from tests.test_stage5b1_vehicle_profile import memory, entry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--iterations', type=int, default=500)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT) or not 50 <= args.iterations <= 10000:
        parser.error('output must be inside repository; iterations 50..10000')
    source = subprocess.check_output(['git', 'show', 'HEAD:core/sdk/scs_sdk.py'], cwd=ROOT, text=True)
    namespace = {'__name__': '_stage5b1_baseline_sdk'}
    exec(compile(source, 'git:HEAD:core/sdk/scs_sdk.py', 'exec'), namespace)
    baseline = namespace['SCSTelemetry']
    report = {'schema_version': 1, 'synthetic_only': True,
              'iterations': args.iterations, 'cases': [], 'bounds_reproduction': {}}
    for label, cls in (('before', baseline), ('after', SCSTelemetry)):
        r = cls(); r.mm = memory(1)
        report['bounds_reproduction'][label] = {
            str(i): r.read_trailer(i) == {} for i in (-1, 10, True)}
    for trailers in (0, 1, 4, 10):
        for label, cls in (('before_reader', baseline), ('after_reader_profile_pickle', SCSTelemetry)):
            reader = cls(); reader.mm = memory(trailers)
            o = capture_vehicle_observation(reader.mm)
            owner = VehicleProfileProvider({'schema_version': 1, 'profiles': [entry(o)]})
            times, size = [], 0
            for i in range(args.iterations+20):
                struct.pack_into('<Q', reader.mm, 8, 1_000_000+i*16_667)
                start = time.perf_counter()
                raw = reader.update()
                if label.startswith('after'):
                    p = owner.update(raw['vehicleObservation'], time.monotonic())
                    size = len(pickle.dumps(p))
                elapsed = (time.perf_counter()-start)*1000
                if i >= 20:
                    times.append(elapsed)
            times.sort()
            report['cases'].append(dict(label=label,trailers=trailers,
                median_ms=statistics.median(times),p95_ms=times[int(.95*(len(times)-1))],
                max_ms=max(times),profile_pickle_bytes=size))
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
