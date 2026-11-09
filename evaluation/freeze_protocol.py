#!/usr/bin/env python3
"""Freeze the tested protocol before evaluation. No ROS or robot connection."""
from datetime import datetime,timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import re
import shutil


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def package_files(root):
    return sorted(p.name for p in root.iterdir()
                  if p.is_file() and p.suffix in ('.py','.json','.md')
                  and p.name!='freeze_manifest.json')


def environment():
    versions={}
    for package in ('numpy','robomaster','opencv-python','opencv-contrib-python','opencv-contrib-python-headless'):
        try:versions[package]=importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:pass
    return dict(python=platform.python_version(),packages=versions)


def verify(root):
    path=root/'freeze_manifest.json'
    if not path.is_file():raise ValueError('Protocol is not frozen. Run freeze_protocol.py first.')
    manifest=json.loads(path.read_text())
    if set(package_files(root))!=set(manifest['package_sha256']):
        raise ValueError('Package files changed after freeze. Do not modify this experiment mid-evaluation.')
    for name,expected in manifest['package_sha256'].items():
        if digest(root/name)!=expected:raise ValueError('Frozen file changed: '+name)
    for name,expected in manifest['external_sha256'].items():
        p=Path(name)
        if not p.is_file() or digest(p)!=expected:raise ValueError('External robot/observer file changed: '+name)
    snapshots={str(p.relative_to(root)):digest(p) for p in (root/'frozen_robot_sources').rglob('*') if p.is_file()}
    if snapshots!=manifest['snapshot_sha256']:raise ValueError('Frozen source snapshots changed.')
    if environment()!=manifest['environment']:
        raise ValueError('Python/package versions differ from freeze. Use the same robot virtual environment.')
    return manifest,digest(path)


def planned_slot(run_id,method):
    match=re.fullmatch(r'eval_([uf])(0[1-5])(?:_retry(0[1-9]|[1-9][0-9]))?',run_id)
    if not match:raise ValueError('Use eval_u01..eval_u05 or eval_f01..eval_f05; retries append _retry01 etc.')
    expected='unfiltered' if match[1]=='u' else 'filtered'
    if method!=expected:raise ValueError('Method does not match the evaluation run ID.')
    return dict(slot='eval_'+match[1]+match[2],attempt=1 if match[3] is None else int(match[3])+1)


def main():
    root=Path(__file__).resolve().parent
    if (root/'freeze_manifest.json').exists():
        _,sha=verify(root);print('Already frozen; verified. Nothing changed.');print('Manifest SHA256:',sha);return
    p=root/'protocol.json';c=root/'ekf_config.json'
    protocol=json.loads(p.read_text());cfg=json.loads(c.read_text())
    from feedback_core import Session
    from ekf_core import EKF
    EKF(cfg);Session('unfiltered',.60,False,protocol,cfg,0.)
    base=Path.home()/'rm_flatness_ws/src/rm_flatness_nav/rm_flatness_nav'
    paths=[base/'driver.py',base/'odometry_support.py',Path('/mnt/c/Users/khanm/Downloads/tag_observer.py')]
    installed=Path.home()/'rm_flatness_ws/build/rm_flatness_nav/rm_flatness_nav/driver.py'
    if not installed.is_file() or not paths[0].is_file() or digest(installed)!=digest(paths[0]):
        raise SystemExit('Built driver must match source. Rebuild and review before freezing.')
    paths.append(installed)
    # Metadata for the isolated observer OpenCV installation; no import/SDK init.
    paths.extend(sorted((Path.home()/'rm_tag_cv5_test').glob('opencv_contrib_python_headless-*.dist-info/METADATA')))
    if any(not x.is_file() for x in paths):raise SystemExit('A required driver/observer source file is missing.')
    if len(paths)<5:raise SystemExit('Isolated observer OpenCV metadata not found.')
    external={str(x):digest(x) for x in paths}
    known=json.loads((root/'commissioning_measurements.json').read_text())
    for run in known['runs']:
        source=Path.home()/'assignment3_tools/assignment3_feedback/feedback_runs'/run['run_id']/'metadata.json'
        if not source.is_file():raise SystemExit('Commissioning metadata missing: '+str(source))
        d=json.loads(source.read_text())
        if d['result']!='COMPLETED_COMMISSIONING' or d['stop_reason']!='RAW_TOF_GUARD':
            raise SystemExit('Commissioning evidence differs from reviewed completion: '+run['run_id'])
        if d['method']!=run['method'] or d['physical_start_gap_m']!=run['start_gap_m']:
            raise SystemExit('Commissioning identity/start measurement mismatch.')
        final=d.get('physical_final_gap_m')
        if final is not None and abs(final-run['final_gap_m'])>1e-9:
            raise SystemExit('Physical endpoint differs from confirmed measurement: '+run['run_id'])
        if d['source_sha256']['feedback_core.py']!=digest(root/'feedback_core.py'):
            raise SystemExit('Controller core differs from commissioned code.')
        for key,value in d['protocol'].items():
            if key in ('stage','evaluation_ready','calibration_frozen','guard_note'):continue
            if key in protocol and protocol[key]!=value:raise SystemExit('Protocol differs from commissioning: '+key)
        for key,value in d['ekf_config'].items():
            if key in ('stage','evaluation_ready','calibration_frozen','provenance'):continue
            if cfg.get(key)!=value:raise SystemExit('EKF configuration differs from commissioning: '+key)
    timestamp=datetime.now(timezone.utc).isoformat()
    snapshot=root/'frozen_robot_sources'
    shutil.copytree(base.parent,snapshot/'rm_flatness_nav',dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__','.git','*.backup_*'))
    snapshot.mkdir(exist_ok=True)
    shutil.copyfile(paths[2],snapshot/'tag_observer.py')
    protocol.update(stage='frozen_evaluation',evaluation_ready=True,calibration_frozen=True,
        guard_note='Frozen conservative raw-ToF guard after commissioning. Physical reference goal remains 0.20 m; early guard stops and their physical gap errors must be reported.')
    cfg.update(stage='frozen_evaluation',evaluation_ready=True,calibration_frozen=True)
    p.write_text(json.dumps(protocol,indent=2)+'\n');c.write_text(json.dumps(cfg,indent=2)+'\n')
    manifest=dict(frozen_utc=timestamp,package_sha256={name:digest(root/name) for name in package_files(root)},
        external_sha256=external,environment=environment(),
        snapshot_sha256={str(x.relative_to(root)):digest(x) for x in snapshot.rglob('*') if x.is_file()},
        nominal_start_gap_m=.60,minimum_start_gap_m=.60,maximum_start_gap_m=.62,
        planned_order=[f'eval_{m}{i:02d}' for i in range(1,6) for m in ('u','f')],
        commissioning_review=known,
        limitations=['Raw 0.25 m guard stops early relative to 0.20 m physical goal.',
          'Global ToF travel scale underestimates travel in the shorter commissioning runs.',
          'Noise/response assumptions are retained unchanged; no EKF improvement is presumed.',
          'Camera/tag pose and marked start must remain physically fixed; source hashes cannot verify physical placement.'])
    path=root/'freeze_manifest.json'
    with path.open('x') as f:f.write(json.dumps(manifest,indent=2)+'\n')
    _,sha=verify(root)
    print('Frozen: tested controller, calibration, Q/R, guard and protocol.')
    print('Nominal measured start: 0.60 m; raw ToF guard: 0.25 m; physical reference goal: 0.20 m.')
    print('Manifest SHA256:',sha)
    print('Evaluation order: '+' -> '.join(manifest['planned_order']))
    print('No robot commands sent. Keep these files unchanged during evaluation.')


if __name__=='__main__':main()
