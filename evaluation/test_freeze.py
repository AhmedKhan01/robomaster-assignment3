import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from freeze_protocol import digest,verify,planned_slot
from evaluation_status import inspect


class FreezeChecks(unittest.TestCase):
    def test_method_slot_and_retry(self):
        self.assertEqual(planned_slot('eval_f03_retry01','filtered'),dict(slot='eval_f03',attempt=2))
        with self.assertRaises(ValueError):planned_slot('eval_u01','filtered')
        with self.assertRaises(ValueError):planned_slot('commission_u01','unfiltered')

    def test_changed_frozen_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);p=root/'protocol.json';p.write_text('{}')
            manifest=dict(package_sha256={'protocol.json':digest(p)},external_sha256={},snapshot_sha256={},environment={'test':True})
            (root/'freeze_manifest.json').write_text(json.dumps(manifest))
            with patch('freeze_protocol.environment',return_value={'test':True}):
                verify(root)
                p.write_text('{"changed":true}')
                with self.assertRaises(ValueError):verify(root)

    def test_completed_slot_requires_physical_measurement(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);manifest=root/'freeze_manifest.json'
            manifest.write_text(json.dumps(dict(planned_order=['eval_u01','eval_f01'])))
            output=root/'feedback_runs';folder=output/'eval_u01';folder.mkdir(parents=True)
            (folder/'raw_events.csv').write_text('header\n')
            data=dict(run_id='eval_u01',method='unfiltered',planned_slot='eval_u01',evaluation_run=True,
                result='COMPLETED_EVALUATION',frozen_manifest_sha256=digest(manifest),
                physical_final_gap_m=None,physical_measurement_resolution_m=None,stop_reason='RAW_TOF_GUARD')
            p=folder/'metadata.json';p.write_text(json.dumps(data))
            _,records,completed=inspect(root,output)
            self.assertEqual(records[0]['status'],'NEEDS_PHYSICAL_ENDPOINT');self.assertFalse(completed)
            data.update(physical_final_gap_m=.25,physical_measurement_resolution_m=.001)
            p.write_text(json.dumps(data));self.assertEqual(inspect(root,output)[2],{'eval_u01'})


if __name__=='__main__':unittest.main()
