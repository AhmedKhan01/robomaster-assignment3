import math
import unittest
from pilot_core import check_health,command_at

class Tests(unittest.TestCase):
    def healthy(self):
        return dict(tof=dict(receipt=10,range=1.87,sensor_age=.02),
          odom=dict(receipt=10,position=(0,0),quaternion=(0,0,0,1),source_stamp=100),
          tag=dict(receipt=10,visible=True,target_id=3,error=-.01,source_stamp=100),
          driver=dict(receipt=10,status='READY'))
    def test_command_boundaries(self):
        self.assertEqual(command_at(0,10,.04,0),(.04,0))
        self.assertEqual(command_at(10,10,.04,.1),(0,0))
        self.assertEqual(command_at(11,10,.04,.1),(0,0))
        self.assertEqual(command_at(1,10,.04,1),(.04,.1))
        self.assertLess(command_at(1,10,.04,-.01)[1],0)
        with self.assertRaises(ValueError):command_at(1,10,.16,0)
    def test_health_and_publisher_counts(self):
        self.assertIsNone(check_health(10.05,100.05,self.healthy(),1,True))
        self.assertIsNone(check_health(10.05,100.05,self.healthy(),0,False))
        self.assertEqual(check_health(10.05,100.05,self.healthy(),2,True),'COMMAND_PUBLISHER_COUNT_2')
        self.assertEqual(check_health(10.05,100.05,self.healthy(),1,False),'EXISTING_COMMAND_PUBLISHER')
    def test_sensor_stops(self):
        for key in ['tof','odom','tag','driver']:
            data=self.healthy();data.pop(key)
            self.assertEqual(check_health(10.05,100.05,data,1,True),key.upper()+'_MISSING')
            data=self.healthy();data[key]['receipt']=9
            self.assertEqual(check_health(10.05,100.05,data,1,True),key.upper()+'_RECEIPT_STALE')
        for value,expected in [(.299,'RAW_TOF_GUARD_0p30m'),(math.nan,'TOF_INVALID')]:
            data=self.healthy();data['tof']['range']=value
            self.assertEqual(check_health(10.05,100.05,data,1,True),expected)
        data=self.healthy();data['driver']['status']='TOF_STOP_LATCHED'
        self.assertEqual(check_health(10.05,100.05,data,1,True),'DRIVER_TOF_STOP_LATCHED')
    def test_stale_source_cannot_hide_behind_fresh_receipt(self):
        for key in ['odom','tag']:
            data=self.healthy();data[key]['source_stamp']=99
            self.assertEqual(check_health(10.05,100.05,data,1,True),key.upper()+'_SOURCE_STALE')
        data=self.healthy();data['tof']['sensor_age']=.5
        self.assertEqual(check_health(10.05,100.05,data,1,True),'TOF_SOURCE_STALE')
    def test_tag_alignment_and_visibility(self):
        for field,value,reason in [('visible',False,'TAG3_NOT_VISIBLE'),('target_id',2,'TAG3_NOT_VISIBLE'),
                ('error',.2,'TAG_NOT_CENTERED')]:
            data=self.healthy();data['tag'][field]=value
            self.assertEqual(check_health(10.05,100.05,data,1,True),reason)

if __name__=='__main__':unittest.main()
