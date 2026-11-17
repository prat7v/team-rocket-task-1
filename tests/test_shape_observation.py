import sys
from pathlib import Path
import cv2
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from observe_shape_sorting import classify_contour,match_shapes


@pytest.mark.parametrize('count,expected',[(3,'triangle'),(4,'square'),(6,'hexagon'),(80,'circle')])
@pytest.mark.parametrize('angle',[.13,.7,1.4])
@pytest.mark.parametrize('epsilon',[.03,.04])
def test_rotated_profiles(count,expected,angle,epsilon):
    theta=np.linspace(0,2*np.pi,count,endpoint=False)+angle
    contour=np.c_[30*np.cos(theta)+100,30*np.sin(theta)+75].astype(np.float32)
    assert classify_contour(contour,epsilon_fraction=epsilon)['shape']==expected


def test_elongated_rectangle_not_accepted_as_square():
    contour=np.array([[0,0],[100,0],[100,20],[0,20]],dtype=np.float32)
    assert classify_contour(contour)['shape']=='unknown'


def test_missing_duplicate_or_unverified_object_blocks_match():
    holes=[{'shape':'square'}]
    block={'expected_shape':'square','shape_verified':True}
    assert match_shapes([block],holes)[0]['status']=='MATCHED'
    assert match_shapes([],holes)[0]['status']!='MATCHED'
    assert match_shapes([block,block],holes)[0]['status']!='MATCHED'
    assert match_shapes([block|{'shape_verified':False}],holes)[0]['status']!='MATCHED'

import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from check_sync_telemetry import ADDRESS,FIELDS,IDS,LENGTH,SyncTelemetry,verify_idle


class Packet:
    def __init__(self,failure=None):
        self.failure=failure

    def syncReadTx(self,*args):
        assert args[1:]==(ADDRESS,LENGTH,list(IDS),len(IDS),False)
        return 0

    def readRx(self,port,i,length):
        data=bytearray(LENGTH)
        values={'position':-7 if i==15 else 3166,'voltage':120,'temperature':30}
        for name,value in values.items():
            address,size,signed=FIELDS[name]
            data[address-ADDRESS:address-ADDRESS+size]=value.to_bytes(size,'little',signed=signed)
        if i==15 and self.failure=='device_error':
            return data,0,128
        if i==15 and self.failure=='timeout':
            return [],-3001,0
        if i==15 and self.failure=='short':
            return data[:-1],0,0
        return data,0,0


def test_decodes_signed_gripper_position_and_all_idle_states():
    cycle=SyncTelemetry(None,Packet()).read_cycle()
    assert cycle[15]['position']==-7
    assert set(cycle)==set(IDS)
    verify_idle(cycle)


@pytest.mark.parametrize('failure',['device_error','timeout','short'])
def test_failed_motor_never_yields_partial_or_stale_cycle(failure):
    reader=SyncTelemetry(None,Packet())
    reader.read_cycle()
    reader.packet.failure=failure
    with pytest.raises(RuntimeError):
        reader.read_cycle()


@pytest.mark.parametrize('key,value',[('torque',1),('hardware_error',32),('watchdog',-1),
                                     ('velocity',1),('current',6),('voltage',99),('temperature',45)])
def test_idle_diagnostic_rejects_active_or_fault_states(key,value):
    cycle=SyncTelemetry(None,Packet()).read_cycle()
    cycle[11][key]=value
    with pytest.raises(RuntimeError):
        verify_idle(cycle)

import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from check_vertical_insertion_reach import vertical_radius_upper_bound
from inspect_robot_kinematics import forward_kinematics


def test_vertical_fk_samples_never_exceed_optimistic_bound():
    origins=[[12,0,17],[0,0,58],[24,0,128],[124,0,0]]
    bound=vertical_radius_upper_bound(origins)
    # Tool pitch is held vertical while shoulder/elbow and base vary widely.
    for q1 in (-2.,0.,1.3):
        for q2 in np.linspace(-1.5,1.5,7):
            for q3 in np.linspace(-1.5,1.4,7):
                q4=np.pi/2-q2-q3
                tool,_=forward_kinematics([q1,q2,q3,q4],origins,[126,0,0])
                assert np.linalg.norm(tool[:2,3]-[12,0]) <= bound+1e-9


def test_bound_excludes_gripper_length():
    origins=[[12,0,17],[0,0,58],[24,0,128],[124,0,0]]
    assert 254 < vertical_radius_upper_bound(origins) < 255
