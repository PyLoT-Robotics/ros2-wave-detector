import numpy as np
import pytest
from wave_detector.core import WaveDetector,WaveConfig,position_from_depth
from wave_detector.cli import demo_frames


def points(x,raised=True,offset=0):
    p=np.zeros((17,3));p[5]=[35+offset,60,1];p[6]=[65+offset,60,1];p[9]=[x+offset,30 if raised else 90,1]
    return p


def feed(detector,xs,identity=1,raised=True):
    return [detector.update(identity,i*.1,points(x,raised),[0,0,100,200]) for i,x in enumerate(xs)]


def test_repeated_raised_hand_motion_detects():
    assert any(feed(WaveDetector(),[20,40,70,40,20,50,75]))


@pytest.mark.parametrize('xs,raised',[(list(range(10,80,10)),True),([30]*10,True),([20,40,70,40,20,50,75],False)])
def test_static_monotonic_and_lowered_hand_are_not_waves(xs,raised):
    assert not any(feed(WaveDetector(),xs,raised=raised))


def test_torso_translation_does_not_make_wave():
    d=WaveDetector()
    for i in range(10):assert d.update(1,i*.1,points(30,offset=i*10),[i*10,0,100+i*10,200]) is None


def test_person_ids_and_time_gaps_do_not_share_history():
    d=WaveDetector()
    for i,x in enumerate([20,40,70,40,20,50,75]):
        assert d.update(i,i*.1,points(x),[0,0,100,200]) is None
    assert d.update(1,10.,points(20),[0,0,100,200]) is None


def test_camera_motion_and_low_confidence_reset_history():
    d=WaveDetector();feed(d,[20,40,70,40,20])
    assert d.update(1,.5,points(70),[0,0,100,200],True) is None
    assert not d.tracks
    p=points(30);p[5,2]=.1
    assert d.update(1,.6,p,[0,0,100,200]) is None


def test_depth_uses_optical_frame_metres():
    assert position_from_depth(60,40,2.,100,100,50,50)==pytest.approx([.2,-.2,2.])
    with pytest.raises(ValueError):position_from_depth(60,40,-2,100,100,50,50)


def test_bad_pose_and_bad_configuration_rejected():
    with pytest.raises(ValueError):WaveDetector().update(1,0.,[],[0,0,100,200])
    with pytest.raises(ValueError):WaveConfig(min_samples=1)


def test_aligned_depth_sampling_and_units():
    from wave_detector.core import sample_person_depth
    image=np.full((30,30),5.);image[10:20,10:15]=2.
    assert sample_person_depth(image,[0,0,30,30])==2.
    assert sample_person_depth((image*1000).astype(np.uint16),[0,0,30,30],'mm')==2.
    image[:]=np.nan;assert sample_person_depth(image,[0,0,30,30]) is None


def test_local_model_must_exist_before_importing_inference_dependencies(tmp_path):
    from wave_detector.yolo import YoloPoseAdapter
    with pytest.raises(ValueError):YoloPoseAdapter(tmp_path/'missing.pt')
