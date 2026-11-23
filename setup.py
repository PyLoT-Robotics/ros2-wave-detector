from glob import glob
from setuptools import setup,find_packages
setup(name='wave_detector',version='0.1.0',description='Temporal waving person detection with optional local YOLO poses',python_requires='>=3.10',
      packages=find_packages(exclude=['test']),install_requires=['numpy'],extras_require={'test': ['pytest'], 'yolo': ['ultralytics']},
      license='Apache-2.0',maintainer='PyLoT Robotics',maintainer_email='163533636+PyLoT-Robotics@users.noreply.github.com',
      data_files=[('share/ament_index/resource_index/packages',['resource/wave_detector']),
                  ('share/wave_detector',['package.xml']),('share/wave_detector/config',glob('config/*')),
                  ('share/wave_detector/launch',glob('launch/*.py'))],
      entry_points={'console_scripts':['ros2-wave-detector = wave_detector.cli:main','wave_detector_node = wave_detector.node:main']})
