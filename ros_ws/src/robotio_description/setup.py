from setuptools import setup
from glob import glob
import os
pkg = 'robotio_description'
share = os.path.join('share', pkg)
setup(
  name=pkg, version='0.0.1', packages=[],
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
    (share, ['package.xml']),
    (os.path.join(share, 'urdf'),   glob('robotio_description/urdf/*')),
    (os.path.join(share, 'config'), glob('robotio_description/config/*')),
  ],
  install_requires=['setuptools'], zip_safe=True,
  maintainer='jose', maintainer_email='you@example.com',
  description='RobotIO URDF/Xacro', license='Apache-2.0',
)
