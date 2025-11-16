from setuptools import setup
package_name = 'robotio_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # <-- marcador correcto
        ('share/' + package_name, ['package.xml']),                                   # <-- instala package.xml
        ('share/' + package_name + '/launch', ['launch/robotio_sim.launch.py']),
        ('lib/' + package_name, ['scripts/run_servo_to_controller']),                 # <-- tu wrapper
    ],
    install_requires=['setuptools'],
    zip_safe=True,
)
