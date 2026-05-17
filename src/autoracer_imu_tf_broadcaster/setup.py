from setuptools import setup


package_name = 'autoracer_imu_tf_broadcaster'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='car',
    maintainer_email='car@autoracer.local',
    description='Visualization-only IMU TF broadcaster for AutoRacer RViz views.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autoracer_imu_tf_broadcaster = autoracer_imu_tf_broadcaster.imu_tf_broadcaster:main',
        ],
    },
)
