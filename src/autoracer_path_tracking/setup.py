from glob import glob

from setuptools import setup


package_name = 'autoracer_path_tracking'
fixture_paths = glob('../../tools/acceptance/fixtures/stage2_paths/*.json')


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/fixtures/stage2_paths', fixture_paths),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autoracer',
    maintainer_email='autoracer@todo.todo',
    description='Phase-2 Pure Pursuit Ackermann path tracking nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_path_publisher = autoracer_path_tracking.test_path_publisher:main',
            'goal_path_publisher = autoracer_path_tracking.goal_path_publisher:main',
            'pure_pursuit_tracker = autoracer_path_tracking.pure_pursuit_tracker:main',
        ],
    },
)
