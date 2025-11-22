from setuptools import setup, find_packages

package_name = 'astarplanner'

setup(
    name=package_name,
    version='0.0.0',
    packages=['astarplanner'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/astarplanner']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A* Planner Node for object avoidance',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_planner = astarplanner.astar_planner:main',
        ],
    },
)
