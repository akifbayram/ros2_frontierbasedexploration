from setuptools import setup

package_name = 'autonomous_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'autonomous_exploration.timer',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abd',
    maintainer_email='abd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_go2 = autonomous_exploration.control_go2:main',
            'control_tb3 = autonomous_exploration.control_tb3:main',
            'control_tb4 = autonomous_exploration.control_tb4:main',
            'voronoi_tb4 = autonomous_exploration.voronoi_tb4:main',
            'timer = autonomous_exploration.timer:main',
        ],
    },
)
