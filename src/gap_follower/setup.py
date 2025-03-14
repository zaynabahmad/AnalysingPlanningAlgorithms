from setuptools import find_packages, setup

package_name = 'gap_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy', 'visualization_msgs'],
    zip_safe=True,
    maintainer='zaynap',
    maintainer_email='zozaahmad203@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'naive_gap_follower = gap_follower.naive_gap_follower:main',
            'optimized_gap_follower = gap_follower.optimized_gap_follower:main',
            'optimized_gap_follower2 = gap_follower.optimized_gap_follower2:main',
            'wallfollower = gap_follower.wallfollower:main',
            'centerfollowing = gap_follower.centerfollowing:main',
            'dualwallstatemachine = gap_follower.dualwallstatemachine:main',
            'optimized_gapfolloweSM = gap_follower.optimized_gapfolloweSM:main',



        ],
    },
)
