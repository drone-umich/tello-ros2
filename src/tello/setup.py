from setuptools import setup

package_name = 'tello'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tello']),
        ('share/tello', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardocastro2',
    maintainer_email='josee.castro@udem.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'drone1 = tello.node:main',
             'drone2 = tello.node2:main',
             'keyboard = tello.keyboard:main',
             'drone3 = tello.node4:main'
        ],
    },
)
