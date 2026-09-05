from setuptools import find_packages, setup

package_name = 'depth'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],  # add other deps if needed
    zip_safe=True,
    maintainer='wasiq',
    maintainer_email='03337406603s@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'droid = depth.rtabmap_droidcam:main',
            'new_rtabmap_depth = depth.new_rtabmap_depth:main'
        ],
    },
)
