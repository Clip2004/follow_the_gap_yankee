from setuptools import find_packages, setup

package_name = 'follow_the_gap_yankee'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clip2004',
    maintainer_email='felipe.mercado59@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gap_finder_yankee = follow_the_gap_yankee.gap_finder_yankee:main',
            'gap_controller_yankee = follow_the_gap_yankee.gap_controller_yankee:main',
        ],
    },
)
