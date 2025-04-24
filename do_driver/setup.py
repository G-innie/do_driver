from setuptools import find_packages, setup

package_name = 'do_driver'

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
    maintainer='ginnie',
    maintainer_email='kingar@kth.se',
    description='DO driver messages',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'do_driver = do_driver.parse_do:main',
            'serial_reader_do = do_driver.serial_reader_do:main',
        ],
    },
)