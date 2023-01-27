from setuptools import setup

package_name = 'ripspy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='esoriano',
    maintainer_email='esoriano@gsyc.urjc.es',
    description='RIPS prototype',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'ripspy = ripspy.ripspy:main',
        ],
},

)
