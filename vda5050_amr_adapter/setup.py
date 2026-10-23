from setuptools import find_packages, setup
from glob import glob

package_name = 'vda5050_amr_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name + "/config", glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Leandro Pineda',
    author_email="leandro@inorbit.ai",
    maintainer='TBD',
    maintainer_email='tbd@tbd.com',
    description='VDA5050 AMR adapter example',
    license='BSD Clause 3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amr_adapter = vda5050_amr_adapter.amr_adapter:main'
        ],
    },
)