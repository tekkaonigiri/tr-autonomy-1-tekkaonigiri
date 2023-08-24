from setuptools import setup

package_name = 'spinnything'

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
    maintainer='Michael Owens',
    maintainer_email='33632547+why-does-ie-still-exist@users.noreply.github.com',
    description='Test node for TR CV training',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spinnything = spinnything.spinnything:main'
        ],
    },
)
