from setuptools import setup, find_packages

package_name = 'sciclops_driver'
install_requires = ['pyusb']

setup(
    name='sciclops_driver',
    version='0.0.2',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=install_requires,
    zip_safe=True,
    python_requires=">=3.8",
    maintainer='Rafael Vescovi and Abe Stoka',
    maintainer_email='ravescovi@anl.gov',
    description='Driver for the Sciclops',
    url='https://github.com/AD-SDL/sciclops_module.git', 
    license='MIT License',
    entry_points={},
)
