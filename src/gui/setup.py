from setuptools import find_packages, setup

package_name = 'gui'

setup(
        name=package_name,
        version='0.1.0',
        packages=find_packages(),
        data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resources', [
        'gui/resources/map.html',
        ]),
            ('share/ament_index/resource_index/packages', ['resource/gui']),
            ('share/gui', ['package.xml']),
            ('share/gui/resources', ['gui/resources/map.html']),
            ('share/gui/resources/vendor/leaflet', [
                'gui/resources/vendor/leaflet/leaflet.js',
                'gui/resources/vendor/leaflet/leaflet.css',
            ]),
            ('share/gui/resources/vendor/leaflet/images', [
                'gui/resources/vendor/leaflet/images/marker-icon.png',
                'gui/resources/vendor/leaflet/images/marker-icon-2x.png',
                'gui/resources/vendor/leaflet/images/marker-shadow.png',
            ]),
        
        # Si incluyes Leaflet en repo, añade:
        # ('share/' + package_name + '/resources/vendor/leaflet', [
        # 'cuadriga_gui/resources/vendor/leaflet/leaflet.js',
        # 'cuadriga_gui/resources/vendor/leaflet/leaflet.css',
        # ]),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='you',
        maintainer_email='you@example.com',
        description='GUI ROS2 con Leaflet + FollowZED',
        license='MIT',
        tests_require=['pytest'],
        entry_points={
        'console_scripts': [
        'gui = gui.gui_node:main',
        ],
    },
)