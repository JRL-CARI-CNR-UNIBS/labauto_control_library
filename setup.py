from setuptools import setup, find_packages

setup(
    name='labauto',
    version='0.1.0',
    packages=find_packages(),  # should now detect 'labauto' folder
    include_package_data=True,
    install_requires=[
        'numpy',
        'scipy',
        'matplotlib',
        'pinocchio',
    ],
    author='Manuel Beschi',
    description='Control library in Python of the JRL‑CARI',
    long_description=open('README.md', encoding='utf-8').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/JRL-CARI-CNR-UNIBS/labauto_control_library',
    project_urls={
        "Lab Website": "https://cari.unibs.it/",
    },
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
        'License :: OSI Approved :: BSD License',
    ],
    python_requires='>=3.10',
)
