import importlib


def test_slvrov_science_python_imports():
    package = importlib.import_module('slvrov_science_python')

    assert package.PACKAGE_NAME == 'slvrov_science_python'


def test_slvrov_science_python_placeholder_packages_import():
    importlib.import_module('slvrov_science_python.capture')
    importlib.import_module('slvrov_science_python.data_logging')
    importlib.import_module('slvrov_science_python.media')
