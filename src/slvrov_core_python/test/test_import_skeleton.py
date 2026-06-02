import importlib


def test_core_web_scaffold_imports():
    package = importlib.import_module('slvrov_core_python.web')

    assert package.PACKAGE_AREA == 'web'
    importlib.import_module('slvrov_core_python.web.adapters')
    importlib.import_module('slvrov_core_python.web.nodes')
    importlib.import_module('slvrov_core_python.web.routes')


def test_core_control_package_imports():
    package = importlib.import_module('slvrov_core_python.control')

    assert package.PACKAGE_AREA == 'control'
