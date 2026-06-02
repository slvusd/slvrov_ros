import importlib


def test_core_web_scaffold_imports():
    package = importlib.import_module('slvrov_core_python.web')

    assert package.PACKAGE_AREA == 'web'
    importlib.import_module('slvrov_core_python.web.adapters')
    importlib.import_module('slvrov_core_python.web.nodes')
    importlib.import_module('slvrov_core_python.web.routes')


def test_core_science_scaffold_imports():
    package = importlib.import_module('slvrov_core_python.science')

    assert package.PACKAGE_AREA == 'science'
    importlib.import_module('slvrov_core_python.science.capture')
    importlib.import_module('slvrov_core_python.science.data_logging')
    importlib.import_module('slvrov_core_python.science.media')
